# Lite3 Robot Plugin for EMOS

A [Sugarcoat](https://automatika-robotics.github.io/sugarcoat/)-ed robot plugin
for the **DeepRobotics Lite3** quadruped — a class-based conversion of
DeepRobotics' `message_transformer` ROS2 package that formally integrates the
Lite3 with [EMOS](https://automatikarobotics.com/emos/).

## What this replaces

DeepRobotics' `message_transformer` ships two C++ bridge nodes (`qnx2ros`,
`ros2qnx`) that translate between the Lite3's **Motion Host UDP protocol** and
ROS topics. This plugin folds that translation into the Sugarcoat plugin
framework: it speaks the UDP protocol _directly_, so no separate bridge process
is required — an EMOS recipe using `Lite3Plugin` talks to the robot on its
own.

## The Lite3 control surface

| Direction | Endpoint             | Wire format                                                                                 |
| :-------- | :------------------- | :------------------------------------------------------------------------------------------ |
| Commands  | UDP → robot `:43893` | `SimpleCMD` (3×int32) / `ComplexCMD` (3×int32 + double)                                     |
| Telemetry | UDP ← bind `:43897`  | `RobotStateReceived` (code 2305), `JointStateReceived` (2306), `HandleStateReceived` (2309) |

`protocol.py` mirrors the DeepRobotics C structs with `ctypes` (`_pack_ = 4`),
so the on-the-wire layout matches `message_transformer/include/protocol.h`
field-for-field. `codecs.py` encodes commands and parses telemetry.

## What the plugin exposes

- **Feedback** — binds UDP `:43897`, decodes `RobotState` packets into three
  streams (the registry key a recipe passes to `Topic(use_plugin=...)` is in
  brackets):
  - `Odometry` — leg odometry, standard `nav_msgs/Odometry` (key `Odometry`).
  - `Imu` — body IMU, custom `SupportedType` via `create_supported_type`
    (key `Imu`).
  - battery percentage as `std_msgs/Float64` (key `battery`).
- **Commands** — a standard `Twist` output is encoded to the Lite3's three
  `ComplexCMD` velocity packets (codes 320 / 325 / 321) and sent to UDP `:43893`.
- **Actions** — named `SimpleCMD` behaviours, exposed as
  `plugin.actions.<name>()`: `sit_stand`, `say_hello`, `twist`, `twist_jump`,
  `moonwalk`, `long_jump`, `stand_zero`, `set_pose_mode`, `set_move_mode`,
  `set_manual_mode`, `set_navigation_mode`, `gait_slow` / `gait_medium` /
  `gait_fast`, `save_data`, `stop`.
- **Events** — `plugin.events.low_battery(threshold=20.0)`.
- **Heartbeat** — the `0x21040001` keep-alive is sent at 4 Hz while the plugin
  is active, so the robot retains external control.

## Usage

`Lite3Plugin` follows Sugarcoat's standard plugin contract: a **zero-argument
constructor** with every robot-specific endpoint baked in as a class attribute.
The recipe author writes nothing about IPs, ports, or wire formats:

```python
from ros_sugar.launch import Launcher
from lite3_plugin import Lite3Plugin

plugin = Lite3Plugin()
launcher = Launcher(robot_plugin=plugin)
launcher.add_pkg(components=[planner, controller], multiprocessing=True)

# React to the robot's own events with its own actions
launcher.on(plugin.events.low_battery(15.0), plugin.actions.sit_stand())

launcher.bringup()
```

A component that declares an `Odometry` or `Imu` input transparently receives
the robot's decoded telemetry; a component that publishes a `Twist` has its
output encoded and sent to the Lite3 over UDP. With `multiprocessing=True`,
Sugarcoat rebuilds the plugin in each component subprocess from a JSON spec
and fans telemetry out over a localhost socket — no extra configuration.

### Overriding for a specific unit

For testing on localhost, a non-default subnet, or per-unit calibration,
**subclass** and override the class attributes:

```python
from lite3_plugin import Lite3Plugin

class MyLite3(Lite3Plugin):
    MOTION_HOST_IP = "10.0.0.42"   # robot lives on a different subnet
    VEL_X_FACTOR = 0.85            # this unit's forward calibration

plugin = MyLite3()
```

Available class attributes: `MOTION_HOST_IP`, `COMMAND_PORT`, `TELEMETRY_PORT`,
`BIND_HOST`, `VEL_X_FACTOR`, `SEND_HEARTBEAT`.

## Command codes

Codes are taken from the DeepRobotics `message_transformer` README and the
_Jueying Lite3 Motion Host Communication Interface_ document; see
`protocol.py::CommandCode`. The `ctypes` struct layout mirrors
`message_transformer/include/protocol.h`.

## Republishing feedback as ROS2 topics

Feedback decoded from a non-ROS transport lives on Sugarcoat's internal feedback
bus — components in the same launcher read it via `Topic(use_plugin=...)`, but
nothing outside the process can. `republishers.py` bridges that gap:
`Lite3FeedbackPublisher` is a component that republishes the `Odometry` and
`Imu` feedbacks on real ROS2 topics (`/odom`, `/imu/data`) and broadcasts an
`odom` → `body` TF. Add it to a launcher like any other component:

```python
from lite3_plugin import Lite3Plugin, Lite3FeedbackPublisher

launcher = Launcher(robot_plugin=Lite3Plugin())
launcher.add_pkg(components=[Lite3FeedbackPublisher()])
launcher.bringup()
```

`FeedbackRepublisher` is the generic base — pass it `FeedbackBridge` entries to
bridge any plugin's feedbacks.

## Extending

The Lite3 also streams `JointState` (code 2306) and `HandleState` (2309)
packets — `codecs.parse_joint_state` / `codecs.parse_handle_state` already
decode them. Add `Feedback` entries (e.g. a `create_supported_type`-wrapped
`JointState`, or `HandleState` as a `Twist`) to expose them, following the IMU
feedback as a template.
