# Lite3 Robot Plugin for EMOS

A [Sugarcoat](https://automatika-robotics.github.io/sugarcoat/)-ed robot plugin
for the **DeepRobotics Lite3** quadruped that integrates the Lite3 with
[EMOS](https://automatikarobotics.com/emos/). It speaks the Lite3's **Motion
Host UDP protocol** directly from inside the Sugarcoat plugin framework, so no
separate bridge process is required — an EMOS recipe using `Lite3Plugin` talks
to the robot on its own.

## The Lite3 control surface

| Direction | Endpoint             | Wire format                                                                                 |
| :-------- | :------------------- | :------------------------------------------------------------------------------------------ |
| Commands  | UDP → robot `:43893` | `SimpleCMD` (3×int32) / `ComplexCMD` (3×int32 + double)                                     |
| Telemetry | UDP ← bind `:43897`  | `RobotStateReceived` (code 2305), `JointStateReceived` (2306), `HandleStateReceived` (2309) |

`protocol.py` defines the wire layout with `ctypes` (`_pack_ = 4`), so the
on-the-wire bytes match the Lite3 Motion Host structs field-for-field and packet
sizes can be used for type dispatch. `codecs.py` encodes commands and parses
telemetry.

## Installation

On a robot running EMOS, install the plugin from the catalog:

```bash
emos plugin install emos-plugin-lite3
```

This builds the plugin and resolves the sensor drivers its manifest
(`emos-plugin.yaml`) declares. See the
[EMOS plugin guide](https://emos.automatikarobotics.com/getting-started/plugins.html).

## What the plugin exposes

- **Feedback** — binds UDP `:43897` and decodes the robot's telemetry into ten
  streams (the feedback key is in brackets; see
  [Usage in Recipes](#usage-in-recipes) for binding a topic to it). From the
  `RobotState` packet:
  - leg odometry, standard `nav_msgs/Odometry` (key `Odometry`).
  - body IMU, built-in `Imu` wrapping `sensor_msgs/Imu` (key `Imu`).
  - battery percentage, `std_msgs/Float64` (key `battery`).
  - front / back ultrasonic distance, built-in `Range` wrapping
    `sensor_msgs/Range` (keys `ultrasound_front`, `ultrasound_back`). Their
    frames are placed on the body by the plugin's `mounts` (nose and tail of
    the trunk, the rear beam turned by pi), which the launcher publishes as
    static transforms `body -> ultrasound_front` / `body -> ultrasound_back`,
    so a consumer can tell which way each beam faces.
  - human-readable status token (`sitting`, `standing`, `walking_flat_fast`,
    `long_jump`, ...), `std_msgs/String` (key `robot_status`).
  - balance flag — `True` while the robot can hold its balance, `False` when
    an external force has disturbed it, `std_msgs/Bool` (key `is_balanced`).
  - fallen flag — `True` in a lose-control-protection or flipping-over state,
    `std_msgs/Bool` (key `is_fallen`).

  From the `JointState` and `HandleState` packets:
  - the 12 leg-joint angles, built-in `JointState` wrapping `sensor_msgs/JointState` (key `JointState`).
  - the operator joystick command as a `geometry_msgs/Twist` (left stick →
    linear x/y, right stick → yaw), built-in `Twist` (key `handle`).

- **Sensors** — the Mid-360 LiDAR and the RealSense camera ship their own ROS
  drivers, so their data stays on native ROS topics; the plugin declares the
  streams and starts the drivers (see [Sensor drivers](#sensor-drivers)):
  - Mid-360 point cloud, `PointCloud2` on `/livox/lidar` (key `lidar`).
  - Mid-360 built-in IMU, `Imu` on `/livox/imu` (key `lidar_imu`), for
    LiDAR-inertial odometry. Not to be confused with the body IMU (key `Imu`)
    above.
  - RealSense colour image and intrinsics, `Image` / `CameraInfo` on
    `/camera/color/image_raw` / `/camera/color/camera_info` (keys `camera`,
    `camera_info`).
  - RealSense synchronised colour + depth, `RGBD` on `/camera/rgbd` (key
    `rgbd`). Exposed only when `realsense2_camera_msgs` is installed.

- **Commands**
  - `Twist` — a standard `Twist` output is encoded to the Lite3's three
    `ComplexCMD` velocity packets (codes 320 / 325 / 321) and sent to UDP `:43893`.
  - `Audio` — an `Audio` output (e.g. from a TextToSpeech component) is
    decoded to raw PCM and streamed to the Motion Host speaker over UDP
    (see [Audio output](#audio-output)).
- **Actions** — named `SimpleCMD` behaviours, exposed as
  `plugin.actions.<name>()`: `sit_stand`, `say_hello`, `twist`, `twist_jump`,
  `moonwalk`, `long_jump`, `stand_zero`, `set_pose_mode`, `set_move_mode`,
  `set_manual_mode`, `set_navigation_mode`, `gait_slow` / `gait_medium` /
  `gait_fast`, `save_data`, `stop`. Each reports `(success, message)`: whether
  the command left the socket. Keyword arguments given to a factory go to the
  `Action` it builds (see
  [Confirming an action worked](#confirming-an-action-worked)). Each also
  carries a tool description with state-machine guidance, so an LLM-driven
  monitor knows when an action is valid to invoke.
- **Events**, exposed as `plugin.events.<name>(...)`:
  - `low_battery(threshold=20.0)` — battery drops below `threshold` percent.
  - `obstacle_ahead(threshold=0.5)` — front ultrasonic distance drops below
    `threshold` metres.
  - `balance_disturbed()` — the robot can no longer hold its balance and must
    step to recover.
  - `fallen()` — the robot has lost its footing (lose-control-protection or
    flipping-over state).
- **Heartbeat** — the `0x21040001` keep-alive is sent at 4 Hz while the plugin
  is active, so the robot retains external control.
- **Shutdown** — when the recipe ends, cleanly or on Ctrl+C, the plugin
  switches the robot back to manual control, handing it back to the
  operator's handset.
- **Mapping** — the plugin declares how the robot's environment is mapped (see
  [Mapping](#mapping)).

## Usage in Recipes

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

A component topic with `use_plugin=True` is bound to one of the plugin's
streams: to the feedback or command whose key matches the topic's name, or,
failing that, to the only one with the topic's message type. So an `Odometry`
input receives the robot's leg odometry whatever it is called, and a `Twist`
output is encoded and sent to the Lite3 over UDP. Where the plugin has several
streams of one type (`Imu`, `Range`, `Bool`), name the topic after the key:

```python
body_imu = Topic(name="Imu", msg_type="Imu", use_plugin=True)
lidar_imu = Topic(name="lidar_imu", msg_type="Imu", use_plugin=True)
front = Topic(name="ultrasound_front", msg_type="Range", use_plugin=True)
```

With `multiprocessing=True`,
Sugarcoat rebuilds the plugin in each component subprocess from a JSON spec
and fans telemetry out over a localhost socket — no extra configuration.

### Confirming an action worked

The Lite3 never acknowledges a command, so an action's `(success, message)`
only says the command was sent. Keyword arguments given to an action factory
go to the `Action` it builds, so a recipe can have an action wait for the robot's own telemetry to confirm it:

```python
status = plugin.feedbacks["robot_status"].as_topic()

stand_up = plugin.actions.sit_stand(
    success=status.msg.data == "standing",
    timeout=10.0,
)
launcher.on(start_event, stand_up)
```

The action then succeeds only once the robot reports `standing`, and fails if
it has not within 10 s. Build it after the plugin is attached to the launcher,
as with events.

Do not give `sit_stand` retries: it toggles, so a second send while the robot
is still getting up can sit it back down.

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

Available network class attributes: `MOTION_HOST_IP`, `COMMAND_PORT`,
`TELEMETRY_PORT`, `BIND_HOST`, `VEL_X_FACTOR`, `SEND_HEARTBEAT`. The sensor
attributes are listed under [Sensor drivers](#sensor-drivers).

## Sensor drivers

The launcher starts the LiDAR and camera drivers only for a recipe that binds
one of their feedbacks, so a recipe that never reads the point cloud never runs
`livox_ros_driver2`. The launcher owns the processes: they respawn if they die
and stop with the recipe.

## Mapping

DeepRobotics ships no mapping tool on the Lite3, so the plugin declares
`NativeMapping`: EMOS builds the map itself from the Mid-360's point cloud
(`lidar`) and built-in IMU (`lidar_imu`). When the 3D map is flattened to an
occupancy grid, only points up to 0.40 m (the robot's height) count as
obstacles. The declaration is `plugin.MAPPING`, and appears under `"mapping"`
in `python -m ros_sugar.robot inspect lite3_plugin:Lite3Plugin`.

## Command codes

Codes follow the _Jueying Lite3 Motion Host Communication Interface_ document;
see `protocol.py::CommandCode`. Velocity is sent as three `ComplexCMD` packets
(the small numeric codes); every other command is a `SimpleCMD`.

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

## Audio output

On the Lite3, EMOS runs on the compute board but the speaker is on the Motion
Host. The plugin exposes an `Audio` command so a recipe doesn't need to know
this: give a TextToSpeech (or any `Audio`-output) component's output topic
`use_plugin=True` and the plugin streams the audio to the Motion Host speaker.

```python
from agents.components import TextToSpeech
from agents.ros import Topic

tts = TextToSpeech(
    inputs=[Topic(name="text_in", msg_type="String")],
    outputs=[Topic(name="audio", msg_type="Audio", use_plugin=True)],
    model_client=tts_client,
    component_name="tts",
)
```

The plugin decodes the audio blob and streams it as raw **mono F32LE PCM** over
UDP to `MOTION_HOST_IP:5005`. Nothing on the wire carries the sample rate, so
the receiver must be told it — `AUDIO_SAMPLE_RATE` (default `24000`, the native
rate of the local sherpa-onnx Kokoro TTS model) must match the TTS model's
output rate and the receiver's caps.

The Motion Host plays the stream with a gstreamer receiver:

```bash
gst-launch-1.0 -v udpsrc port=5005 \
  caps="audio/x-raw,format=F32LE,channels=1,rate=24000" \
  ! queue ! audioconvert ! audioresample ! alsasink
```

Add `device=hw:0` to `alsasink` to target a specific ALSA device. Run it as a
normal user in the `audio` group (not `sudo`). There is no documented
direct-PCM endpoint on the Lite3, so this receiver process must run on the
host the speaker is attached to.

Audio class attributes, overridable by subclass: `AUDIO_HOST` (defaults to
`MOTION_HOST_IP`), `AUDIO_PORT`, `AUDIO_SAMPLE_RATE`, `AUDIO_BLOCK_SIZE`.

## EMOS robot config

The plugin auto-builds a `RobotConfig` (DIFFERENTIAL_DRIVE drive model, BOX
footprint of ~61×37×40 cm, sensible Lite3 velocity/acceleration limits) and
exposes it as `plugin.robot_config`. Sugarcoat's `Launcher` picks this up at
`bringup` and broadcasts it to the components that consume it (e.g. the Kompass
planner / controller) -- so recipes don't need to construct a `RobotConfig`
themselves:

```python
from ros_sugar.launch import Launcher
from lite3_plugin import Lite3Plugin

launcher = Launcher(robot_plugin=Lite3Plugin())
launcher.add_pkg(components=[planner, controller, drive_manager], multiprocessing=True)
# no `launcher.robot = ...` needed -- the plugin provides it
launcher.bringup()
```

A recipe can still override with `launcher.robot = my_overridden_config`;
explicit recipe wins.

Override the geometry / limits in a subclass when defaults don't match a
particular unit:

```python
class TunedLite3(Lite3Plugin):
    ROBOT_GEOMETRY_PARAMS = (0.61, 0.37, 0.4)   # [length, width, height] in metres
    ROBOT_VX_MAX = 0.6   # safety-capped
```

Available robot-model class attributes: `ROBOT_DRIVE_TYPE`, `ROBOT_GEOMETRY_TYPE`,
`ROBOT_GEOMETRY_PARAMS`, `ROBOT_VX_MAX` / `ROBOT_VX_ACC` / `ROBOT_VX_DECEL`,
`ROBOT_OMEGA_MAX` / `ROBOT_OMEGA_ACC` / `ROBOT_OMEGA_DECEL`, `ROBOT_STEER_MAX`.
