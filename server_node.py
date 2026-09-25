"""Mock DeepRobotics Lite3 for exercising the plugin without hardware.

Speaks the Lite3 Motion Host UDP protocol from the robot's side: streams
``RobotState`` telemetry packets, and receives + logs ``SimpleCMD`` /
``ComplexCMD`` command packets (integrating velocity commands into a simple
pose so the telemetry reflects what was commanded).

    python3 server_node.py --command-port 43893 --telemetry-port 43897

Run it alongside a Sugarcoat recipe that uses ``Lite3Plugin`` with matching
host/ports, or on its own to sanity-check the wire protocol.
"""

import argparse
import ctypes
import math
import socket
import threading
import time

from lite3_plugin import protocol
from lite3_plugin.protocol import CommandCode


class MockLite3:
    """A localhost UDP stand-in for the Lite3 Motion Host."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        command_port: int = protocol.DEFAULT_COMMAND_PORT,
        telemetry_port: int = protocol.DEFAULT_TELEMETRY_PORT,
        telemetry_rate_hz: float = 100.0,
    ):
        self.telemetry_addr = (host, telemetry_port)
        self.command_port = command_port
        self.telemetry_period = 1.0 / telemetry_rate_hz
        self._stop = threading.Event()

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, command_port))
        self._sock.settimeout(0.5)

        # Integrated pose + commanded velocity.
        self._x = self._y = self._yaw_deg = 0.0
        self._vx = self._vy = self._wz = 0.0
        self._battery = 100.0

    def run(self) -> None:
        """Stream telemetry on a thread; receive commands on this thread."""
        tx = threading.Thread(target=self._telemetry_loop, daemon=True)
        tx.start()
        print(
            f"[mock lite3] streaming telemetry to {self.telemetry_addr}, "
            f"listening for commands on :{self.command_port}"
        )
        try:
            while not self._stop.is_set():
                try:
                    data, _ = self._sock.recvfrom(1024)
                except socket.timeout:
                    continue
                self._handle_inbound(data)
        except KeyboardInterrupt:
            pass
        finally:
            self._stop.set()
            self._sock.close()
            print("[mock lite3] stopped")

    def _handle_inbound(self, data: bytes) -> None:
        if len(data) == ctypes.sizeof(protocol.ComplexCMD):
            cmd = protocol.ComplexCMD.from_buffer_copy(data)
            if cmd.cmd_code == CommandCode.VEL_FORWARD:
                self._vx = cmd.data
            elif cmd.cmd_code == CommandCode.VEL_LATERAL:
                self._vy = cmd.data
            elif cmd.cmd_code == CommandCode.VEL_YAW:
                self._wz = -cmd.data  # the plugin negates yaw on the wire
            print(
                f"[mock lite3] <- ComplexCMD code={cmd.cmd_code} data={cmd.data:.3f}"
            )
        elif len(data) == ctypes.sizeof(protocol.SimpleCMD):
            cmd = protocol.SimpleCMD.from_buffer_copy(data)
            if cmd.cmd_code == CommandCode.HEARTBEAT:
                print("[mock lite3] <- heartbeat")
            else:
                print(
                    f"[mock lite3] <- SimpleCMD code=0x{cmd.cmd_code & 0xFFFFFFFF:08X} "
                    f"value={cmd.cmd_value}"
                )
        else:
            print(f"[mock lite3] <- unrecognized packet ({len(data)} bytes)")

    def _telemetry_loop(self) -> None:
        last = time.time()
        while not self._stop.is_set():
            now = time.time()
            dt = now - last
            last = now
            # Integrate the commanded velocity into a world pose.
            yaw = math.radians(self._yaw_deg)
            self._x += (
                self._vx * math.cos(yaw) - self._vy * math.sin(yaw)
            ) * dt
            self._y += (
                self._vx * math.sin(yaw) + self._vy * math.cos(yaw)
            ) * dt
            self._yaw_deg += math.degrees(self._wz) * dt
            self._battery = max(0.0, self._battery - 0.01 * dt)
            self._sock.sendto(self._build_robot_state(), self.telemetry_addr)
            # The robot interleaves the other two reports in the same stream
            self._sock.sendto(self._build_joint_state(), self.telemetry_addr)
            self._sock.sendto(self._build_handle_state(), self.telemetry_addr)
            time.sleep(self.telemetry_period)

    def _build_robot_state(self) -> bytes:
        frame = protocol.RobotStateReceived()
        frame.code = protocol.ROBOT_STATE_CODE
        frame.size = ctypes.sizeof(protocol.RobotState)
        frame.cons_code = 0
        state = frame.data
        state.pos_world[0] = self._x
        state.pos_world[1] = self._y
        state.pos_world[2] = 0.0
        state.rpy[2] = self._yaw_deg
        state.vel_body[0] = self._vx
        state.vel_body[1] = self._vy
        state.rpy_vel[2] = self._wz
        state.battery_level = self._battery
        state.is_charging = False
        return bytes(frame)

    def _build_joint_state(self) -> bytes:
        """The 12 leg joints, standing still in a plausible crouch."""
        frame = protocol.JointStateReceived()
        frame.code = protocol.JOINT_STATE_CODE
        frame.size = ctypes.sizeof(protocol.JointState)
        frame.cons_code = 0
        for leg in ("LF", "RF", "LB", "RB"):
            setattr(frame.data, leg + "_Joint", 0.0)      # HipX
            setattr(frame.data, leg + "_Joint_1", -0.8)   # HipY
            setattr(frame.data, leg + "_Joint_2", 1.6)    # Knee
        return bytes(frame)

    def _build_handle_state(self) -> bytes:
        """The operator's sticks, centred: the handset reports even when idle."""
        frame = protocol.HandleStateReceived()
        frame.code = protocol.HANDLE_STATE_CODE
        frame.size = ctypes.sizeof(protocol.HandleState)
        frame.cons_code = 0
        return bytes(frame)


def main() -> None:
    parser = argparse.ArgumentParser(description="Mock UDP DeepRobotics Lite3")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument(
        "--command-port", type=int, default=protocol.DEFAULT_COMMAND_PORT
    )
    parser.add_argument(
        "--telemetry-port", type=int, default=protocol.DEFAULT_TELEMETRY_PORT
    )
    parser.add_argument("--telemetry-rate", type=float, default=100.0)
    args = parser.parse_args()

    MockLite3(
        host=args.host,
        command_port=args.command_port,
        telemetry_port=args.telemetry_port,
        telemetry_rate_hz=args.telemetry_rate,
    ).run()


if __name__ == "__main__":
    main()
