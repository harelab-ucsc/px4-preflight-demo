#!/usr/bin/env python3
"""Offboard mission commander.

Streams OFFBOARD setpoints, arms the vehicle, switches it to OFFBOARD nav
state, then flies through a fixed list of waypoints in NED. After the last
waypoint is reached (or when the node is shut down), waypoint hit/miss
results are written to /tmp/waypoint_results.json so the runner's assertion
script can grade the run.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


WAYPOINTS = [
    (5.0, 0.0, -5.0),
    (5.0, 5.0, -5.0),
    (0.0, 5.0, -5.0),
    (0.0, 0.0, -2.0),
]
WAYPOINT_RADIUS = 0.5
RESULTS_PATH = "/tmp/waypoint_results.json"

ARMING_STATE_ARMED = 2
NAV_STATE_OFFBOARD = 14
VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


class OffboardMissionNode(Node):
    def __init__(self):
        super().__init__("offboard_mission")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self.cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self._on_status, qos
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._on_position,
            qos,
        )

        self.armed = False
        self.offboard = False
        self.position = None
        self.current_wp = 0
        self.setpoint_count = 0
        self.results = {
            "waypoints_hit": [False] * len(WAYPOINTS),
            "hit_count": 0,
            "total": len(WAYPOINTS),
            "pass": False,
            "reason": "mission did not complete",
        }

        self.timer = self.create_timer(0.1, self._control_loop)

    # ── Subscriptions ────────────────────────────────────────────────────
    def _on_status(self, msg):
        self.armed = msg.arming_state == ARMING_STATE_ARMED
        self.offboard = msg.nav_state == NAV_STATE_OFFBOARD

    def _on_position(self, msg):
        self.position = (msg.x, msg.y, msg.z)

    # ── Helpers ──────────────────────────────────────────────────────────
    def _now_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self.offboard_pub.publish(msg)

    def _publish_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float("nan")
        msg.timestamp = self._now_us()
        self.setpoint_pub.publish(msg)

    def _send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self.cmd_pub.publish(msg)

    # ── Main loop ────────────────────────────────────────────────────────
    def _control_loop(self):
        # Setpoints must stream continuously for OFFBOARD to stay engaged.
        self._publish_offboard_mode()

        if self.current_wp >= len(WAYPOINTS):
            self._finish(passed=True, reason="all waypoints reached")
            return

        wp = WAYPOINTS[self.current_wp]
        self._publish_setpoint(*wp)
        self.setpoint_count += 1

        # PX4 requires a few cycles of streaming setpoints before it accepts
        # the OFFBOARD mode switch + arm command.
        if self.setpoint_count == 10:
            self._send_command(
                VEHICLE_CMD_DO_SET_MODE, 1.0, PX4_CUSTOM_MAIN_MODE_OFFBOARD
            )
            self._send_command(VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        if self.position is not None:
            dx = self.position[0] - wp[0]
            dy = self.position[1] - wp[1]
            dz = self.position[2] - wp[2]
            if math.sqrt(dx * dx + dy * dy + dz * dz) < WAYPOINT_RADIUS:
                self.results["waypoints_hit"][self.current_wp] = True
                self.results["hit_count"] += 1
                self.get_logger().info(
                    f"waypoint {self.current_wp + 1}/{len(WAYPOINTS)} reached"
                )
                self.current_wp += 1

    def _finish(self, passed, reason):
        if self.results["pass"] and not passed:
            return  # already finished
        self.results["pass"] = bool(passed)
        self.results["reason"] = reason
        with open(RESULTS_PATH, "w") as f:
            json.dump(self.results, f, indent=2)
        self.get_logger().info(f"results written to {RESULTS_PATH}: {self.results}")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = OffboardMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.results["pass"]:
            node._finish(
                passed=False,
                reason=f"only {node.results['hit_count']}/{node.results['total']} "
                       "waypoints reached",
            )
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
