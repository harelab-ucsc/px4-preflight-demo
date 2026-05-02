#!/usr/bin/env python3
"""Post-run assertion for the offboard waypoint test case.

Invoked by px4-preflight-runner after run_sim.sh has shut PX4 down.
Reads the JSON results dropped by offboard_mission_node and (optionally)
cross-checks the ULog to confirm the vehicle actually armed and entered
OFFBOARD nav state at some point.

Usage: waypoint_check.py <results_json> [<ulg_path>]
"""

import json
import sys


ARMING_STATE_ARMED = 2
NAV_STATE_OFFBOARD = 14


def main(argv):
    if len(argv) < 2:
        print("usage: waypoint_check.py <results_json> [<ulg_path>]", file=sys.stderr)
        return 2

    results_path = argv[1]
    ulg_path = argv[2] if len(argv) > 2 else None

    try:
        with open(results_path) as f:
            results = json.load(f)
    except FileNotFoundError:
        print(f"FAIL: results file not produced at {results_path}", file=sys.stderr)
        return 1

    print(f"Waypoints hit: {results['hit_count']}/{results['total']}")
    for i, hit in enumerate(results["waypoints_hit"]):
        marker = "PASS" if hit else "MISS"
        print(f"  Waypoint {i + 1}: {marker}")

    if not results.get("pass"):
        print(f"FAIL: {results.get('reason', 'unknown reason')}", file=sys.stderr)
        return 1

    if ulg_path:
        try:
            from pyulog import ULog

            ulog = ULog(ulg_path)
            status = next(
                (d for d in ulog.data_list if d.name == "vehicle_status"), None
            )
            if status is None:
                print(
                    "FAIL: vehicle_status missing from ULog — did PX4 boot?",
                    file=sys.stderr,
                )
                return 1
            nav = status.data["nav_state"]
            arm = status.data["arming_state"]
            if NAV_STATE_OFFBOARD not in nav:
                print(
                    "FAIL: vehicle never entered OFFBOARD nav_state",
                    file=sys.stderr,
                )
                return 1
            if ARMING_STATE_ARMED not in arm:
                print("FAIL: vehicle never armed", file=sys.stderr)
                return 1
            print("ULog cross-check: vehicle armed and entered OFFBOARD")
        except ImportError:
            print("WARN: pyulog not installed — skipping ULog cross-check")
        except Exception as e:  # noqa: BLE001
            print(f"FAIL: ULog cross-check error: {e}", file=sys.stderr)
            return 1

    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
