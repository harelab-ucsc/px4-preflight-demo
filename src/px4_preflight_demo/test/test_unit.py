"""Unit tests for px4_preflight_demo (no PX4 / no DDS bridge required)."""

from px4_preflight_demo import offboard_mission_node as omn


def test_waypoint_list_is_nonempty():
    assert omn.WAYPOINTS, "expected at least one waypoint to fly"


def test_waypoints_are_3d():
    assert all(len(wp) == 3 for wp in omn.WAYPOINTS)


def test_waypoint_radius_is_reasonable():
    assert 0.1 < omn.WAYPOINT_RADIUS < 5.0
