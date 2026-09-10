import math

import pytest
from geometry_msgs.msg import PoseStamped, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ros2_shadow.metrics import metrics_for

TWIST = metrics_for("geometry_msgs/msg/Twist")
POSE = metrics_for("geometry_msgs/msg/PoseStamped")
TRAJ = metrics_for("trajectory_msgs/msg/JointTrajectory")


def twist(x=0.0, z=0.0):
    msg = Twist()
    msg.linear.x = x
    msg.angular.z = z
    return msg


def pose(x=0.0, y=0.0, z=0.0, quat=(0.0, 0.0, 0.0, 1.0)):
    msg = PoseStamped()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    (msg.pose.orientation.x, msg.pose.orientation.y,
     msg.pose.orientation.z, msg.pose.orientation.w) = quat
    return msg


def trajectory(names, points, duration=1.0):
    msg = JointTrajectory()
    msg.joint_names = list(names)
    for positions in points:
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        msg.points.append(point)
    return msg


# -- twist ---------------------------------------------------------------

def test_identical_twists_do_not_diverge():
    assert TWIST["linear_error"](twist(0.4), twist(0.4)) == 0.0


def test_linear_error_is_the_magnitude_of_the_difference():
    assert TWIST["linear_error"](twist(0.4), twist(0.1)) == pytest.approx(0.3)


def test_direction_reversal_fires_on_a_sign_flip():
    """The case an error norm understates: production drives forward, the
    candidate drives back. As a magnitude that is 0.7, indistinguishable from
    ordinary drift; as a behaviour it is the robot going the wrong way."""
    assert TWIST["direction_reversal"](twist(0.4), twist(-0.3)) == 1.0


def test_direction_reversal_ignores_agreement():
    assert TWIST["direction_reversal"](twist(0.4), twist(0.1)) == 0.0
    assert TWIST["direction_reversal"](twist(-0.4), twist(-0.1)) == 0.0


def test_direction_reversal_ignores_a_standstill():
    """Sign is meaningless at rest, so noise either side of zero must not be
    reported as a reversal."""
    assert TWIST["direction_reversal"](twist(0.0001), twist(-0.0002)) == 0.0


# -- pose ----------------------------------------------------------------

def test_translation_error_is_euclidean():
    assert POSE["translation_error"](pose(0, 0, 0), pose(3, 4, 0)) == pytest.approx(5.0)


def test_rotation_error_is_zero_for_the_same_orientation():
    assert POSE["rotation_error"](pose(), pose()) == pytest.approx(0.0, abs=1e-9)


def test_negated_quaternion_is_the_same_rotation():
    """q and -q describe one orientation. Without taking the dot product
    absolute, identical poses report a full pi of error."""
    error = POSE["rotation_error"](pose(quat=(0, 0, 0, 1)), pose(quat=(0, 0, 0, -1)))
    assert error == pytest.approx(0.0, abs=1e-9)


def test_rotation_error_measures_a_real_turn():
    half = math.sin(math.pi / 4), math.cos(math.pi / 4)   # 90 degrees about z
    error = POSE["rotation_error"](pose(), pose(quat=(0, 0, half[0], half[1])))
    assert error == pytest.approx(math.pi / 2, abs=1e-6)


# -- trajectory ----------------------------------------------------------

def test_identical_trajectories_do_not_diverge():
    a = trajectory(["shoulder", "elbow"], [[0.0, 0.0], [0.5, 0.5]])
    b = trajectory(["shoulder", "elbow"], [[0.0, 0.0], [0.5, 0.5]])
    assert TRAJ["joint_position_rmse"](a, b) == 0.0


def test_joints_are_matched_by_name_not_by_index():
    """Two planners need not agree on joint ordering. Comparing index by index
    would measure a shoulder against an elbow and report confident nonsense."""
    a = trajectory(["shoulder", "elbow", "wrist"], [[0.1, 0.2, 0.3]])
    b = trajectory(["wrist", "elbow", "shoulder"], [[0.3, 0.2, 0.1]])
    assert TRAJ["joint_position_rmse"](a, b) == pytest.approx(0.0)
    assert TRAJ["max_joint_delta"](a, b) == pytest.approx(0.0)


def test_max_joint_delta_finds_the_worst_joint():
    a = trajectory(["shoulder", "elbow"], [[0.0, 0.0]])
    b = trajectory(["shoulder", "elbow"], [[0.01, 0.62]])
    assert TRAJ["max_joint_delta"](a, b) == pytest.approx(0.62)


def test_endpoint_delta_uses_the_final_waypoint():
    """Two trajectories can wander apart and still finish together, or track
    closely and end somewhere else. Where the arm stops is its own question."""
    a = trajectory(["j"], [[0.0], [5.0], [1.0]])
    b = trajectory(["j"], [[0.0], [-5.0], [1.0]])
    assert TRAJ["endpoint_delta"](a, b) == pytest.approx(0.0)
    assert TRAJ["max_joint_delta"](a, b) == pytest.approx(10.0)


def test_duration_delta_compares_planned_completion():
    a = trajectory(["j"], [[0.0]], duration=2.0)
    b = trajectory(["j"], [[0.0]], duration=3.5)
    assert TRAJ["duration_delta"](a, b) == pytest.approx(1.5)


def test_disjoint_joint_names_do_not_crash():
    a = trajectory(["left"], [[0.0]])
    b = trajectory(["right"], [[1.0]])
    assert TRAJ["joint_position_rmse"](a, b) == 0.0
