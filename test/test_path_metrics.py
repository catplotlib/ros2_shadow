import math

import pytest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from ros2_shadow.metrics import metrics_for

PATH = metrics_for("nav_msgs/msg/Path")


def path(points):
    msg = Path()
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        msg.poses.append(pose)
    return msg


STRAIGHT = [(i * 1.0, 0.0) for i in range(11)]      # 10 m along x


def test_identical_paths_do_not_diverge():
    a, b = path(STRAIGHT), path(STRAIGHT)
    assert PATH["hausdorff_distance"](a, b) == pytest.approx(0.0)
    assert PATH["endpoint_distance"](a, b) == pytest.approx(0.0)
    assert PATH["length_difference"](a, b) == pytest.approx(0.0)


def test_parallel_offset_path_is_its_offset_away():
    a = path(STRAIGHT)
    b = path([(x, y + 0.5) for x, y in STRAIGHT])
    assert PATH["hausdorff_distance"](a, b) == pytest.approx(0.5)
    assert PATH["mean_deviation"](a, b) == pytest.approx(0.5)
    assert PATH["length_difference"](a, b) == pytest.approx(0.0)


def test_same_destination_by_a_different_route():
    """The case that distance alone cannot judge. Both paths start and end
    together; one bulges out. Large Hausdorff, zero endpoint distance."""
    a = path(STRAIGHT)
    b = path([(x, 3.0 if 3 <= x <= 7 else 0.0) for x, _ in STRAIGHT])

    assert PATH["hausdorff_distance"](a, b) == pytest.approx(3.0)
    assert PATH["endpoint_distance"](a, b) == pytest.approx(0.0)
    assert PATH["start_distance"](a, b) == pytest.approx(0.0)
    assert PATH["length_difference"](a, b) > 1.0


def test_same_route_stopping_somewhere_else():
    """The inverse case: the candidate tracks production closely and then stops
    short. Small deviation, large endpoint distance."""
    a = path(STRAIGHT)
    b = path(STRAIGHT[:6])

    assert PATH["mean_deviation"](a, b) == pytest.approx(0.0)
    assert PATH["endpoint_distance"](a, b) == pytest.approx(5.0)


def test_hausdorff_is_symmetric():
    """A candidate that adds a detour and one that skips a section are
    different failures; a one-way distance misses one of them."""
    a = path(STRAIGHT)
    b = path(STRAIGHT[:6])
    assert PATH["hausdorff_distance"](a, b) == pytest.approx(
        PATH["hausdorff_distance"](b, a)
    )
    assert PATH["hausdorff_distance"](a, b) == pytest.approx(5.0)


def test_mean_deviation_is_less_alarmed_by_a_single_spike():
    a = path(STRAIGHT)
    spiked = [(x, 4.0 if x == 5.0 else 0.0) for x, _ in STRAIGHT]
    b = path(spiked)

    assert PATH["hausdorff_distance"](a, b) == pytest.approx(4.0)
    assert PATH["mean_deviation"](a, b) < 0.5


def test_length_difference_measures_extra_travel():
    a = path([(0, 0), (10, 0)])
    b = path([(0, 0), (0, 10), (10, 10)])
    assert PATH["length_difference"](a, b) == pytest.approx(10.0)


def test_diagonal_length_is_euclidean():
    assert PATH["length_difference"](path([(0, 0), (3, 4)]), path([(0, 0)])) == pytest.approx(5.0)


def test_an_empty_path_is_unmeasurable_not_agreement():
    """Reporting 0.0 when one side has no poses would say the two paths
    coincide, when in fact nothing was compared. The node turns NaN into a
    warning rather than folding it into the statistics."""
    empty = path([])
    geometric = ["endpoint_distance", "start_distance", "hausdorff_distance", "mean_deviation"]

    for name in geometric:
        assert math.isnan(PATH[name](empty, path(STRAIGHT))), name
        assert math.isnan(PATH[name](path(STRAIGHT), empty)), name


def test_length_of_an_empty_path_is_well_defined():
    """Unlike the geometric comparisons, this one has an answer: no poses is
    no distance travelled."""
    assert PATH["length_difference"](path([]), path(STRAIGHT)) == pytest.approx(10.0)


def test_large_paths_stay_fast():
    """Nav2 global paths run to hundreds of points and the planner republishes
    continuously, so an O(n*m) comparison has to be vectorised."""
    import time

    big_a = path([(i * 0.05, math.sin(i * 0.01)) for i in range(1500)])
    big_b = path([(i * 0.05, math.sin(i * 0.01) + 0.1) for i in range(1500)])

    start = time.perf_counter()
    PATH["hausdorff_distance"](big_a, big_b)
    assert time.perf_counter() - start < 0.5
