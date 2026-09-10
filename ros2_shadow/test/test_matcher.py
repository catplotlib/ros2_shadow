"""The matcher decides which outputs are compared at all.

A fault here does not crash, it silently pairs unrelated results and every
number downstream is wrong but plausible. So the edges get tested explicitly.
"""

from geometry_msgs.msg import PoseStamped, Twist

from ros2_shadow.divergence import Clock
from ros2_shadow.matcher import Matcher, message_stamp

TOL = 0.02


def stamped(seconds: float) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp.sec = int(seconds)
    msg.header.stamp.nanosec = int((seconds % 1) * 1e9)
    return msg


def test_headerless_message_pairs_on_arrival():
    value, clock = message_stamp(Twist(), receive_time=12.5)
    assert value == 12.5
    assert clock == Clock.RECEIVE


def test_header_stamp_is_preferred():
    value, clock = message_stamp(stamped(7.25), receive_time=99.0)
    assert abs(value - 7.25) < 1e-6
    assert clock == Clock.HEADER


def test_zero_stamp_falls_back_to_arrival():
    """An unpopulated header says nothing about ordering, so pairing on it
    would join messages that have nothing to do with each other."""
    value, clock = message_stamp(PoseStamped(), receive_time=4.0)
    assert value == 4.0
    assert clock == Clock.RECEIVE


def test_pairs_within_tolerance():
    m = Matcher(TOL)
    m.push_production(Twist(), 1.000)
    m.push_candidate(Twist(), 1.005)

    pairs = m.drain(now=1.01)
    assert len(pairs) == 1
    assert m.matched == 1
    assert (m.unmatched_production, m.unmatched_candidate) == (0, 0)


def test_pairs_regardless_of_arrival_order():
    m = Matcher(TOL)
    m.push_candidate(Twist(), 1.005)   # candidate first this time
    m.push_production(Twist(), 1.000)
    assert len(m.drain(now=1.01)) == 1


def test_tolerance_boundary_is_inclusive():
    m = Matcher(TOL)
    m.push_production(Twist(), 1.000)
    m.push_candidate(Twist(), 1.000 + TOL)
    assert len(m.drain(now=1.03)) == 1


def test_just_outside_tolerance_does_not_pair():
    m = Matcher(TOL)
    m.push_production(Twist(), 1.000)
    m.push_candidate(Twist(), 1.000 + TOL + 0.001)

    assert m.drain(now=1.03) == []
    assert m.matched == 0


def test_partner_is_waited_for_before_being_called_unmatched():
    """Declaring a miss the instant no partner is present would call every
    slightly late candidate a dropped output."""
    m = Matcher(TOL, max_wait_s=0.2)
    m.push_production(Twist(), 1.000)

    assert m.drain(now=1.05) == []          # still in flight
    assert m.unmatched_production == 0

    m.push_candidate(Twist(), 1.010)
    assert len(m.drain(now=1.06)) == 1


def test_output_with_no_partner_is_eventually_unmatched():
    m = Matcher(TOL, max_wait_s=0.2)
    m.push_production(Twist(), 1.000)

    assert m.drain(now=1.5) == []
    assert m.unmatched_production == 1
    assert m.unmatched_candidate == 0


def test_candidate_without_partner_is_counted_on_its_own_side():
    m = Matcher(TOL, max_wait_s=0.2)
    m.push_candidate(Twist(), 1.000)

    m.drain(now=1.5)
    assert (m.unmatched_production, m.unmatched_candidate) == (0, 1)


def test_streams_stay_aligned_over_many_messages():
    m = Matcher(TOL)
    for i in range(50):
        t = 1.0 + i * 0.05
        m.push_production(Twist(), t)
        m.push_candidate(Twist(), t + 0.004)

    pairs = m.drain(now=10.0)
    assert len(pairs) == 50
    assert m.matched == 50
    assert (m.unmatched_production, m.unmatched_candidate) == (0, 0)
    # each pair must join the same instant, not neighbouring ones
    for production, candidate in pairs:
        assert abs(production.stamp - candidate.stamp) <= TOL


def test_a_dropped_candidate_does_not_shift_every_later_pair():
    """The failure that matters: one missing output causing every subsequent
    production message to pair with the wrong candidate."""
    m = Matcher(TOL, max_wait_s=0.1)
    times = [1.0, 1.05, 1.10, 1.15]
    for t in times:
        m.push_production(Twist(), t)
    for t in times[1:]:                      # the candidate missed the first
        m.push_candidate(Twist(), t + 0.003)

    pairs = m.drain(now=2.0)
    assert m.unmatched_production == 1
    assert len(pairs) == 3
    for production, candidate in pairs:
        assert abs(production.stamp - candidate.stamp) <= TOL


def test_pending_counts_what_is_still_buffered():
    m = Matcher(TOL, max_wait_s=5.0)
    m.push_production(Twist(), 1.0)
    m.push_candidate(Twist(), 9.0)
    m.drain(now=1.1)
    assert m.pending == 2
