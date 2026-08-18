"""Metrics 11-14 have to be right before M4 is recorded, not after.

No M4 run exists yet, so the scorer is exercised against a stub bag. Run it
with: python3 -m pytest paper/results/test_formation_metrics.py
"""

import sys, os
sys.path.insert(0, os.path.dirname(__file__))

import analyze_sessions as A
import bagreader as B


class _StubBag:
    """Enough of bagreader.Bag for _score_formation."""

    def __init__(self, status, odom=None):
        self._status = status
        self._odom = odom or []
        self.topics = {'/formations/status': ('x', 0)}
        if odom:
            self.topics['/robot_0/odom'] = ('x', 0)

    def read(self, topic):
        rows = self._status if topic == '/formations/status' else self._odom
        for t, msg in rows:
            yield B.BagMessage(topic, t, msg)


def _status(t, state, errors, leader='robot_0'):
    return (t, {'header': {'stamp': t, 'frame_id': ''},
                'formations': [{
                    'header': {'stamp': t, 'frame_id': ''},
                    'formation_id': 'col1', 'leader_ns': leader,
                    'follower_ns': ['robot_1', 'robot_2'],
                    'state': state, 'failure_code': 0, 'failure_reason': '',
                    'errors_m': errors,
                    'max_error_m': max(errors) if errors else -1.0,
                    'mean_error_m': -1.0}]})


def _odom(t, y):
    return (t, {'header': {'stamp': t, 'frame_id': 'map'},
                'child_frame_id': '', 'position': (2.7, y, 0.0)})


def test_no_data_is_reported_as_absent_not_as_zero():
    out = A._score_formation(_StubBag([]))
    assert out == {'present': False}


def test_minus_one_is_dropped_rather_than_averaged_as_perfect_tracking():
    """The whole point of the -1.0 rule: a lost follower must not read as 0."""
    bag = _StubBag([
        _status(0.0, 1, [0.4, -1.0]),
        _status(0.1, 2, [0.6, -1.0]),
    ])
    out = A._score_formation(bag)
    assert out['eps_peak_m'] == 0.6                 # not -1.0, not 0
    assert out['eps_all_median_m'] == 0.5
    assert out['dropped_frac'] == 0.5               # half the samples were -1


def test_t_stable_runs_from_activation_not_from_the_first_sample():
    bag = _StubBag([
        _status(10.0, 0, []),        # INACTIVE, before activation
        _status(11.0, 0, []),
        _status(12.0, 1, [0.9]),     # FORMING -- t0
        _status(13.0, 1, [0.5]),
        _status(15.5, 2, [0.1]),     # STABLE
    ])
    out = A._score_formation(bag)
    assert out['t_stable_s'] == 3.5
    assert out['n_samples'] == 5 and out['n_active'] == 3


def test_f_degraded_counts_only_active_samples():
    bag = _StubBag([
        _status(0.0, 0, []),         # INACTIVE, must not dilute the fraction
        _status(1.0, 2, [0.1]),
        _status(2.0, 3, [0.4]),      # DEGRADED
        _status(3.0, 2, [0.1]),
        _status(4.0, 4, [0.9]),      # BROKEN
    ])
    out = A._score_formation(bag)
    assert out['f_degraded'] == 0.25                # 1 of 4 active
    assert out['f_broken'] == 0.25


def test_eps_ss_uses_the_wide_stretches_only():
    """y = 1.0 is inside the measured pinch, y = 5.0 is not."""
    bag = _StubBag(
        [_status(1.0, 2, [0.10]),    # leader at y=5.0  -> wide
         _status(2.0, 2, [0.12]),    # y=5.0            -> wide
         _status(3.0, 2, [0.50]),    # y=1.0            -> narrow
         _status(4.0, 2, [0.60])],   # y=1.0            -> narrow
        odom=[_odom(0.5, 5.0), _odom(2.5, 1.0)])
    out = A._score_formation(bag)
    assert out['eps_ss_wide_m'] == 0.11             # median of the wide pair
    assert out['eps_narrow_m'] == 0.55
    assert out['n_wide'] == 2 and out['n_narrow'] == 2
    # eps_peak is over the whole traverse, pinch included
    assert out['eps_peak_m'] == 0.6


def test_narrow_bands_match_the_measured_corridor():
    assert A._in_narrow(1.0) and A._in_narrow(-3.7)
    assert not A._in_narrow(5.0) and not A._in_narrow(-8.0)
    assert not A._in_narrow(1.61) and not A._in_narrow(-0.26)


def test_missing_leader_odom_does_not_raise():
    bag = _StubBag([_status(1.0, 2, [0.2])])        # no odom topic
    out = A._score_formation(bag)
    assert out['eps_ss_wide_m'] is None
    assert out['eps_peak_m'] == 0.2                 # the rest still scores
