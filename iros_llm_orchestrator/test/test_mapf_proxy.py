"""Unit tests for the MAPF LLM proxy feedback classifier."""

from iros_llm_orchestrator.mapf_proxy import (
    _decision_event_from_feedback,
    _feedback_summary,
)
from iros_llm_swarm_interfaces.action import SetGoals


def _feedback(**kwargs):
    fb = SetGoals.Feedback()
    fb.status = kwargs.get('status', 'executing')
    fb.elapsed_ms = kwargs.get('elapsed_ms', 123)
    fb.robots_arrived = kwargs.get('robots_arrived', 1)
    fb.robots_active = kwargs.get('robots_active', 2)
    fb.robot_stall = kwargs.get('robot_stall', 0)
    fb.replans_done = kwargs.get('replans_done', 0)
    fb.info = kwargs.get('info', '')
    fb.warning = kwargs.get('warning', '')
    return fb


def test_warning_feedback_becomes_warn_event():
    event = _decision_event_from_feedback(
        _feedback(warning='robot_0 blocked goal'))

    assert event.level == 'WARN'
    assert event.event == 'robot_0 blocked goal'


def test_failed_status_becomes_error_event():
    event = _decision_event_from_feedback(
        _feedback(status='failed', info='planner failed'))

    assert event.level == 'ERROR'
    assert event.event == 'planner failed'


def test_robot_stall_becomes_warn_event():
    event = _decision_event_from_feedback(
        _feedback(robot_stall=2, replans_done=1, info='stalled robots'))

    assert event.level == 'WARN'
    assert 'robot_stall=2' in event.event
    assert 'replans=1' in event.event


def test_healthy_feedback_does_not_trigger_decision():
    assert _decision_event_from_feedback(_feedback(info='progress normal')) is None


def test_feedback_summary_includes_key_fields():
    summary = _feedback_summary(
        _feedback(status='replanning', robot_stall=1, replans_done=3,
                  info='retry', warning='blocked'))

    assert 'status=replanning' in summary
    assert 'stall=1' in summary
    assert 'replans=3' in summary
    assert 'INFO: retry' in summary
    assert 'WARN: blocked' in summary
