"""Unit tests for the semantic read-only /llm/chat tool layer."""

import asyncio
import math
from types import SimpleNamespace

from iros_llm_orchestrator.common.user_prompt import load_map_config
from iros_llm_orchestrator.context import ChatContextConfig
from iros_llm_orchestrator.context.geometry_utils import (
    centroid,
    euclidean_distance,
    rms_spread,
)
from iros_llm_orchestrator.context.semantic_tools import (
    CombinedReadOnlyToolBroker,
    DEFAULT_SEMANTIC_READ_TOOLS,
    SemanticToolProvider,
)


class _FakePoseCache:
    def __init__(self, poses=None):
        self._poses = poses or {}

    def snapshot(self, stale_threshold_ms: int = 2000):
        return {
            rid: {
                'x': float(pose['x']),
                'y': float(pose['y']),
                'yaw': float(pose.get('yaw', 0.0)),
                'stale_ms': int(pose.get('stale_ms', 0)),
                'stale': int(pose.get('stale_ms', 0)) > stale_threshold_ms,
            }
            for rid, pose in self._poses.items()
        }


def _run(coro):
    return asyncio.run(coro)


def _provider(map_name='amongus', *, pose_cache=None, runtime_context=None):
    return _provider_from_config(
        load_map_config(map_name),
        map_name=map_name,
        pose_cache=pose_cache,
        runtime_context=runtime_context,
    )


def _provider_from_config(
    cfg,
    *,
    map_name='test_map',
    pose_cache=None,
    runtime_context=None,
):
    config = ChatContextConfig(
        provider='none',
        map_name=map_name,
        map_config=cfg,
    )
    return SemanticToolProvider(
        SimpleNamespace(),
        config,
        pose_cache=pose_cache,
        get_cached_context=lambda: runtime_context or {},
        get_obstacle_context=lambda: '',
    )


def _mock_map():
    return {
        'name': 'mock',
        'bounds': {'x_min': -20.0, 'x_max': 20.0, 'y_min': -20.0, 'y_max': 20.0},
        'named_locations': {
            'base': [0.0, 0.0],
            'zone': [10.0, 0.0],
            'tiny_zone': [0.0, 10.0],
        },
        'location_aliases': {'home': 'base', 'big zone': 'zone'},
        'robot_groups': {
            'cyan': {
                'ids': [0, 1, 2, 3],
                'home': [0.0, 0.0],
                'aliases': ['cyan'],
            },
        },
        'formation_zones': [
            {'name': 'zone', 'coords': [10.0, 0.0], 'radius': 10.0},
            {'name': 'tiny_zone', 'coords': [0.0, 10.0], 'radius': 0.5},
        ],
        'heuristics': '- Bottleneck near the main corridor.\n',
    }


def test_geometry_helpers_compute_distance_and_spread():
    assert euclidean_distance((0.0, 0.0), (3.0, 4.0)) == 5.0

    points = [(0.0, 0.0), (2.0, 0.0), (0.0, 2.0), (2.0, 2.0)]
    center = centroid(points)

    assert center == (1.0, 1.0)
    assert math.isclose(rms_spread(points, center), math.sqrt(2.0))


def test_semantic_resolve_location_resolves_aliases_from_map_config():
    provider = _provider('amongus')

    result = _run(provider.execute_tool(
        'semantic_resolve_location',
        {'query': 'caf'},
    ))

    assert result['resolved'] is True
    assert result['canonical'] == 'cafeteria'
    assert result['coords'] == [2.7, 10.1]
    assert result['match_type'] == 'alias'
    assert result['is_formation_zone'] is True
    assert result['formation_radius'] == 5.0


def test_semantic_resolve_location_unknown_returns_known_locations():
    provider = _provider('amongus')

    result = _run(provider.execute_tool(
        'semantic_resolve_location',
        {'query': 'moon base'},
    ))

    assert result['resolved'] is False
    assert 'cafeteria' in result['known_locations']
    assert isinstance(result['suggestions'], list)


def test_semantic_get_group_state_computes_centroid_spread_and_nearest():
    provider = _provider_from_config(
        _mock_map(),
        pose_cache=_FakePoseCache({
            0: {'x': 0.0, 'y': 0.0},
            1: {'x': 2.0, 'y': 0.0},
            2: {'x': 0.0, 'y': 2.0},
            3: {'x': 2.0, 'y': 2.0},
        }),
        runtime_context={
            'bt_state': {
                'mode': 'idle',
                'action_status': 'OK',
                'active_action': 'none',
                'robot_ids': [],
            },
        },
    )

    result = _run(provider.execute_tool(
        'semantic_get_group_state',
        {'group': 'cyan'},
    ))

    assert result['resolved'] is True
    assert result['robot_ids'] == [0, 1, 2, 3]
    assert result['known_pose_count'] == 4
    assert result['missing_pose_count'] == 0
    assert result['group_center'] == [1.0, 1.0]
    assert math.isclose(result['spread']['rms_m'], math.sqrt(2.0), abs_tol=0.001)
    assert math.isclose(
        result['spread']['max_radius_m'],
        math.sqrt(2.0),
        abs_tol=0.001,
    )
    assert result['nearest_locations']['0'] == 'base'
    assert result['robot_locations']['0']['nearest'] == 'base'
    assert result['state_guess'] == 'idle'


def test_semantic_get_group_state_keeps_legacy_fields_for_partial_poses():
    provider = _provider(
        'amongus',
        pose_cache=_FakePoseCache({
            0: {'x': -21.8, 'y': 9.8},
            1: {'x': -22.0, 'y': 10.0},
        }),
        runtime_context={
            'bt_state': {
                'mode': 'idle',
                'action_status': 'OK',
                'active_action': 'none',
                'robot_ids': [],
            },
        },
    )

    result = _run(provider.execute_tool(
        'semantic_get_group_state',
        {'group': 'cyan'},
    ))

    assert result['home'] == [-21.9, 9.9]
    assert result['nearest_locations']['0'] == 'upper_engine'
    assert result['state_guess'] == 'partially_unknown'


def test_semantic_get_route_context_returns_straight_line_distance():
    provider = _provider(
        'amongus',
        pose_cache=_FakePoseCache({
            0: {'x': -21.9, 'y': 9.9},
            1: {'x': -21.9, 'y': 9.9},
            2: {'x': -21.9, 'y': 9.9},
            3: {'x': -21.9, 'y': 9.9},
        }),
    )

    result = _run(provider.execute_tool(
        'semantic_get_route_context',
        {'group': 'cyan', 'target': 'cafeteria'},
    ))

    assert result['target_resolved'] is True
    assert result['start_area_guess'] == 'upper_engine'
    assert result['route']['source'] == 'straight_line_fallback'
    assert result['route']['confidence'] == 'low'
    assert math.isclose(
        result['straight_line_distance_m'],
        euclidean_distance((-21.9, 9.9), (2.7, 10.1)),
        abs_tol=0.001,
    )


def test_semantic_find_free_group_goals_in_room_avoids_existing_robots():
    cfg = _mock_map()
    cfg['named_locations']['room'] = [0.0, 0.0]
    cfg['geometry'] = {
        'room': {
            'center': [0.0, 0.0],
            'width_m': 6.0,
            'height_m': 6.0,
        },
    }
    provider = _provider_from_config(
        cfg,
        pose_cache=_FakePoseCache({
            16: {'x': 0.0, 'y': 0.0},
            17: {'x': 0.8, 'y': 0.0},
        }),
    )

    result = _run(provider.execute_tool(
        'semantic_find_free_group_goals_in_room',
        {
            'room': 'room',
            'robot_ids': [12, 13],
            'avoid_robot_ids': [16, 17],
            'prefer_near_group': [16, 17],
            'placement_mode': 'around_group',
        },
    ))

    assert 'semantic_find_free_group_goals_in_room' in DEFAULT_SEMANTIC_READ_TOOLS
    assert result['ok'] is True
    assert result['robot_ids'] == [12, 13]
    assert len(result['goals']) == 2
    assert result['checks']['avoids_existing_robots'] is True
    assert result['checks']['avoids_avoid_robot_ids'] is True
    assert result['mapf_leaf']['type'] == 'mapf'
    assert 'spread' not in result['mapf_leaf']


def test_semantic_check_goal_feasibility_scores_risk():
    provider = _provider_from_config(
        _mock_map(),
        pose_cache=_FakePoseCache({
            0: {'x': 0.0, 'y': 0.0},
            1: {'x': 0.0, 'y': 0.0},
            2: {'x': 0.0, 'y': 0.0},
            3: {'x': 0.0, 'y': 0.0},
        }),
    )

    result = _run(provider.execute_tool(
        'semantic_check_goal_feasibility',
        {'group': 'cyan', 'target': 'zone'},
    ))

    assert result['target_resolved'] is True
    assert result['target_inside_bounds'] is True
    assert result['checks']['bounds'] == 'pass'
    assert result['checks']['occupancy'] == 'not_checked'
    assert result['risk_score'] == 2
    assert result['risk'] == 'medium'
    assert result['recommendation'] == 'choose_staging'


def test_semantic_check_goal_feasibility_unknown_target():
    provider = _provider_from_config(_mock_map())

    result = _run(provider.execute_tool(
        'semantic_check_goal_feasibility',
        {'group': 'cyan', 'target': 'missing'},
    ))

    assert result['target_resolved'] is False
    assert result['risk'] == 'unknown'
    assert result['recommendation'] == 'ask_clarification'


def test_formation_footprint_generation_for_line_wedge_circle():
    provider = _provider_from_config(_mock_map())

    line = _run(provider.execute_tool(
        'semantic_check_formation_feasibility',
        {'group': 'cyan', 'formation': 'line', 'location': 'zone'},
    ))
    wedge = _run(provider.execute_tool(
        'semantic_check_formation_feasibility',
        {'group': 'cyan', 'formation': 'wedge', 'location': 'zone'},
    ))
    circle = _run(provider.execute_tool(
        'semantic_check_formation_feasibility',
        {'group': 'cyan', 'formation': 'circle', 'location': 'zone'},
    ))

    assert len(line['footprint']['points']) == 4
    assert line['footprint']['points'][0] == [8.5, 0.0]
    assert len(wedge['footprint']['points']) == 4
    assert wedge['footprint']['points'][0] == [10.0, 0.0]
    assert len(circle['footprint']['points']) == 4
    assert math.isclose(
        circle['footprint']['radius_m'],
        math.sqrt(0.5),
        abs_tol=0.001,
    )


def test_semantic_check_formation_feasibility_respects_formation_zones():
    provider = _provider('amongus')

    cafeteria = _run(provider.execute_tool(
        'semantic_check_formation_feasibility',
        {'group': 'cyan', 'formation': 'wedge', 'location': 'cafeteria'},
    ))
    admin = _run(provider.execute_tool(
        'semantic_check_formation_feasibility',
        {'group': 'cyan', 'formation': 'wedge', 'location': 'admin'},
    ))

    assert cafeteria['can_form_here'] is True
    assert cafeteria['recommended_location'] == 'cafeteria'
    assert cafeteria['checks']['zone_fit'] == 'pass'
    assert admin['can_form_here'] is False
    assert 'formation zone' in ' '.join(admin['warnings'])


def test_semantic_check_formation_feasibility_zone_fit_fails_when_too_small():
    provider = _provider_from_config(_mock_map())

    result = _run(provider.execute_tool(
        'semantic_check_formation_feasibility',
        {'group': 'cyan', 'formation': 'line', 'location': 'tiny_zone'},
    ))

    assert result['can_form_here'] is False
    assert result['checks']['zone_fit'] == 'fail'


def test_semantic_get_allowed_action_schema_matches_plan_executor_types():
    provider = _provider('amongus')

    result = _run(provider.execute_tool(
        'semantic_get_allowed_action_schema',
        {},
    ))

    assert result['leaf_types'] == ['mapf', 'formation', 'idle']
    assert result['container_types'] == ['sequence', 'parallel']
    assert 'obstacles' not in result['leaf_types']


def test_semantic_tools_rejected_if_not_in_allowlist():
    provider = _provider('amongus')
    broker = CombinedReadOnlyToolBroker(
        semantic_provider=provider,
        semantic_allowlist=['semantic_resolve_location'],
        mcp_broker=None,
        max_tools_per_round=3,
        tool_timeout_sec=0.1,
        max_result_chars=6000,
    )

    result = _run(broker.execute_tool_request({
        'tools': [{'name': 'semantic_get_group_state', 'args': {'group': 'cyan'}}],
    }))

    assert result['results'][0]['status'] == 'rejected'
    assert 'semantic_tool_allowlist' in result['results'][0]['error']


def test_write_control_tools_still_rejected():
    provider = _provider('amongus')
    broker = CombinedReadOnlyToolBroker(
        semantic_provider=provider,
        semantic_allowlist=list(DEFAULT_SEMANTIC_READ_TOOLS),
        mcp_broker=None,
        max_tools_per_round=3,
        tool_timeout_sec=0.1,
        max_result_chars=6000,
    )

    result = _run(broker.execute_tool_request({
        'tools': [{'name': 'call_service', 'args': {'service': '/reset'}}],
    }))

    assert result['results'][0]['status'] == 'rejected'
    assert 'blocked' in result['results'][0]['error']


def test_semantic_tools_work_without_mcp():
    provider = _provider('amongus')
    broker = CombinedReadOnlyToolBroker(
        semantic_provider=provider,
        semantic_allowlist=['semantic_resolve_location'],
        mcp_broker=None,
        max_tools_per_round=3,
        tool_timeout_sec=0.1,
        max_result_chars=6000,
    )

    result = _run(broker.execute_tool_request({
        'tools': [{'name': 'semantic_resolve_location',
                   'args': {'query': 'кафе'}}],
    }))

    assert result['results'][0]['status'] == 'ok'
    assert result['results'][0]['result']['canonical'] == 'cafeteria'
