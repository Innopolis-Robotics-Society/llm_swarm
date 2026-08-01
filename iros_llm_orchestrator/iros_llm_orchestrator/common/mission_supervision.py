"""Bounded mission supervision loop for channel-3 chat execution.

The supervisor is intentionally pure orchestration: it does not know about ROS,
LLM clients, or tools.  Callers provide async callbacks for execute, verify,
and continuation-plan generation.
"""

from __future__ import annotations

import asyncio
import json
import time
from dataclasses import dataclass, field
from typing import Any, Awaitable, Callable


@dataclass
class MissionConfig:
    enabled: bool = True
    max_duration_sec: float = 180.0
    max_steps: int = 6
    no_progress_limit: int = 2
    min_progress_required: bool = True
    allow_repair: bool = True


@dataclass
class MissionStepContext:
    original_request: str
    step: int
    remaining_time_sec: float
    current_plan: dict
    execution_ok: bool
    failure_info: dict | None
    verification: dict
    previous_verification: dict | None


@dataclass
class MissionContinuation:
    reply: str
    plan: dict
    raw: str = ''


@dataclass
class MissionOutcome:
    ok: bool
    status: str
    reason: str
    final_reply: str
    final_plan: dict
    final_verification: dict | None
    steps_completed: int
    plan_executed: bool
    last_failure: dict | None = None
    history: list[dict] = field(default_factory=list)


ExecutePlan = Callable[[dict, int], Awaitable[tuple[bool, dict | None]]]
VerifyPlan = Callable[
    [dict, bool, dict | None, dict | None, int],
    Awaitable[dict],
]
GenerateContinuation = Callable[
    [MissionStepContext],
    Awaitable[MissionContinuation],
]
LogFn = Callable[[str], None]
NowFn = Callable[[], float]
SleepFn = Callable[[float], Awaitable[None]]


async def supervise_mission(
    *,
    original_request: str,
    initial_plan: dict,
    initial_reply: str,
    config: MissionConfig,
    execute_plan: ExecutePlan,
    verify_plan: VerifyPlan,
    generate_continuation: GenerateContinuation,
    is_help_plan: Callable[[dict], bool] | None = None,
    log_fn: LogFn | None = None,
    now_fn: NowFn | None = None,
    sleep_fn: SleepFn | None = None,
) -> MissionOutcome:
    """Run execute -> verify -> continuation until success or a hard bound."""
    log = log_fn or (lambda _msg: None)
    now = now_fn or time.monotonic
    sleep = sleep_fn or asyncio.sleep
    is_help = is_help_plan or (lambda _plan: False)

    max_steps = max(1, int(config.max_steps))
    max_duration = max(0.1, float(config.max_duration_sec))
    no_progress_limit = max(1, int(config.no_progress_limit))
    started_at = now()
    current_plan = initial_plan
    current_reply = initial_reply
    last_verification: dict | None = None
    last_failure: dict | None = None
    last_execution_ok = False
    failed_plan_fingerprints: set[str] = set()
    last_progress_sig = ''
    repeated_no_progress = 0
    history: list[dict] = []

    log(
        f'LLM mission: start max_duration={max_duration:.1f} '
        f'max_steps={max_steps}'
    )

    for step in range(1, max_steps + 1):
        elapsed = now() - started_at
        if elapsed >= max_duration:
            log(f'LLM mission: timeout duration={elapsed:.1f}')
            return _outcome(
                ok=False,
                status='timeout',
                reason=f'mission supervision timeout after {elapsed:.1f}s',
                reply=current_reply,
                plan=current_plan,
                verification=last_verification,
                steps=step - 1,
                plan_executed=last_execution_ok,
                failure=last_failure,
                history=history,
            )

        log(f'LLM mission: step={step} executing plan_type={current_plan.get("type", "")}')
        execution_ok, failure_info = await execute_plan(current_plan, step)
        last_execution_ok = bool(execution_ok)
        last_failure = failure_info
        log(
            f'LLM mission: step={step} execution_result '
            f'ok={bool(execution_ok)}'
        )

        verification = await verify_plan(
            current_plan,
            bool(execution_ok),
            failure_info,
            last_verification,
            step,
        )
        last_verification = verification
        summary = str((verification or {}).get('summary') or '')[:180]
        verification_ok = bool((verification or {}).get('ok'))
        log(
            f'LLM mission: step={step} verification '
            f'ok={verification_ok} summary={summary}'
        )

        history.append({
            'step': step,
            'execution_ok': bool(execution_ok),
            'verification': verification,
            'plan': current_plan,
            'failure_info': failure_info or {},
        })

        if execution_ok and verification_ok:
            log(f'LLM mission: success step={step} summary={summary}')
            return _outcome(
                ok=True,
                status='success',
                reason='mission verified complete',
                reply=current_reply,
                plan=current_plan,
                verification=verification,
                steps=step,
                plan_executed=True,
                failure=None,
                history=history,
            )

        rec = (verification or {}).get('repair_recommendation') or {}
        repairable = bool(rec.get('repairable')) or (
            not execution_ok and bool(failure_info)
        )
        if not config.allow_repair:
            log(f'LLM mission: abort reason=repair_disabled summary={summary}')
            return _outcome(
                ok=False,
                status='repair_disabled',
                reason='mission incomplete and repair disabled',
                reply=current_reply,
                plan=current_plan,
                verification=verification,
                steps=step,
                plan_executed=bool(execution_ok),
                failure=failure_info,
                history=history,
            )
        if not repairable:
            log(f'LLM mission: abort reason=non_repairable summary={summary}')
            return _outcome(
                ok=False,
                status='non_repairable',
                reason='verification marked mission as non-repairable',
                reply=current_reply,
                plan=current_plan,
                verification=verification,
                steps=step,
                plan_executed=bool(execution_ok),
                failure=failure_info,
                history=history,
            )

        progress_sig = verification_progress_signature(verification)
        if config.min_progress_required:
            if progress_sig and progress_sig == last_progress_sig:
                repeated_no_progress += 1
            else:
                repeated_no_progress = 0
            last_progress_sig = progress_sig
            if repeated_no_progress >= no_progress_limit:
                log(f'LLM mission: no_progress repeated={repeated_no_progress}')
                return _outcome(
                    ok=False,
                    status='no_progress',
                    reason=(
                        'mission supervision stopped: no progress after '
                        'repeated repair attempts'
                    ),
                    reply=current_reply,
                    plan=current_plan,
                    verification=verification,
                    steps=step,
                    plan_executed=bool(execution_ok),
                    failure=failure_info,
                    history=history,
                )

        failed_plan_fingerprints.add(plan_fingerprint(current_plan))
        if step >= max_steps:
            log(f'LLM mission: exhausted steps={max_steps}')
            return _outcome(
                ok=False,
                status='exhausted',
                reason=f'mission supervision exhausted max_steps={max_steps}',
                reply=current_reply,
                plan=current_plan,
                verification=verification,
                steps=step,
                plan_executed=bool(execution_ok),
                failure=failure_info,
                history=history,
            )

        remaining = max(0.0, max_duration - (now() - started_at))
        log(
            f'LLM mission: step={step} repairable=true next=continue'
        )
        context = MissionStepContext(
            original_request=original_request,
            step=step,
            remaining_time_sec=remaining,
            current_plan=current_plan,
            execution_ok=bool(execution_ok),
            failure_info=failure_info,
            verification=verification,
            previous_verification=history[-2]['verification'] if len(history) >= 2 else None,
        )
        try:
            continuation = await generate_continuation(context)
        except Exception as exc:
            log(f'LLM mission: abort reason=invalid_continuation error={exc}')
            return _outcome(
                ok=False,
                status='invalid_continuation',
                reason=f'continuation generation failed: {exc}',
                reply=current_reply,
                plan=current_plan,
                verification=verification,
                steps=step,
                plan_executed=bool(execution_ok),
                failure=failure_info,
                history=history,
            )

        if not isinstance(continuation.plan, dict) or not continuation.plan:
            log('LLM mission: abort reason=invalid_continuation empty_plan')
            return _outcome(
                ok=False,
                status='invalid_continuation',
                reason='continuation plan was empty or invalid',
                reply=current_reply,
                plan=current_plan,
                verification=verification,
                steps=step,
                plan_executed=bool(execution_ok),
                failure=failure_info,
                history=history,
            )
        if is_help(continuation.plan):
            log('LLM mission: abort reason=needs_help')
            return _outcome(
                ok=False,
                status='needs_help',
                reason='LLM requested operator help during mission supervision',
                reply=continuation.reply,
                plan=continuation.plan,
                verification=verification,
                steps=step,
                plan_executed=bool(execution_ok),
                failure=failure_info,
                history=history,
            )

        next_fp = plan_fingerprint(continuation.plan)
        if next_fp in failed_plan_fingerprints:
            log('LLM mission: no_progress repeated_plan=true')
            return _outcome(
                ok=False,
                status='repeated_plan',
                reason='LLM repeated an already failed plan',
                reply=continuation.reply,
                plan=continuation.plan,
                verification=verification,
                steps=step,
                plan_executed=bool(execution_ok),
                failure=failure_info,
                history=history,
            )

        current_plan = continuation.plan
        current_reply = continuation.reply
        log(
            f'LLM mission: step={step + 1} generated continuation '
            f'plan_type={current_plan.get("type", "")}'
        )
        await sleep(0.0)

    log(f'LLM mission: exhausted steps={max_steps}')
    return _outcome(
        ok=False,
        status='exhausted',
        reason=f'mission supervision exhausted max_steps={max_steps}',
        reply=current_reply,
        plan=current_plan,
        verification=last_verification,
        steps=max_steps,
        plan_executed=last_execution_ok,
        failure=last_failure,
        history=history,
    )


def plan_fingerprint(plan: dict) -> str:
    return json.dumps(plan or {}, ensure_ascii=False, sort_keys=True, separators=(',', ':'))


def verification_progress_signature(verification: dict | None) -> str:
    if not isinstance(verification, dict):
        return 'verification-unavailable'
    rec = verification.get('repair_recommendation') or {}
    checks = verification.get('checks') or {}
    failed_bits = _failed_check_signature(checks)
    data = {
        'ok': bool(verification.get('ok')),
        'summary': str(verification.get('summary') or ''),
        'missing_state': list(verification.get('missing_state') or []),
        'repair_type': rec.get('type', ''),
        'repair_reason': rec.get('reason', ''),
        'failed_checks': failed_bits,
    }
    return json.dumps(data, ensure_ascii=False, sort_keys=True, separators=(',', ':'))


def _failed_check_signature(checks: dict) -> list[str]:
    out: list[str] = []
    for key, value in sorted((checks or {}).items()):
        if isinstance(value, dict):
            if value.get('ok') is False:
                out.append(str(key))
            for sub_key in ('failed', 'missing', 'inactive'):
                if value.get(sub_key):
                    out.append(f'{key}.{sub_key}:{_stable(value.get(sub_key))}')
        elif value is False:
            out.append(str(key))
    return out


def _stable(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'))


def _outcome(
    *,
    ok: bool,
    status: str,
    reason: str,
    reply: str,
    plan: dict,
    verification: dict | None,
    steps: int,
    plan_executed: bool,
    failure: dict | None,
    history: list[dict],
) -> MissionOutcome:
    return MissionOutcome(
        ok=ok,
        status=status,
        reason=reason,
        final_reply=reply or '',
        final_plan=plan or {},
        final_verification=verification,
        steps_completed=max(0, int(steps)),
        plan_executed=bool(plan_executed),
        last_failure=failure,
        history=list(history),
    )
