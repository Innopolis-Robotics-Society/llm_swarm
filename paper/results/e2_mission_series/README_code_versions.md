# Which run was recorded on which code

The campaign outlived several defects, and some runs were recorded before the
fixes that affect their result. This file records which ones, so it does not
have to be reconstructed from commit hashes later.

## Single-carrier guard (`57e8fdc`)

The model regularly adds a spare robot to a plan, sending it to the pickup zone
of a carry that another robot already services under the same plan. The
consequences and the mechanism are in the header of `common/carry_guard.py`.

Checked by replaying **every** recorded plan through the new code:

| cell | run | guard would have fired | what was recorded |
|---|---|---|---|
| model-9b | `20260815_145533` | no | 5/5 |
| model-9b | `20260815_150251` | **yes**, drop robot_4 | 5/5 |
| model-9b | `20260815_150815` | **yes**, drop robot_4 | 5/5 |
| model-9b | `20260815_151322` | **yes**, drop robot_4 | 5/5 |
| model-9b | `20260815_151832` | no | 5/5 |
| B | `20260818_084101` | no | 5/5 |
| B | `20260818_084602` | no | 5/5 |
| B | `20260818_085054` | no | 5/5 |
| B | `20260818_093216` | **yes**, dropped robot_6 | 5/5 |
| B | `20260818_093704` | **yes**, dropped robot_5 | 5/5 |

The three B-M3 runs (`20260818_111548`, `113030`, `114445`) are not in the
table because they were recorded *after* the guard existed: the guard was live
and fired zero times in all three, so their plans contain no duplicate carrier
and there is nothing to replay.

Runs `20260818_085836` and `20260818_090145` (both 4/5, the mission stalled)
were removed from the set and replaced by `093216` and `093704`. Those two are
where the defect was found; their plans are analysed in the header of
`common/carry_guard.py`.

The `no-LLM` runs and the two pre-guard runs (`20260815_153035`,
`20260815_154541`) are absent from the table: the first five have no chat
journal by construction of the cell, and the last two have an empty one (see
`cell_b_m3/README.md`).

**What follows from this.**

Five of the ten plans that can be checked contain a duplicate carrier — 50%.
Counting the three B-M3 plans as well, which are checkable and contain none,
the figure over all thirteen language runs is 5 of 13. Either way it is a
stable planning pattern, not a one-off.

The outcome, however, is decided by geometry rather than by the plan: in
`model-9b` the spare robot got in nobody's way three times and the mission
still finished 5/5; in cell B it twice stood in a chokepoint and killed the
delivery outright. **The same planning error yields 5/5 or 4/5 depending on who
arrived where first.**

**Cell B was not re-recorded in full.** Two of the five runs were re-recorded;
the three successful ones were left as they are. That is correct precisely
because on their plans the guard is an identity operation, which the table
above establishes. In the two new runs the model produced the same defective
plan, the guard fired, and both gave 5/5 — so the fix is verified on the live
stack, not only in tests. The `model-9b` cell is not re-recorded at all and
stays recorded without the guard; for the task-completion fraction this changes
nothing (all five are 5/5 anyway), but its `guard_single_carrier_fired` counter
is zero because the guard did not exist yet, not because the defect was absent.

## Honest MAPF reporting of partial delivery

The M4 run `20260818_140918` exposed a chain of four substrate defects, each
hiding the next:

1. `mapf_lns2_node` considered the mission done based on `plan_required`, from
   which `unplanable` had already been subtracted, and returned `success=true`
   with the message `All planable agents arrived (N unplanable skipped)`. A
   robot named in the goal never moved, and the leg reported success.
2. Substituting a blocked goal with the nearest free cell was not bounded by
   distance, and arrival is measured against the substituted point. In this run
   the goal moved 1.97 m and the robot "arrived".
3. `compute_formation_staging` builds slots from the leader's live pose. The
   leader had been dropped from the plan (item 1), stayed at spawn, and the
   substitution sent three followers back 25 m from the cafeteria.
4. `SetFormation`/`DisableFormation` did not reset `@active_action` on failure,
   and `leaf_sender` reads `@action_status` only after it returns to `none`.
   The manager refused instantly; the executor found out 300 s later — a fifth
   of the mission budget.

**Impact on the already recorded runs is zero.** All four events
(`Marking unplanable`, `Skipped robots`, `using nearest free cell`,
`left agents with empty paths`) occur in exactly one run out of twenty — in
`20260818_140918` itself. In the other nineteen they are zero, checked against
`launch.log`. No recorded run would have received a different score under the
new code, so the B, B-M3, model-9b and no-LLM cells do not need re-recording.

What changed in behaviour:

- new error code `SetGoals.Result.PARTIAL=205`; `MapfPlan` turns it into a hard
  leg failure instead of `WARN`+`SUCCESS`;
- parameter `max_goal_rehome_m` (0.5 m) bounds the goal substitution; beyond it
  the robot is marked `unplanable`;
- parameter `formation_max_staging_travel_m` (8 m) — the substitution refuses
  with `FormationStagingRefused`, naming the leader and its actual position;
- channel-3 `timeout_sec` raised 120 → 180 (supervision continuation on M4 did
  not fit).

### Correction: the real root is `store_active_plan`

The first version of the fix treated the symptom one level above the defect. In
`20260818_140918` the server did not return "N unplanable skipped" but
literally **`All agents arrived`, `planned=4`** — while robot_0 had not moved.

`store_active_plan` (`mapf_lns2_node.cpp`) did
`active_robot_ids_ = plan_ids_ext`, i.e. it overwrote the set of requested
robots with the set of planned ones. One vector carried two different roles:
"whom the caller asked for" and "who is in the current plan". They diverge
exactly when a robot is benched during a replan: robot_0 was benched, the
replan planned three, the set shrank to three — and the robot never came back,
because the replan takes `snap_ids = active_robot_ids_`, where it is no longer
present. Five seconds later it left the bench, but by then the mission had
forgotten it was asked to carry it, and honestly closed as 3 of 3.

Fixed with two changes:

- `requested_robot_ids_` is fixed when the goal is accepted and lives for the
  whole mission; `PARTIAL` is computed from it rather than from the latest
  plan, so a robot filtered out at validation also appears in the report;
- `store_active_plan` only adds and updates goals, and never shrinks
  `active_robot_ids_`, so a benched robot is skipped in the current replan and
  returns in the next one — as the lives mechanism intended.

The blast radius is the same: the benching and skipping events
(`retrying benched robots`, `crossed empty-path threshold`, `out of lives`,
`Skipped robots`, `Marking unplanable`) appear in none of the nineteen runs,
and all four appear in `20260818_140918`.
