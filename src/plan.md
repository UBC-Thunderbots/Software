# Plan: Remove Coroutine Support from Plays (Ticket #2359)

## Context

The play framework currently supports two patterns: **coroutine-based** (legacy, using `boost::coroutines2`) and **FSM-based** (modern, using Boost SML). 12 plays have already been migrated to FSM but still carry dead coroutine stub methods. 11 plays still actively use coroutines. The `Play` base class contains significant coroutine infrastructure marked with `TODO (#2359)` comments. This plan converts all remaining coroutine plays to FSM and removes all coroutine support code from the play framework. It also converts the validation test coroutines.

---

## Phase 1: Convert Simple Single-State Coroutine Plays to FSM

These plays have a single `do { ... yield(); } while(true)` loop — they map to a single FSM state that self-transitions on every update.

### 1a. StopPlay
- **Create** `software/ai/hl/stp/play/stop_play/stop_play_fsm.h` — single `StopState` with `updateStop` action
- **Rewrite** `stop_play.h` → inherit `PlayBase<StopPlayFSM>`, remove `getNextTactics`
- **Rewrite** `stop_play.cpp` → implement `updateTactics` via `fsm.process_event()`
- **Move** files into new `stop_play/` subdirectory (matching existing convention)
- **Update** BUILD files

### 1b. KickoffEnemyPlay
- **Create** `software/ai/hl/stp/play/kickoff_enemy/kickoff_enemy_play_fsm.h` — single `DefendState` with `updateDefense` action
- **Rewrite** `kickoff_enemy_play.h/cpp` → `PlayBase<KickoffEnemyPlayFSM>`
- **Move** into `kickoff_enemy/` subdirectory
- **Update** BUILD files

### 1c. HaltTestPlay
- **Create** `software/ai/hl/stp/play/test_plays/halt_test_play_fsm.h` — single `HaltState`
- **Rewrite** `halt_test_play.h/cpp` → `PlayBase<HaltTestPlayFSM>`
- **Update** BUILD

### 1d. Hardware Challenge Plays (4 plays)
All 4 have identical structure: `do { if(playing) {action} else {setup} yield(); } while(true)`

Each gets a single FSM with one state and a guard for `isPlaying()`:
- `dribbling_parcour_play_fsm.h` — DribblingParcourPlayFSM
- `pass_endurance_play_fsm.h` — PassEndurancePlayFSM
- `scoring_from_contested_possession_play_fsm.h` — ScoringFromContestedPossessionPlayFSM
- `scoring_with_static_defenders_play_fsm.h` — ScoringWithStaticDefendersPlayFSM

Rewrite each play to inherit `PlayBase<*FSM>` and implement `updateTactics`.

---

## Phase 2: Convert Multi-State Coroutine Plays to FSM

### 2a. KickoffFriendlyPlay (2 stages)
- **States:** `SetupState` (while `isSetupState()`), `ChipState` (while `!isPlaying()`)
- **Guards:** `setupDone` (transitions when `!isSetupState()`), `chipDone` (when `isPlaying()`)
- **Create** `kickoff_friendly/kickoff_friendly_play_fsm.h`
- **Move** into `kickoff_friendly/` subdirectory
- **Rewrite** play files

### 2b. ShootOrChipPlay (conditional loop)
- **State:** Single `ShootOrChipState` with `attackerDone` guard for terminal transition
- **Create** `shoot_or_chip/shoot_or_chip_play_fsm.h`
- **Move** into `shoot_or_chip/` subdirectory
- **Rewrite** play files

### 2c. MoveTestPlay (conditional loop)
- **State:** Single `MoveState` with `moveDone` guard checking `move_test_tactic_center_field->done()`
- **Create** `test_plays/move_test_play_fsm.h`
- **Rewrite** play files

---

## Phase 3: Handle AssignedTacticsPlay (Special Case)

`AssignedTacticsPlay` overrides `get()` entirely and already has an empty `updateTactics()`. Its coroutine method is dead code.
- **Remove** `getNextTactics` override from header and source
- No FSM needed — this play has its own unique dispatch pattern via `get()`

---

## Phase 4: Clean Up Play Base Class

Once all plays are FSM-based, remove all coroutine infrastructure from the base class.

### Files: `play.h`, `play.cpp`

**Remove from `play.h`:**
- `#include <boost/coroutine2/all.hpp>`
- `using TacticCoroutine = ...` typedef
- `PriorityTacticVector getTactics(...)` private method
- `void getNextTacticsWrapper(...)` private method
- `virtual void getNextTactics(...)` pure virtual
- `TacticCoroutine::pull_type tactic_sequence` member
- `std::optional<WorldPtr> world_ptr_` member
- `PriorityTacticVector priority_tactics` member
- All `TODO (#2359)` comments

**Make `updateTactics` pure virtual** (currently has default impl that delegates to coroutine)

**Remove from `play.cpp`:**
- `getTactics()` function (lines 30-85)
- `getNextTacticsWrapper()` function (lines 236-251)
- Default `updateTactics()` implementation (lines 253-257)
- Coroutine initialization in constructor (tactic_sequence, world_ptr_ init)

**Update `play.h` constructor** — remove `tactic_sequence` and `world_ptr_` from initializer list

**Remove from BUILD** (`software/ai/hl/stp/play/BUILD`):
- `@boost//:coroutine2` from the `play` target deps

---

## Phase 5: Clean Up FSM Play Stubs

Remove empty `getNextTactics` overrides from all 12 existing FSM plays:

| Play | File |
|------|------|
| HaltPlay | `halt_play/halt_play.h` + `.cpp` |
| ExamplePlay | `example/example_play.h` + `.cpp` |
| OffensePlay | `offense/offense_play.h` + `.cpp` |
| ShootOrPassPlay | `shoot_or_pass/shoot_or_pass_play.h` + `.cpp` |
| PenaltyKickPlay | `penalty_kick/penalty_kick_play.h` + `.cpp` |
| PenaltyKickEnemyPlay | `penalty_kick_enemy/penalty_kick_enemy_play.h` + `.cpp` |
| FreeKickPlay | `free_kick/free_kick_play.h` + `.cpp` |
| BallPlacementPlay | `ball_placement/ball_placement_play.h` + `.cpp` |
| CreaseDefensePlay | `crease_defense/crease_defense_play.h` + `.cpp` |
| DefensePlay | `defense/defense_play.h` + `.cpp` |
| EnemyBallPlacementPlay | `enemy_ball_placement/enemy_ball_placement_play.h` + `.cpp` |
| EnemyFreeKickPlay | `enemy_free_kick/enemy_free_kick_play.h` + `.cpp` |

For each: remove `getNextTactics` declaration from `.h` and empty implementation from `.cpp`.

---

## Phase 6: Convert Validation Coroutines

### 6a. Replace Validation Infrastructure

**Files:**
- `software/simulated_tests/validation/validation_function.h`
- `software/simulated_tests/validation/terminating_function_validator.h` + `.cpp`
- `software/simulated_tests/validation/non_terminating_function_validator.h` + `.cpp`

**Change `ValidationFunction` signature** from:
```cpp
std::function<void(std::shared_ptr<World>, ValidationCoroutine::push_type&)>
```
to a per-tick checker:
```cpp
std::function<std::optional<std::string>(std::shared_ptr<World>)>
```

**Rewrite `TerminatingFunctionValidator`:**
- Remove coroutine state (`pull_type`, wrapper)
- Track completion: call the function each tick. If it returns `nullopt`, mark as complete. If it returns a string, record the error and keep checking.

**Rewrite `NonTerminatingFunctionValidator`:**
- Same simplification: call function each tick, return error or nullopt
- No restart logic needed since there's no coroutine to restart

Remove `boost::coroutine2` includes and typedefs.

### 6b. Rewrite All 17 Validation Functions

**8 Terminating validation functions** (in `simulated_tests/terminating_validation_functions/`):
- `ball_at_point_validation`
- `ball_kicked_validation`
- `friendly_scored_validation`
- `robot_halt_validation`
- `robot_in_center_circle_validation`
- `robot_in_polygon_validation`
- `robot_received_ball_validation`
- `robot_state_validation`

**9 Non-terminating validation functions** (in `simulated_tests/non_terminating_validation_functions/`):
- `ball_in_play_or_scored_validation`
- `ball_never_moves_backward_validation`
- `enemy_never_scores_validation`
- `robot_not_excessively_dribbling_validation`
- `robots_avoid_ball_validation`
- `robots_in_friendly_half_validation`
- `robots_not_in_center_circle_validation`
- `robots_slow_down_validation`
- `robots_violating_motion_constraint`

Each function changes from `void fn(World, yield)` with while-loop-yield to `optional<string> fn(World)` returning error or nullopt.

**For stateful validators** (e.g., `ball_never_moves_backward` which tracks previous ball position): convert to a class or use `std::function` with captured state via a factory function.

### 6c. Update BUILD Files
- Remove `@boost//:coroutine2` from:
  - `software/simulated_tests/validation/BUILD`
  - `software/simulated_tests/terminating_validation_functions/BUILD`
  - `software/simulated_tests/non_terminating_validation_functions/BUILD`

---

## Key Patterns to Reuse

- **FSM template:** `PlayFSM<T>` in `play_fsm.hpp` — all new FSMs inherit from this
- **Play template:** `PlayBase<T>` in `play_base.hpp` — all converted plays inherit from this
- **SML macros:** `DEFINE_SML_STATE`, `DEFINE_SML_EVENT`, `DEFINE_SML_ACTION`, `DEFINE_SML_GUARD` in `software/util/sml_fsm/sml_fsm.h`
- **Reference FSM plays:** `HaltPlayFSM` (simplest), `FreeKickPlayFSM` (multi-state)
- **Factory registration:** `TGenericFactory` pattern stays the same

---

## Verification

1. **Build:** `bazel build //software/ai/hl/stp/play/...` — ensure all play targets compile
2. **Play tests:** Run existing play tests:
   - `bazel test //software/ai/hl/stp/play:kickoff_friendly_play_cpp_test`
   - `bazel test //software/ai/hl/stp/play:kickoff_enemy_play_cpp_test`
   - `bazel test //software/ai/hl/stp/play:stop_play_test`
   - `bazel test //software/ai/hl/stp/play:shoot_or_chip_play_cpp_test`
   - `bazel test //software/ai/hl/stp/play:play_factory_test`
3. **Validation tests:**
   - `bazel test //software/simulated_tests/validation/...`
4. **Full build:** `bazel build //...` — ensure no boost::coroutine2 references remain in play/validation code
5. **Grep check:** Verify no remaining `TacticCoroutine`, `getNextTactics`, or coroutine yield patterns in play files
