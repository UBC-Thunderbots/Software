# Play and Tactic FSM Diagrams

## [PlaySelectionFSM](/src/software/ai/play_selection_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> Halt
Halt --> Stop : [gameStateStopped]\n<i>setupStopPlay</i>
Halt --> Playing : [gameStatePlaying]\n<i>setupOffensePlay</i>
Halt --> SetPlay : [gameStateSetupRestart]\n<i>setupSetPlay</i>
Stop --> Halt : [gameStateHalted]\n<i>setupHaltPlay</i>
Stop --> Playing : [gameStatePlaying]\n<i>setupOffensePlay</i>
Stop --> SetPlay : [gameStateSetupRestart]\n<i>setupSetPlay</i>
Playing --> Halt : [gameStateHalted]\n<i>setupHaltPlay</i>
Playing --> Stop : [gameStateStopped]\n<i>setupStopPlay</i>
Playing --> SetPlay : [gameStateSetupRestart]\n<i>setupSetPlay</i>
SetPlay --> Halt : [gameStateHalted]\n<i>resetSetPlay, setupHaltPlay</i>
SetPlay --> Stop : [gameStateStopped]\n<i>resetSetPlay, setupStopPlay</i>
SetPlay --> Playing : [gameStatePlaying]\n<i>resetSetPlay, setupOffensePlay</i>
SetPlay --> SetPlay : [gameStateSetupRestart]\n<i>setupSetPlay</i>
Terminate:::terminate --> Terminate:::terminate

```

## [BallPlacementPlayFSM](/src/software/ai/hl/stp/play/ball_placement/ball_placement_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> StartState
StartState --> AlignPlacementState : [!shouldPickOffWall]\n<i>alignPlacement</i>
StartState --> AlignWallState : [shouldPickOffWall]
AlignWallState --> AlignWallState : [!wallAlignDone && shouldPickOffWall]\n<i>alignWall</i>
AlignWallState --> PickOffWallState : [wallAlignDone]\n<i>setPickOffDest</i>
AlignWallState --> AlignPlacementState : [!shouldPickOffWall]
PickOffWallState --> PickOffWallState : [!wallPickOffDone]\n<i>pickOffWall</i>
PickOffWallState --> ReleaseBallState : [wallPickOffDone]\n<i>startWait</i>
AlignPlacementState --> AlignWallState : [shouldPickOffWall]
AlignPlacementState --> AlignPlacementState : [!alignDone]\n<i>alignPlacement</i>
AlignPlacementState --> PlaceBallState : [alignDone]
PlaceBallState --> PlaceBallState : [!ballPlaced]\n<i>placeBall</i>
PlaceBallState --> ReleaseBallState : [ballPlaced]\n<i>startWait</i>
ReleaseBallState --> ReleaseBallState : [!waitDone && ballPlaced]\n<i>releaseBall</i>
ReleaseBallState --> StartState : [!ballPlaced]
ReleaseBallState --> RetreatState : [waitDone]
RetreatState --> Terminate:::terminate : [retreatDone && ballPlaced]
RetreatState --> RetreatState : [ballPlaced]\n<i>retreat</i>
RetreatState --> StartState : [!ballPlaced]

```

## [DefensePlayFSM](/src/software/ai/hl/stp/play/defense/defense_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> DefenseState
DefenseState --> DefenseState : <i>defendAgainstThreats</i>
Terminate:::terminate --> Terminate:::terminate

```

## [EnemyBallPlacementPlayFSM](/src/software/ai/hl/stp/play/enemy_ball_placement/enemy_ball_placement_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> WaitState
WaitState --> AvoidState : [hasPlacementPoint]\n<i>setPlacementPoint</i>
WaitState --> WaitState : [!hasPlacementPoint]
AvoidState --> AvoidState : [!isNearlyPlaced]\n<i>avoid</i>
AvoidState --> DefenseState : [isNearlyPlaced]
DefenseState --> DefenseState : [isNearlyPlaced]\n<i>enterDefensiveFormation</i>
DefenseState --> AvoidState : [!isNearlyPlaced]

```

## [EnemyFreeKickPlayFSM](/src/software/ai/hl/stp/play/enemy_free_kick/enemy_free_kick_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> BlockEnemyKickerState
BlockEnemyKickerState --> BlockEnemyKickerState : <i>blockEnemyKicker</i>
Terminate:::terminate --> Terminate:::terminate

```

## [FreeKickPlayFSM](/src/software/ai/hl/stp/play/free_kick/free_kick_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> SetupPositionState
SetupPositionState --> SetupPositionState : [!setupDone]\n<i>setupPosition</i>
SetupPositionState --> ShootState : [shotFound]
ShootState --> ShootState : [!shotDone]\n<i>shootBall</i>
ShootState --> Terminate:::terminate : [shotDone]
SetupPositionState --> AttemptPassState : <i>startLookingForPass</i>
AttemptPassState --> ChipState : [timeExpired]
AttemptPassState --> AttemptPassState : [!passFound]\n<i>lookForPass</i>
AttemptPassState --> PassState : [passFound]
PassState --> AttemptPassState : [shouldAbortPass]
PassState --> PassState : [!passDone]\n<i>passBall</i>
PassState --> Terminate:::terminate : [passDone]
ChipState --> ChipState : [!chipDone]\n<i>chipBall</i>
ChipState --> Terminate:::terminate : [chipDone]

```

## [PenaltyKickPlayFSM](/src/software/ai/hl/stp/play/penalty_kick/penalty_kick_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> SetupPositionState
SetupPositionState --> SetupPositionState : [!setupPositionDone]\n<i>setupPosition</i>
SetupPositionState --> PerformKickState : [setupPositionDone]
PerformKickState --> PerformKickState : [!kickDone]\n<i>performKick</i>
PerformKickState --> Terminate:::terminate : [kickDone]
Terminate:::terminate --> Terminate:::terminate

```

## [PenaltyKickEnemyPlayFSM](/src/software/ai/hl/stp/play/penalty_kick_enemy/penalty_kick_enemy_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> SetupPositionState
SetupPositionState --> SetupPositionState : [!setupPositionDone]\n<i>setupPosition</i>
SetupPositionState --> DefendKickState : [setupPositionDone]\n<i>defendKick</i>
DefendKickState --> DefendKickState : <i>defendKick</i>
Terminate:::terminate --> Terminate:::terminate

```

## [ChipSkillFSM](/src/software/ai/hl/stp/skill/chip/chip_skill_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> GetBehindBallSkillFSM
GetBehindBallSkillFSM --> GetBehindBallSkillFSM : <i>updateGetBehindBall</i>
GetBehindBallSkillFSM --> ChipState
ChipState --> GetBehindBallSkillFSM : [shouldRealignWithBall]\n<i>updateGetBehindBall</i>
ChipState --> ChipState : [!ballChicked]\n<i>updateChip</i>
ChipState --> Terminate:::terminate : [ballChicked]\n<i>SET_STOP_PRIMITIVE_ACTION</i>
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [DribbleSkillFSM](/src/software/ai/hl/stp/skill/dribble/dribble_skill_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> GetBallControl
GetBallControl --> Dribble : [haveBallControl]\n<i>dribble</i>
GetBallControl --> GetBallControl : <i>getBallControl</i>
Dribble --> GetBallControl : [lostBallControl]\n<i>getBallControl</i>
Dribble --> LoseBall : [shouldLoseBall && shouldExcessivelyDribble]\n<i>loseBall</i>
Dribble --> Terminate:::terminate : [shouldLoseBall && !shouldExcessivelyDribble]\n<i>dribble</i>
Dribble --> Terminate:::terminate : [dribblingDone]\n<i>dribble</i>
Dribble --> Dribble : <i>dribble</i>
LoseBall --> GetBallControl : [lostBallControl]\n<i>getBallControl</i>
LoseBall --> LoseBall : <i>loseBall</i>
Terminate:::terminate --> Terminate:::terminate : [!shouldExcessivelyDribble]\n<i>dribble</i>
Terminate:::terminate --> GetBallControl : [lostBallControl]\n<i>getBallControl</i>
Terminate:::terminate --> Dribble : [!dribblingDone]\n<i>dribble</i>
Terminate:::terminate --> Terminate:::terminate : <i>dribble</i>

```

## [GetBehindBallSkillFSM](/src/software/ai/hl/stp/skill/get_behind_ball/get_behind_ball_skill_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> GetBehindBallState
GetBehindBallState --> GetBehindBallState : [!behindBall]\n<i>updateMove</i>
GetBehindBallState --> Terminate:::terminate : [behindBall]\n<i>updateMove</i>
Terminate:::terminate --> GetBehindBallState : [!behindBall]\n<i>updateMove</i>
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [KeepAwaySkillFSM](/src/software/ai/hl/stp/skill/keep_away/keep_away_skill_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> DribbleSkillFSM
DribbleSkillFSM --> Terminate:::terminate : [!shouldKeepAway]\n<i>SET_STOP_PRIMITIVE_ACTION</i>
DribbleSkillFSM --> DribbleSkillFSM : <i>keepAway</i>
DribbleSkillFSM --> Terminate:::terminate
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [KickSkillFSM](/src/software/ai/hl/stp/skill/kick/kick_skill_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> GetBehindBallSkillFSM
GetBehindBallSkillFSM --> GetBehindBallSkillFSM : <i>updateGetBehindBall</i>
GetBehindBallSkillFSM --> KickState
KickState --> GetBehindBallSkillFSM : [shouldRealignWithBall]\n<i>updateGetBehindBall</i>
KickState --> KickState : [!ballChicked]\n<i>updateKick</i>
KickState --> Terminate:::terminate : [ballChicked]\n<i>SET_STOP_PRIMITIVE_ACTION</i>
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [PassSkillFSM](/src/software/ai/hl/stp/skill/pass/pass_skill_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> DribbleSkillFSM
DribbleSkillFSM --> PivotKickSkillFSM : [passFound]\n<i>takePass</i>
DribbleSkillFSM --> DribbleSkillFSM : <i>findPass</i>
DribbleSkillFSM --> PivotKickSkillFSM
PivotKickSkillFSM --> Terminate:::terminate : [shouldAbortPass]\n<i>abortPass</i>
PivotKickSkillFSM --> PivotKickSkillFSM : <i>takePass</i>
PivotKickSkillFSM --> Suspended
Suspended --> Terminate:::terminate : [passReceived_G||strayPass]\n<i>resetSkillState</i>
Suspended --> Suspended
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [PivotKickSkillFSM](/src/software/ai/hl/stp/skill/pivot_kick/pivot_kick_skill_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> StartState
StartState --> DribbleSkillFSM : <i>getBallControlAndPivot</i>
DribbleSkillFSM --> DribbleSkillFSM : <i>getBallControlAndPivot</i>
DribbleSkillFSM --> KickStartState
KickStartState --> KickStartState : <i>(setKickStartTime</i>
kickBall_A) --> KickState
KickState --> Terminate:::terminate : [ballKicked]\n<i>SET_STOP_PRIMITIVE_ACTION</i>
KickState --> DribbleSkillFSM : [lostBallControl]\n<i>getBallControlAndPivot</i>
KickState --> KickState : <i>kickBall</i>
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [ShootSkillFSM](/src/software/ai/hl/stp/skill/shoot/shoot_skill_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> DribbleSkillFSM
DribbleSkillFSM --> DribbleSkillFSM : <i>getBallControl</i>
DribbleSkillFSM --> PivotKickSkillFSM
PivotKickSkillFSM --> Terminate:::terminate : [shouldAbortShot]\n<i>abortShot</i>
PivotKickSkillFSM --> PivotKickSkillFSM : <i>pivotKick</i>
PivotKickSkillFSM --> Terminate:::terminate
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [CreaseDefenderFSM](/src/software/ai/hl/stp/tactic/crease_defender/crease_defender_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> MoveFSM
MoveFSM --> DribbleSkillFSM : [ballNearbyWithoutThreat]\n<i>prepareGetPossession</i>
MoveFSM --> MoveFSM : <i>blockThreat</i>
MoveFSM --> Terminate:::terminate
DribbleSkillFSM --> MoveFSM : [!ballNearbyWithoutThreat]\n<i>blockThreat</i>
DribbleSkillFSM --> DribbleSkillFSM : <i>prepareGetPossession</i>
Terminate:::terminate --> DribbleSkillFSM : [ballNearbyWithoutThreat]\n<i>prepareGetPossession</i>
Terminate:::terminate --> MoveFSM : <i>blockThreat</i>

```

## [GoalieFSM](/src/software/ai/hl/stp/tactic/goalie/goalie_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> PositionToBlock
PositionToBlock --> MoveToGoalLine : [shouldMoveToGoalLine]\n<i>moveToGoalLine</i>
PositionToBlock --> DribbleSkillFSM : [shouldEvacuateCrease]\n<i>retrieveFromDeadZone</i>
PositionToBlock --> Panic : [shouldPanic]\n<i>panic</i>
PositionToBlock --> PivotKickSkillFSM : [shouldPivotChip]\n<i>updatePivotKick</i>
PositionToBlock --> PositionToBlock : <i>positionToBlock</i>
DribbleSkillFSM --> PivotKickSkillFSM : [retrieveDone]\n<i>updatePivotKick</i>
DribbleSkillFSM --> MoveToGoalLine : [shouldMoveToGoalLine]\n<i>moveToGoalLine</i>
DribbleSkillFSM --> DribbleSkillFSM : [ballInInflatedDefenseArea]\n<i>retrieveFromDeadZone</i>
DribbleSkillFSM --> PositionToBlock : [!ballInInflatedDefenseArea]\n<i>positionToBlock</i>
Panic --> MoveToGoalLine : [shouldMoveToGoalLine]\n<i>moveToGoalLine</i>
Panic --> PivotKickSkillFSM : [shouldPivotChip]\n<i>updatePivotKick</i>
Panic --> PositionToBlock : [panicDone]\n<i>positionToBlock</i>
Panic --> Panic : <i>panic</i>
PivotKickSkillFSM --> MoveToGoalLine : [shouldMoveToGoalLine]\n<i>moveToGoalLine</i>
PivotKickSkillFSM --> PivotKickSkillFSM : [ballInInflatedDefenseArea]\n<i>updatePivotKick</i>
PivotKickSkillFSM --> PositionToBlock : [!ballInInflatedDefenseArea]\n<i>positionToBlock</i>
MoveToGoalLine --> MoveToGoalLine : [shouldMoveToGoalLine]\n<i>moveToGoalLine</i>
MoveToGoalLine --> PositionToBlock : [!shouldMoveToGoalLine]\n<i>positionToBlock</i>
Terminate:::terminate --> Terminate:::terminate

```

## [MoveFSM](/src/software/ai/hl/stp/tactic/move/move_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> MoveState
MoveState --> MoveState : [!moveDone]\n<i>updateMove</i>
MoveState --> Terminate:::terminate : [moveDone]\n<i>updateMove</i>
Terminate:::terminate --> MoveState : [!moveDone]\n<i>updateMove</i>
Terminate:::terminate --> Terminate:::terminate : <i>updateMove</i>

```

## [PassDefenderFSM](/src/software/ai/hl/stp/tactic/pass_defender/pass_defender_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> BlockPassState
BlockPassState --> InterceptBallState : [passStarted]\n<i>interceptBall</i>
BlockPassState --> BlockPassState : <i>blockPass</i>
InterceptBallState --> BlockPassState : [ballDeflected]\n<i>blockPass</i>
InterceptBallState --> DribbleSkillFSM : [ballNearbyWithoutThreat]\n<i>prepareGetPossession</i>
DribbleSkillFSM --> BlockPassState : [!ballNearbyWithoutThreat]\n<i>blockPass</i>
DribbleSkillFSM --> DribbleSkillFSM : <i>prepareGetPossession</i>
InterceptBallState --> InterceptBallState : <i>interceptBall</i>
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [PenaltyKickFSM](/src/software/ai/hl/stp/tactic/penalty_kick/penalty_kick_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> DribbleSkillFSM
DribbleSkillFSM --> DribbleSkillFSM : [!takePenaltyShot]\n<i>updateApproachKeeper</i>
DribbleSkillFSM --> KickSkillFSM : [timeOutApproach]\n<i>shoot</i>
DribbleSkillFSM --> DribbleSkillFSM : <i>adjustOrientationForShot</i>
DribbleSkillFSM --> KickSkillFSM
KickSkillFSM --> KickSkillFSM : <i>shoot</i>
KickSkillFSM --> Terminate:::terminate
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [ReceiverFSM](/src/software/ai/hl/stp/tactic/receiver/receiver_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> WaitingForPassState
WaitingForPassState --> WaitingForPassState : [!passStarted]\n<i>updateReceive</i>
WaitingForPassState --> OneTouchShotState : [passStarted && onetouchPossible]\n<i>updateOnetouch</i>
WaitingForPassState --> ReceiveAndDribbleState : [passStarted && !onetouchPossible]\n<i>updateReceive</i>
ReceiveAndDribbleState --> WaitingForPassState : [passReceivedByTeammate]\n<i>updateReceive</i>
ReceiveAndDribbleState --> DribbleSkillFSM : [strayPass_G||slowPass]\n<i>retrieveBall</i>
ReceiveAndDribbleState --> ReceiveAndDribbleState : <i>adjustReceive</i>
DribbleSkillFSM --> WaitingForPassState : [passReceivedByTeammate]\n<i>updateReceive</i>
DribbleSkillFSM --> DribbleSkillFSM : <i>retrieveBall</i>
OneTouchShotState --> OneTouchShotState : [!passReceived && !strayPass]\n<i>updateOnetouch</i>
OneTouchShotState --> ReceiveAndDribbleState : [!passReceived && strayPass]\n<i>adjustReceive</i>
OneTouchShotState --> WaitingForPassState : [passReceived]\n<i>updateOnetouch</i>
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [ShadowEnemyFSM](/src/software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> MoveFSM
MoveFSM --> BlockPassState : [!enemyThreatHasBall]\n<i>blockPass</i>
MoveFSM --> MoveFSM : <i>blockShot</i>
MoveFSM --> StealAndChipState
BlockPassState --> BlockPassState : [!enemyThreatHasBall]\n<i>blockPass</i>
BlockPassState --> MoveFSM : [enemyThreatHasBall]\n<i>blockShot</i>
StealAndChipState --> StealAndChipState : [enemyThreatHasBall]\n<i>stealAndChip</i>
StealAndChipState --> Terminate:::terminate : [!enemyThreatHasBall]\n<i>blockPass</i>
Terminate:::terminate --> BlockPassState : [!enemyThreatHasBall]\n<i>blockPass</i>
Terminate:::terminate --> MoveFSM : [enemyThreatHasBall]\n<i>blockShot</i>
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [StopFSM](/src/software/ai/hl/stp/tactic/stop/stop_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> StopState
StopState --> StopState : [!stopDone]\n<i>updateStop</i>
StopState --> Terminate:::terminate : [stopDone]\n<i>updateStop</i>
Terminate:::terminate --> StopState : [!stopDone]\n<i>updateStop</i>
Terminate:::terminate --> Terminate:::terminate : [stopDone]\n<i>updateStop</i>

```

