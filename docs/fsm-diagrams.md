# Play and Tactic FSM Diagrams

## [PlaySelectionFSM](/src/software/ai/play_selection_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> Halt
Halt --> Stop : [gameStateStopped]<br><i>setupStopPlay</i>
Halt --> Playing : [gameStatePlaying]<br><i>setupOffensePlay</i>
Halt --> SetPlay : [gameStateSetupRestart]<br><i>setupSetPlay</i>
Halt --> Halt : <i>setupOverridePlay</i>
Halt --> Halt : <i>resetPlaySelection</i>
Stop --> Halt : [gameStateHalted]<br><i>setupHaltPlay</i>
Stop --> Playing : [gameStatePlaying]<br><i>setupOffensePlay</i>
Stop --> SetPlay : [gameStateSetupRestart]<br><i>setupSetPlay</i>
Stop --> Stop : <i>setupOverridePlay</i>
Stop --> Halt : <i>resetPlaySelection</i>
Playing --> Halt : [gameStateHalted]<br><i>setupHaltPlay</i>
Playing --> Stop : [gameStateStopped]<br><i>setupStopPlay</i>
Playing --> SetPlay : [gameStateSetupRestart]<br><i>setupSetPlay</i>
Playing --> Playing : <i>setupOverridePlay</i>
Playing --> Halt : <i>resetPlaySelection</i>
SetPlay --> Halt : [gameStateHalted]<br><i>resetSetPlay, setupHaltPlay</i>
SetPlay --> Stop : [gameStateStopped]<br><i>resetSetPlay, setupStopPlay</i>
SetPlay --> Playing : [gameStatePlaying]<br><i>resetSetPlay, setupOffensePlay</i>
SetPlay --> SetPlay : [gameStateSetupRestart]<br><i>setupSetPlay</i>
SetPlay --> SetPlay : <i>setupOverridePlay</i>
SetPlay --> Halt : <i>resetPlaySelection</i>
Terminate:::terminate --> Terminate:::terminate

```

## [StopPlayFSM](/src/software/ai/stp/play/stop_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> StopState
StopState --> StopState : <i>updateStopPosition</i>

```

## [BallPlacementPlayFSM](/src/software/ai/stp/play/ball_placement/ball_placement_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> StartState
StartState --> AlignPlacementState : [!shouldKickOffWall]<br><i>alignPlacement</i>
StartState --> KickOffWallState : [shouldKickOffWall]
KickOffWallState --> KickOffWallState : [!kickDone && shouldKickOffWall]<br><i>kickOffWall</i>
KickOffWallState --> KickOffWallState : [kickDone]
KickOffWallState --> AlignPlacementState : [!kickDone]
AlignPlacementState --> KickOffWallState : [shouldKickOffWall]
AlignPlacementState --> AlignPlacementState : [!alignDone]<br><i>alignPlacement</i>
AlignPlacementState --> PlaceBallState : [alignDone]
PlaceBallState --> PlaceBallState : [!ballPlaced]<br><i>placeBall</i>
PlaceBallState --> WaitState : [ballPlaced]<br><i>startWait</i>
WaitState --> WaitState : [!waitDone]
WaitState --> RetreatState : [waitDone]
RetreatState --> Terminate:::terminate : [retreatDone && ballPlaced]
RetreatState --> RetreatState : [ballPlaced]<br><i>retreat</i>

```

## [CreaseDefensePlayFSM](/src/software/ai/stp/play/crease_defense/crease_defense_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> DefenseState
DefenseState --> DefenseState : <i>defendDefenseArea</i>
Terminate:::terminate --> Terminate:::terminate

```

## [DefensePlayFSM](/src/software/ai/stp/play/defense/defense_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> DefenseState
DefenseState --> AggressiveDefenseState : [shouldDefendAggressively]<br><i>shadowAndBlockShots</i>
DefenseState --> DefenseState : <i>blockShots</i>
AggressiveDefenseState --> DefenseState : [!shouldDefendAggressively]<br><i>blockShots</i>
AggressiveDefenseState --> AggressiveDefenseState : <i>shadowAndBlockShots</i>
Terminate:::terminate --> Terminate:::terminate

```

## [EnemyBallPlacementPlayFSM](/src/software/ai/stp/play/enemy_ball_placement/enemy_ball_placement_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> WaitState
WaitState --> AvoidState : [hasPlacementPoint]<br><i>setPlacementPoint</i>
WaitState --> WaitState : [!hasPlacementPoint]
AvoidState --> AvoidState : [!isNearlyPlaced]<br><i>avoid</i>
AvoidState --> DefenseState : [isNearlyPlaced]
DefenseState --> DefenseState : [isNearlyPlaced]<br><i>enterDefensiveFormation</i>
DefenseState --> AvoidState : [!isNearlyPlaced]

```

## [EnemyFreeKickPlayFSM](/src/software/ai/stp/play/enemy_free_kick/enemy_free_kick_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> BlockEnemyKickerState
BlockEnemyKickerState --> BlockEnemyKickerState : <i>blockEnemyKicker</i>
Terminate:::terminate --> Terminate:::terminate

```

## [ExamplePlayFSM](/src/software/ai/stp/play/example/example_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> MoveState
MoveState --> MoveState : <i>moveToPosition</i>
Terminate:::terminate --> Terminate:::terminate

```

## [FreeKickPlayFSM](/src/software/ai/stp/play/free_kick/free_kick_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> SetupPositionState
SetupPositionState --> SetupPositionState : [!setupDone]<br><i>setupPosition</i>
SetupPositionState --> ShootState : [shotFound]
ShootState --> ShootState : [!shotDone]<br><i>shootBall</i>
ShootState --> Terminate:::terminate : [shotDone]
SetupPositionState --> AttemptPassState : <i>startLookingForPass</i>
AttemptPassState --> ChipState : [timeExpired]
AttemptPassState --> AttemptPassState : [!passFound]<br><i>lookForPass</i>
AttemptPassState --> PassState : [passFound]
PassState --> AttemptPassState : [shouldAbortPass]
PassState --> PassState : [!passDone]<br><i>passBall</i>
PassState --> Terminate:::terminate : [passDone]
ChipState --> ChipState : [!chipDone]<br><i>chipBall</i>
ChipState --> Terminate:::terminate : [chipDone]

```

## [HaltPlayFSM](/src/software/ai/stp/play/halt_play/halt_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> HaltState
HaltState --> HaltState : <i>updateStop</i>
Terminate:::terminate --> Terminate:::terminate : <i>updateStop</i>

```

## [KickoffEnemyPlayFSM](/src/software/ai/stp/play/kickoff_enemy/kickoff_enemy_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> SetupState
SetupState --> SetupState : <i>kickoff</i>

```

## [KickoffFriendlyPlayFSM](/src/software/ai/stp/play/kickoff_friendly/kickoff_friendly_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> SetupState
SetupState --> SetupState : [!isSetupDone]<br><i>setupKickoff</i>
SetupState --> ChipState : [isSetupDone]
ChipState --> ChipState : [!isPlaying]<br><i>chipBall</i>
ChipState --> Terminate:::terminate : [isPlaying]
Terminate:::terminate --> Terminate:::terminate

```

## [OffensePlayFSM](/src/software/ai/stp/play/offense/offense_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> OffensiveState
OffensiveState --> DefensiveState : [enemyHasPossession]<br><i>setupDefensiveStrategy</i>
OffensiveState --> OffensiveState : <i>setupOffensiveStrategy</i>
DefensiveState --> OffensiveState : [!enemyHasPossession]<br><i>setupOffensiveStrategy</i>
DefensiveState --> DefensiveState : <i>setupDefensiveStrategy</i>
Terminate:::terminate --> Terminate:::terminate

```

## [PenaltyKickPlayFSM](/src/software/ai/stp/play/penalty_kick/penalty_kick_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> SetupPositionState
SetupPositionState --> SetupPositionState : [!setupPositionDone]<br><i>setupPosition</i>
SetupPositionState --> PerformKickState : [setupPositionDone]
PerformKickState --> PerformKickState : [!kickDone]<br><i>performKick</i>
PerformKickState --> Terminate:::terminate : [kickDone]
Terminate:::terminate --> Terminate:::terminate

```

## [PenaltyKickEnemyPlayFSM](/src/software/ai/stp/play/penalty_kick_enemy/penalty_kick_enemy_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> SetupPositionState
SetupPositionState --> SetupPositionState : [!setupPositionDone]<br><i>setupPosition</i>
SetupPositionState --> DefendKickState : [setupPositionDone]<br><i>defendKick</i>
DefendKickState --> DefendKickState : <i>defendKick</i>
Terminate:::terminate --> Terminate:::terminate

```

## [ShootOrChipPlayFSM](/src/software/ai/stp/play/shoot_or_chip/shoot_or_chip_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> ShootOrChipState
ShootOrChipState --> ShootOrChipState : [!attackerDone]<br><i>updateShootOrChip</i>
ShootOrChipState --> Terminate:::terminate : [attackerDone]<br><i>updateShootOrChip</i>
Terminate:::terminate --> Terminate:::terminate

```

## [ShootOrPassPlayFSM](/src/software/ai/stp/play/shoot_or_pass/shoot_or_pass_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> StartState
StartState --> AttemptShotState : <i>startLookingForPass</i>
AttemptShotState --> TakePassState : [passFound]<br><i>takePass</i>
AttemptShotState --> Terminate:::terminate : [tookShot]
AttemptShotState --> AttemptShotState : [!passFound]<br><i>lookForPass</i>
TakePassState --> AttemptShotState : [shouldAbortPass]<br><i>startLookingForPass</i>
TakePassState --> TakePassState : [!passCompleted]<br><i>takePass</i>
TakePassState --> Terminate:::terminate : [passCompleted]<br><i>takePass</i>
Terminate:::terminate --> AttemptShotState : <i>startLookingForPass</i>

```

## [AttackerFSM](/src/software/ai/stp/tactic/attacker/attacker_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> DribbleFSM
DribbleFSM --> PivotKickFSM : [shouldKick]<br><i>pivotKick</i>
DribbleFSM --> KeepAwayFSM : [!shouldKick]<br><i>keepAway</i>
KeepAwayFSM --> PivotKickFSM : [shouldKick]<br><i>pivotKick</i>
KeepAwayFSM --> KeepAwayFSM : <i>keepAway</i>
KeepAwayFSM --> DribbleFSM
PivotKickFSM --> PivotKickFSM : <i>pivotKick</i>
PivotKickFSM --> Terminate:::terminate
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [CreaseDefenderFSM](/src/software/ai/stp/tactic/crease_defender/crease_defender_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> MoveFSM
MoveFSM --> DribbleFSM : [ballNearbyWithoutThreat]<br><i>prepareGetPossession</i>
MoveFSM --> MoveFSM : <i>blockThreat</i>
MoveFSM --> Terminate:::terminate
DribbleFSM --> MoveFSM : [!ballNearbyWithoutThreat]<br><i>blockThreat</i>
DribbleFSM --> DribbleFSM : <i>prepareGetPossession</i>
Terminate:::terminate --> DribbleFSM : [ballNearbyWithoutThreat]<br><i>prepareGetPossession</i>
Terminate:::terminate --> MoveFSM : <i>blockThreat</i>

```

## [DribbleFSM](/src/software/ai/stp/tactic/dribble/dribble_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> GetPossession
GetPossession --> Dribble : [havePossession]<br><i>dribble</i>
GetPossession --> GetPossession : [!havePossession]<br><i>getPossession</i>
Dribble --> LoseBall : [shouldLoseBall]<br><i>loseBall</i>
Dribble --> GetPossession : [lostPossession]<br><i>getPossession</i>
Dribble --> Dribble : [!dribblingDone]<br><i>dribble</i>
Dribble --> Terminate:::terminate : [dribblingDone]<br><i>dribble</i>
LoseBall --> LoseBall : [shouldLoseBall]<br><i>loseBall</i>
LoseBall --> GetPossession : [!shouldLoseBall]<br><i>getPossession</i>
Terminate:::terminate --> GetPossession : [lostPossession]<br><i>getPossession</i>
Terminate:::terminate --> Dribble : [!dribblingDone]<br><i>dribble</i>
Terminate:::terminate --> Terminate:::terminate : <i>dribble</i>

```

## [GetBehindBallFSM](/src/software/ai/stp/tactic/get_behind_ball/get_behind_ball_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> GetBehindBallState
GetBehindBallState --> GetBehindBallState : [!behindBall]<br><i>updateMove</i>
GetBehindBallState --> Terminate:::terminate : [behindBall]<br><i>updateMove</i>
Terminate:::terminate --> GetBehindBallState : [!behindBall]<br><i>updateMove</i>
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [GoalieFSM](/src/software/ai/stp/tactic/goalie/goalie_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> PositionToBlock
PositionToBlock --> MoveToGoalLine : [shouldMoveToGoalLine]<br><i>moveToGoalLine</i>
PositionToBlock --> DribbleFSM : [shouldEvacuateCrease]<br><i>retrieveFromDeadZone</i>
PositionToBlock --> Panic : [shouldPanic]<br><i>panic</i>
PositionToBlock --> PivotKickFSM : [shouldPivotChip]<br><i>updatePivotKick</i>
PositionToBlock --> PositionToBlock : <i>positionToBlock</i>
DribbleFSM --> PivotKickFSM : [retrieveDone]<br><i>updatePivotKick</i>
DribbleFSM --> MoveToGoalLine : [shouldMoveToGoalLine]<br><i>moveToGoalLine</i>
DribbleFSM --> DribbleFSM : [ballInInflatedDefenseArea]<br><i>retrieveFromDeadZone</i>
DribbleFSM --> PositionToBlock : [!ballInInflatedDefenseArea]<br><i>positionToBlock</i>
Panic --> MoveToGoalLine : [shouldMoveToGoalLine]<br><i>moveToGoalLine</i>
Panic --> PivotKickFSM : [shouldPivotChip]<br><i>updatePivotKick</i>
Panic --> PositionToBlock : [panicDone]<br><i>positionToBlock</i>
Panic --> Panic : <i>panic</i>
PivotKickFSM --> MoveToGoalLine : [shouldMoveToGoalLine]<br><i>moveToGoalLine</i>
PivotKickFSM --> PivotKickFSM : [ballInInflatedDefenseArea]<br><i>updatePivotKick</i>
PivotKickFSM --> PositionToBlock : [!ballInInflatedDefenseArea]<br><i>positionToBlock</i>
MoveToGoalLine --> MoveToGoalLine : [shouldMoveToGoalLine]<br><i>moveToGoalLine</i>
MoveToGoalLine --> PositionToBlock : [!shouldMoveToGoalLine]<br><i>positionToBlock</i>
Terminate:::terminate --> Terminate:::terminate

```

## [HaltFSM](/src/software/ai/stp/tactic/halt/halt_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> StopState
StopState --> StopState : [!stopDone]<br><i>updateStop</i>
StopState --> Terminate:::terminate : [stopDone]<br><i>updateStop</i>
Terminate:::terminate --> StopState : [!stopDone]<br><i>updateStop</i>
Terminate:::terminate --> Terminate:::terminate : [stopDone]<br><i>updateStop</i>

```

## [KeepAwayFSM](/src/software/ai/stp/tactic/keep_away/keep_away_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> DribbleFSM
DribbleFSM --> DribbleFSM : <i>keepAway</i>
DribbleFSM --> Terminate:::terminate
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [KickOrChipFSM](/src/software/ai/stp/tactic/kick_or_chip/kick_or_chip_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> GetBehindBallFSM
GetBehindBallFSM --> GetBehindBallFSM : <i>updateGetBehindBall</i>
GetBehindBallFSM --> KickOrChipState
KickOrChipState --> GetBehindBallFSM : [shouldRealignWithBall]<br><i>updateGetBehindBall</i>
KickOrChipState --> KickOrChipState : [!ballChicked]<br><i>kickOrChipBall</i>
KickOrChipState --> Terminate:::terminate : [ballChicked]<br><i>SET_STOP_PRIMITIVE_ACTION</i>
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [MoveFSM](/src/software/ai/stp/tactic/move/move_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> MoveState
MoveState --> MoveState : [!moveDone]<br><i>updateMove</i>
MoveState --> Terminate:::terminate : [moveDone]<br><i>updateMove</i>
Terminate:::terminate --> MoveState : [!moveDone]<br><i>updateMove</i>
Terminate:::terminate --> Terminate:::terminate : <i>updateMove</i>

```

## [PassDefenderFSM](/src/software/ai/stp/tactic/pass_defender/pass_defender_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> BlockPassState
BlockPassState --> InterceptBallState : [passStarted]<br><i>interceptBall</i>
BlockPassState --> BlockPassState : <i>blockPass</i>
InterceptBallState --> BlockPassState : [ballDeflected]<br><i>blockPass</i>
InterceptBallState --> DribbleFSM : [ballNearbyWithoutThreat]<br><i>prepareGetPossession</i>
DribbleFSM --> BlockPassState : [!ballNearbyWithoutThreat]<br><i>blockPass</i>
DribbleFSM --> DribbleFSM : <i>prepareGetPossession</i>
InterceptBallState --> InterceptBallState : <i>interceptBall</i>
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [PenaltyKickFSM](/src/software/ai/stp/tactic/penalty_kick/penalty_kick_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> DribbleFSM
DribbleFSM --> DribbleFSM : [!takePenaltyShot]<br><i>updateApproachKeeper</i>
DribbleFSM --> KickOrChipFSM : [timeOutApproach]<br><i>shoot</i>
DribbleFSM --> DribbleFSM : <i>adjustOrientationForShot</i>
DribbleFSM --> KickOrChipFSM
KickOrChipFSM --> KickOrChipFSM : <i>shoot</i>
KickOrChipFSM --> Terminate:::terminate
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [PivotKickFSM](/src/software/ai/stp/tactic/pivot_kick/pivot_kick_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> StartState
StartState --> DribbleFSM : <i>getPossessionAndPivot</i>
DribbleFSM --> DribbleFSM : <i>getPossessionAndPivot</i>
DribbleFSM --> KickState
KickState --> KickState : [!ballKicked]<br><i>kickBall</i>
KickState --> Terminate:::terminate : [ballKicked]<br><i>SET_STOP_PRIMITIVE_ACTION</i>
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [ReceiverFSM](/src/software/ai/stp/tactic/receiver/receiver_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> WaitingForPassState
WaitingForPassState --> WaitingForPassState : [!passStarted]<br><i>updateReceive</i>
WaitingForPassState --> OneTouchShotState : [passStarted && onetouchPossible]<br><i>updateOnetouch</i>
WaitingForPassState --> ReceiveAndDribbleState : [passStarted && !onetouchPossible]<br><i>updateReceive</i>
ReceiveAndDribbleState --> ReceiveAndDribbleState : [!passFinished]<br><i>adjustReceive</i>
OneTouchShotState --> OneTouchShotState : [!passFinished && !strayPass]<br><i>updateOnetouch</i>
OneTouchShotState --> ReceiveAndDribbleState : [!passFinished && strayPass]<br><i>adjustReceive</i>
ReceiveAndDribbleState --> Terminate:::terminate : [passFinished]<br><i>adjustReceive</i>
OneTouchShotState --> Terminate:::terminate : [passFinished]<br><i>updateOnetouch</i>
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [ShadowEnemyFSM](/src/software/ai/stp/tactic/shadow_enemy/shadow_enemy_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> MoveFSM
MoveFSM --> BlockPassState : [!enemyThreatHasBall]<br><i>blockPass</i>
MoveFSM --> GoAndStealState : [blockedShot]<br><i>goAndSteal</i>
MoveFSM --> MoveFSM : <i>blockShot</i>
MoveFSM --> GoAndStealState
BlockPassState --> BlockPassState : [!enemyThreatHasBall]<br><i>blockPass</i>
BlockPassState --> MoveFSM : [enemyThreatHasBall]<br><i>blockShot</i>
GoAndStealState --> GoAndStealState : [enemyThreatHasBall && !contestedBall]<br><i>goAndSteal</i>
GoAndStealState --> StealAndPullState : [enemyThreatHasBall && contestedBall]<br><i>goAndSteal</i>
GoAndStealState --> Terminate:::terminate : [!enemyThreatHasBall]<br><i>blockPass</i>
StealAndPullState --> StealAndPullState : [enemyThreatHasBall]<br><i>stealAndPull</i>
StealAndPullState --> Terminate:::terminate : [!enemyThreatHasBall]<br><i>blockPass</i>
Terminate:::terminate --> BlockPassState : [!enemyThreatHasBall]<br><i>blockPass</i>
Terminate:::terminate --> MoveFSM : [enemyThreatHasBall]<br><i>blockShot</i>
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

