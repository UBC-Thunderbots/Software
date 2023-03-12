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
SetPlay --> Halt : [gameStateHalted]\n<i>setupHaltPlay</i>
SetPlay --> Stop : [gameStateStopped]\n<i>setupStopPlay</i>
SetPlay --> Playing : [gameStatePlaying]\n<i>setupOffensePlay</i>
Terminate:::terminate --> Terminate:::terminate

```

## [AttackerFSM](/src/software/ai/hl/stp/tactic/attacker/attacker_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> DribbleFSM
DribbleFSM --> PivotKickFSM : [shouldKick]\n<i>pivotKick</i>
DribbleFSM --> DribbleFSM : [!shouldKick]\n<i>keepAway</i>
PivotKickFSM --> PivotKickFSM : <i>pivotKick</i>
PivotKickFSM --> Terminate:::terminate
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [ReceiverFSM](/src/software/ai/hl/stp/tactic/receiver/receiver_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> WaitingForPassState
WaitingForPassState --> WaitingForPassState : [!passStarted]\n<i>updateReceive</i>
WaitingForPassState --> OneTouchShotState : [passStarted_G&&onetouchPossible]\n<i>updateOnetouch</i>
WaitingForPassState --> ReceiveAndDribbleState : [passStarted_G&&!onetouchPossible]\n<i>updateReceive</i>
ReceiveAndDribbleState --> ReceiveAndDribbleState : [!passFinished]\n<i>adjustReceive</i>
OneTouchShotState --> OneTouchShotState : [!passFinished_G&&!strayPass]\n<i>updateOnetouch</i>
OneTouchShotState --> ReceiveAndDribbleState : [!passFinished_G&&strayPass]\n<i>adjustReceive</i>
ReceiveAndDribbleState --> Terminate:::terminate : [passFinished]\n<i>adjustReceive</i>
OneTouchShotState --> Terminate:::terminate : [passFinished]\n<i>updateOnetouch</i>
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

## [DribbleFSM](/src/software/ai/hl/stp/tactic/dribble/dribble_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> GetPossession
GetPossession --> Dribble : [havePossession]\n<i>startDribble</i>
GetPossession --> GetPossession : [!havePossession]\n<i>getPossession</i>
Dribble --> GetPossession : [lostPossession]\n<i>getPossession</i>
Dribble --> LoseBall : [shouldLoseBall]\n<i>loseBall</i>
Dribble --> Dribble : [!dribblingDone]\n<i>dribble</i>
Dribble --> Terminate:::terminate : [dribblingDone]\n<i>dribble</i>
LoseBall --> LoseBall : [!lostPossession]\n<i>loseBall</i>
LoseBall --> GetPossession : [lostPossession]\n<i>getPossession</i>
Terminate:::terminate --> GetPossession : [lostPossession]\n<i>getPossession</i>
Terminate:::terminate --> Dribble : [!dribblingDone]\n<i>dribble</i>
Terminate:::terminate --> Terminate:::terminate : <i>dribble</i>

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

## [PivotKickFSM](/src/software/ai/hl/stp/tactic/pivot_kick/pivot_kick_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> StartState
StartState --> DribbleFSM : <i>getPossessionAndPivot</i>
DribbleFSM --> DribbleFSM : <i>getPossessionAndPivot</i>
DribbleFSM --> KickState
KickState --> KickState : [!ballKicked]\n<i>kickBall</i>
KickState --> Terminate:::terminate : [ballKicked]\n<i>SET_STOP_PRIMITIVE_ACTION</i>
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [PenaltyKickFSM](/src/software/ai/hl/stp/tactic/penalty_kick/penalty_kick_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> DribbleFSM
DribbleFSM --> DribbleFSM : [!takePenaltyShot]\n<i>updateApproachKeeper</i>
DribbleFSM --> KickFSM : [timeOutApproach]
DribbleFSM --> DribbleFSM : <i>adjustOrientationForShot</i>
DribbleFSM --> KickFSM
KickFSM --> KickFSM : <i>shoot</i>
KickFSM --> Terminate:::terminate
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [GetBehindBallFSM](/src/software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h)

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

## [CreaseDefenderFSM](/src/software/ai/hl/stp/tactic/crease_defender/crease_defender_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> MoveFSM
MoveFSM --> MoveFSM : <i>blockThreat</i>
MoveFSM --> Terminate:::terminate
Terminate:::terminate --> MoveFSM : <i>blockThreat</i>

```

## [KickFSM](/src/software/ai/hl/stp/tactic/kick/kick_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> GetBehindBallFSM
GetBehindBallFSM --> GetBehindBallFSM : <i>updateGetBehindBall</i>
GetBehindBallFSM --> KickState
KickState --> KickState : [!ballChicked]\n<i>updateKick</i>
KickState --> Terminate:::terminate : [ballChicked]\n<i>SET_STOP_PRIMITIVE_ACTION</i>
Terminate:::terminate --> Terminate:::terminate : <i>SET_STOP_PRIMITIVE_ACTION</i>

```

## [GoalieFSM](/src/software/ai/hl/stp/tactic/goalie/goalie_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> PositionToBlock
PositionToBlock --> MoveToGoalLine : [shouldMoveToGoalLine]\n<i>moveToGoalLine</i>
PositionToBlock --> Panic : [shouldPanic]\n<i>panic</i>
PositionToBlock --> PivotKickFSM : [shouldPivotChip]\n<i>updatePivotKick</i>
PositionToBlock --> PositionToBlock : <i>positionToBlock</i>
Panic --> MoveToGoalLine : [shouldMoveToGoalLine]\n<i>moveToGoalLine</i>
Panic --> PivotKickFSM : [shouldPivotChip]\n<i>updatePivotKick</i>
Panic --> PositionToBlock : [panicDone]\n<i>positionToBlock</i>
Panic --> Panic : <i>panic</i>
PivotKickFSM --> MoveToGoalLine : [shouldMoveToGoalLine]\n<i>moveToGoalLine</i>
PivotKickFSM --> PivotKickFSM : [ballInDefenseArea]\n<i>updatePivotKick</i>
PivotKickFSM --> PositionToBlock : [!ballInDefenseArea]\n<i>positionToBlock</i>
MoveToGoalLine --> MoveToGoalLine : [shouldMoveToGoalLine]\n<i>moveToGoalLine</i>
MoveToGoalLine --> PositionToBlock : [!shouldMoveToGoalLine]\n<i>positionToBlock</i>
Terminate:::terminate --> Terminate:::terminate

```

## [ChipFSM](/src/software/ai/hl/stp/tactic/chip/chip_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> GetBehindBallFSM
GetBehindBallFSM --> GetBehindBallFSM : <i>updateGetBehindBall</i>
GetBehindBallFSM --> ChipState
ChipState --> ChipState : [!ballChicked]\n<i>updateChip</i>
ChipState --> Terminate:::terminate : [ballChicked]\n<i>SET_STOP_PRIMITIVE_ACTION</i>
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

## [CreaseDefensePlayFSM](/src/software/ai/hl/stp/play/crease_defense/crease_defense_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> DefenseState
DefenseState --> DefenseState : <i>defendDefenseArea</i>
Terminate:::terminate --> Terminate:::terminate

```

## [ShootOrPassPlayFSM](/src/software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play_fsm.h)

```mermaid

stateDiagram-v2
classDef terminate fill:white,color:black,font-weight:bold
direction LR
[*] --> StartState
StartState --> AttemptShotState : <i>startLookingForPass</i>
AttemptShotState --> TakePassState : [passFound]\n<i>takePass</i>
AttemptShotState --> Terminate:::terminate : [tookShot]
AttemptShotState --> AttemptShotState : [!passFound]\n<i>lookForPass</i>
TakePassState --> AttemptShotState : [shouldAbortPass]\n<i>startLookingForPass</i>
TakePassState --> TakePassState : [!passCompleted]\n<i>takePass</i>
TakePassState --> Terminate:::terminate : [passCompleted]\n<i>takePass</i>
Terminate:::terminate --> AttemptShotState : <i>startLookingForPass</i>

```

