#pragma once

// We forward-declare all the tactics because if we include them we induce a circular
// dependency between the individual library for each tactic and this visitor. This is
// because tactic.h includes tactic_visitor.h, and each individual library includes
// tactic.h. Note: every subclass of this visitor must include all of the classes listed
// below
class ChipTactic;
class CreaseDefenderTactic;
class GetBehindBallTactic;
class GoalieTactic;
class KickTactic;
class KickoffChipTactic;
class MoveTactic;
class AttackerTactic;
class PassDefenderTactic;
class PenaltyKickTactic;
class PenaltySetupTactic;
class ReceiverTactic;
class ShadowEnemyTactic;
class StopTactic;
class MoveGoalieToGoalLineTactic;
class PrepareKickoffMoveTactic;
class PlaceBallMoveTactic;
class SkillTactic;

/**
 * Refer to the docs about why we use the Visitor Design Pattern
 */
class TacticVisitor
{
   public:
    virtual ~TacticVisitor() = default;

    // The javadoc comment for all methods here can be read as:
    /**
     * Visits an instance of X to perform an operation
     *
     * @param tactic The tactic to visit
     */

    virtual void visit(const ChipTactic &tactic)                 = 0;
    virtual void visit(const CreaseDefenderTactic &tactic)       = 0;
    virtual void visit(const GetBehindBallTactic &tactic)        = 0;
    virtual void visit(const GoalieTactic &tactic)               = 0;
    virtual void visit(const KickTactic &tactic)                 = 0;
    virtual void visit(const KickoffChipTactic &tactic)          = 0;
    virtual void visit(const MoveTactic &tactic)                 = 0;
    virtual void visit(const AttackerTactic &tactic)             = 0;
    virtual void visit(const PassDefenderTactic &tactic)         = 0;
    virtual void visit(const PenaltyKickTactic &tactic)          = 0;
    virtual void visit(const PenaltySetupTactic &tactic)         = 0;
    virtual void visit(const ReceiverTactic &tactic)             = 0;
    virtual void visit(const ShadowEnemyTactic &tactic)          = 0;
    virtual void visit(const StopTactic &tactic)                 = 0;
    virtual void visit(const MoveGoalieToGoalLineTactic &tactic) = 0;
    virtual void visit(const PrepareKickoffMoveTactic &tactic)   = 0;
    virtual void visit(const PlaceBallMoveTactic &tactic)        = 0;
    virtual void visit(const SkillTactic &tactic)                = 0;
};
