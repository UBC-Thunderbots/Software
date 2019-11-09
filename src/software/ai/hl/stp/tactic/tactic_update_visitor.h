#pragma once

#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/world/world.h"
#include "software/ai/hl/stp/tactic/all_tactics.h"
#include "software/ai/hl/stp/tactic/grab_ball_tactic.h"

class TacticUpdateVisitor : public TacticVisitor
{
public:
    /**
     * Creates a new TacticUpdateVisitor
     *
     * @param world The World we would like to update the Tactics with
     */
    explicit TacticUpdateVisitor(const World &world)
    {
        this->world = world;
    }

    /**
     * Calls the updateWorldParam in CherryPickTactic to update params
     *
     * @param tactic The CherryPickTactic to update
     */
    void visit(CherryPickTactic& tactic);

    /**
     * Calls the updateWorldParam in ChipTactic to update params
     *
     * @param tactic The ChipTactic to update
     */
    void visit(ChipTactic& tactic);

    /**
     * Calls the updateWorldParam in CreaseDefenderTactic to update params
     *
     * @param tactic The CreaseDefenderTactic to update
     */
    void visit(CreaseDefenderTactic& tactic);

    /**
     * Calls the updateWorldParam in DefenseShadowEnemyTactic to update params
     *
     * @param tactic The DefenseShadowEnemyTactic to update
     */
    void visit(DefenseShadowEnemyTactic& tactic);

    /**
     * Calls the updateWorldParam in GoalieTactic to update params
     *
     * @param tactic The GoalieTactic to update
     */
    void visit(GoalieTactic& tactic);

    /**
     * Calls the updateWorldParam in GrabBallTactic to update params
     *
     * @param tactic The GrabBallTactic to update
     */
    void visit(GrabBallTactic& tactic);

    /**
     * Calls the updateWorldParam in PasserTactic to update params
     *
     * @param tactic The PasserTactic to update
     */
    void visit(PasserTactic& tactic);

    /**
     * Calls the updateWorldParam in PenaltyKickTactic to update params
     *
     * @param tactic The PenaltyKickTactic to update
     */
    void visit(PenaltyKickTactic& tactic);

    /**
     * Calls the updateWorldParam in ReceiverTactic to update params
     *
     * @param tactic The ReceiverTactic to update
     */
    void visit(ReceiverTactic& tactic);

    /**
     * Calls the updateWorldParam in ShadowEnemyTactic to update params
     *
     * @param tactic The ShadowEnemyTactic to update
     */
    void visit(ShadowEnemyTactic& tactic);

    /**
     * Calls the updateWorldParam in ShadowFreekickerTactic to update params
     *
     * @param tactic The ShadowFreekickerTactic to update
     */
    void visit(ShadowFreekickerTactic& tactic);

    /**
     * Calls the updateWorldParam in ShootGoalTactic to update params
     *
     * @param tactic The ShootGoalTactic to update
     */
    void visit(ShootGoalTactic& tactic);

    World world;
};
