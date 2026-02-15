#include "software/ai/hl/stp/play/shoot_or_chip/shoot_or_chip_play_fsm.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/evaluation/find_open_areas.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/halt/halt_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"

ShootOrChipPlayFSM::ShootOrChipPlayFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
          : PlayFSM<ShootOrChipPlayFSM>(ai_config_ptr),
			crease_defender_tactics{
			std::make_shared<CreaseDefenderTactic>(ai_config_ptr),
			std::make_shared<CreaseDefenderTactic>(ai_config_ptr),
			},
        	move_to_open_area_tactics{
			std::make_shared<MoveTactic>(ai_config_ptr),
        	std::make_shared<MoveTactic>(ai_config_ptr),
			},
        	attacker(std::make_shared<AttackerTactic>(ai_config_ptr)){

			}

void ShootOrChipPlayFSM::updateShootOrChip(const Update& event){

    /**
     * Our general strategy here is:
     * - 1 goalie
     * - 2 crease defenders moving around the friendly defense box
     * - 2 robots moving into the first and second largest open free space on
     *   the enemy half
     * - 1 robot trying to shoot on the goal. If an enemy gets too close to this
     *   robot, it will chip to right in front of the robot in the largest open free area
     */
		
    // Figure out where the fallback chip target is
    // Experimentally determined to be a reasonable value
    double fallback_chip_target_x_offset = 1.5;

    Point fallback_chip_target =
        event.common.world_ptr->field().enemyGoalCenter() - Vector(fallback_chip_target_x_offset, 0);

        // Update chipper
        std::optional<Point> chip_target = fallback_chip_target;


        PriorityTacticVector result = {{}};

        // Update crease defenders
        std::get<0>(crease_defender_tactics)
            ->updateControlParams(event.common.world_ptr->ball().position(),
                                  TbotsProto::CreaseDefenderAlignment::LEFT);
        result[0].emplace_back(std::get<0>(crease_defender_tactics));
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(event.common.world_ptr->ball().position(),
                                  TbotsProto::CreaseDefenderAlignment::RIGHT);
        result[0].emplace_back(std::get<1>(crease_defender_tactics));

        // Update tactics moving to open areas
        std::vector<Circle> chip_targets = findGoodChipTargets(*event.common.world_ptr);
        for (unsigned i = 0;
             i < chip_targets.size() && i < move_to_open_area_tactics.size(); i++)
        {
            // Face towards the ball
            Angle orientation =
                (event.common.world_ptr->ball().position() - chip_targets[i].origin()).orientation();
            // Move a bit backwards to make it more likely we'll receive the chip
            Point position =
                chip_targets[i].origin() -
                Vector::createFromAngle(orientation).normalize(ROBOT_MAX_RADIUS_METERS);
            ;
            move_to_open_area_tactics[i]->updateControlParams(position, orientation);
            result[0].emplace_back(move_to_open_area_tactics[i]);
        }

        if (!chip_targets.empty())
        {
            chip_target = chip_targets[0].origin();
        }
        attacker->updateControlParams(chip_target);

        // We want this second in priority only to the goalie
        result[0].insert(result[0].begin() + 1, attacker);

        // set the the Tactics this Play wants to run, in order of priority
        event.common.set_tactics(result);
}

bool ShootOrChipPlayFSM::attackerDone(const Update& event){
	return attacker->done();
}
	
