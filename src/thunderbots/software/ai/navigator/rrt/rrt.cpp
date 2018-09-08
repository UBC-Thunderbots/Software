#include "rrt.h"

RRTNav::RRTNav()
{
}

std::vector<std::unique_ptr<Primitive>> RRTNav::getAssignedPrimitives(
    const World &world, const std::vector<std::unique_ptr<Intent>> &assignedIntents) const
{
    std::vector<std::unique_ptr<Primitive>> assigned_primitives =
        std::vector<std::unique_ptr<Primitive>>();

    // Get vectors of robot obstacles
    // TODO: do something with these for path planning
    std::vector<RobotObstacle> friendly_obsts = generate_friendly_obstacles(
        world.friendly_team(),
        DynamicParameters::Navigator::default_avoid_dist.value());
    std::vector<RobotObstacle> enemy_obsts = generate_enemy_obstacles(
        world.enemy_team(),
        DynamicParameters::Navigator::default_avoid_dist.value());

    std::vector<RobotObstacle> all_obsts;
	all_obsts.reserve(friendly_obsts.size() + enemy_obsts.size());
	all_obsts.insert( all_obsts.end(), friendly_obsts.begin(), friendly_obsts.end() );
	all_obsts.insert( all_obsts.end(), enemy_obsts.begin(), enemy_obsts.end() );

    // Hand the different types of Intents here
    for (const auto &intent : assignedIntents)
    {
        if (intent->getIntentName() == MOVE_INTENT_NAME)
        {
            // TODO: Implement this
            // https://github.com/UBC-Thunderbots/Software/issues/23
            // Cast down to the MoveIntent class so we can access its members
            MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent);

			std::optional<Robot> r = world.friendly_team().getRobotById(move_intent.getRobotId());
			assert(r!=std::nullopt && "the world is messed up");
			Point currPos = r->position();
			Point destPos = move_intent.getDestination();
			double angleToDest = atan2(currPos.y(), currPos.x());
			if ((destPos-currPos).len()<stepSize){
				continue;
			}

			Point initPoint(stepSize * cos(angleToDest), stepSize * sin(angleToDest));
			std::optional<Point> stepPoint = findFreePoint(initPoint, angleToDest, all_obsts);

			std::unique_ptr<Primitive> move_prim;
			if (stepPoint!=std::nullopt){
				 move_prim = std::make_unique<MovePrimitive>(
				    move_intent.getRobotId(), *stepPoint,
				    move_intent.getFinalAngle(), move_intent.getFinalSpeed());
			}else{
				//no path forward
				move_prim = std::make_unique<MovePrimitive>(
				    move_intent.getRobotId(), currPos,
				    move_intent.getFinalAngle(), 0);
			}

			assigned_primitives.emplace_back(std::move(move_prim));
        }
        else
        {
            // TODO: Throw a proper exception here
            // https://github.com/UBC-Thunderbots/Software/issues/16
            std::cerr << "Error: Unrecognized Intent given to navigator" << std::endl;
            exit(1);
        }
    }

    return assigned_primitives;
}

std::optional<Point> RRTNav::findFreePoint(Point initPoint, double angleToDest, std::vector<RobotObstacle> obsts) const {
	//Use simple bug pathfinding algorithm and no collision detection
	//counter clockwise bias
	Point stepPoint = initPoint;

	//TODO: refactor into function
	bool goodPoint = true;
	bool cc = true;
	double angleShift = 0;
	Point nextStepPoint = stepPoint;
	do{
		goodPoint = true;
		stepPoint = nextStepPoint;
		//TODO: define all_obsts from friendly and enemy obsts
		for (const auto ro: obsts){
			if(ro.getViolationDistance(stepPoint)!=0.0){
				goodPoint = false;
				break;
			}
		}
		angleShift += angleStep;
		if (angleShift>180){
			break;
		}
		if (cc){
			nextStepPoint.set(stepSize * cos(angleToDest + angleShift), stepSize * sin(angleToDest + angleShift));
			cc = false;
		}else{
			nextStepPoint.set(stepSize * cos(angleToDest - angleShift), stepSize * sin(angleToDest - angleShift));
			cc = true;
		}
	} while (!goodPoint);

	if (angleShift>180){
		return std::nullopt;
	}else{
		return std::make_optional(stepPoint);
	}
}
