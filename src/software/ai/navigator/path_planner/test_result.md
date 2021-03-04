# Theta* Path Planner Testing

## Default Test Environment
```cpp
    Point initial_position = Point(-3, 1.5);
    Point destination      = Point(4.5, -3);
    setBallState(BallState(Point(0.25, 0.12), Vector(0, 0)));
    addFriendlyRobots(
        TestUtil::createStationaryRobotStatesWithId({Point(-3, 2.5), initial_position}));
    addEnemyRobots(TestUtil::createStationaryRobotStatesWithId({Point(1, 0)}));
    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::FORCE_START);
    auto tactic = std::make_shared<MoveTactic>(false);
    tactic->updateControlParams(destination, Angle::zero(), 0);
    setTactic(tactic);
    setRobotId(1);

    std::set<MotionConstraint> motion_constraints;
    motion_constraints.insert(MotionConstraint::ENEMY_ROBOTS_COLLISION);
    setMotionConstraints(motion_constraints);
```


## Tests
### 1. Destination set to center of enemy
```cpp
Point destination = Point(1, 0);
addEnemyRobots(TestUtil::createStationaryRobotStatesWithId({Point(1, 0)}));
```
#### Observations:
The tactic never finishes since the robot never reaches its destination, eventhough it is stationary after getting close to opponent

### 2. 
```cpp

```