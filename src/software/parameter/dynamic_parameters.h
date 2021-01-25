#pragma once
#include <stdio.h>

#include <boost/program_options.hpp>
#include <iostream>

#include "software/parameter/config.h"
class PassingConfig;
class NavigatorConfig;
class RobotNavigationObstacleFactoryConfig;
class GoalieTacticConfig;
class ShootGoalTacticConfig;
class DefenseShadowEnemyTacticConfig;
class CornerKickPlayConfig;
class ShootOrPassPlayConfig;
class AIConfig;
class AIControlConfig;
class EnemyCapabilityConfig;
class FullSystemMainCommandLineArgs;
class HandheldControllerConfig;
class SSLCommunicationConfig;
class NetworkConfig;
class RobotCapabilitiesConfig;
class RobotDiagnosticsMainCommandLineArgs;
class SensorFusionConfig;
class SimulatedTestMainCommandLineArgs;
class SimulatorConfig;
class StandaloneSimulatorConfig;
class StandaloneSimulatorMainCommandLineArgs;
class PassingConfig : public Config
{
   public:
    PassingConfig()
    {
        init();
    }
    void init()
    {
        StaticFieldPositionQualityXOffset_param =
            std::make_shared<Parameter<double>>("StaticFieldPositionQualityXOffset", 0.3);
        StaticFieldPositionQualityYOffset_param =
            std::make_shared<Parameter<double>>("StaticFieldPositionQualityYOffset", 0.3);
        StaticFieldPositionQualityFriendlyGoalDistanceWeight_param =
            std::make_shared<Parameter<double>>(
                "StaticFieldPositionQualityFriendlyGoalDistanceWeight", 0.3);
        EnemyProximityImportance_param =
            std::make_shared<Parameter<double>>("EnemyProximityImportance", 0.5);
        IdealMaxRotationToShootDegrees_param =
            std::make_shared<Parameter<double>>("IdealMaxRotationToShootDegrees", 60);
        MinPassSpeedMPerS_param =
            std::make_shared<Parameter<double>>("MinPassSpeedMPerS", 3.5);
        MaxPassSpeedMPerS_param =
            std::make_shared<Parameter<double>>("MaxPassSpeedMPerS", 5.5);
        MinTimeOffsetForPassSeconds_param =
            std::make_shared<Parameter<double>>("MinTimeOffsetForPassSeconds", 0.7);
        MaxTimeOffsetForPassSeconds_param =
            std::make_shared<Parameter<double>>("MaxTimeOffsetForPassSeconds", 4.0);
        EnemyReactionTime_param =
            std::make_shared<Parameter<double>>("EnemyReactionTime", 0.4);
        NumPassesToOptimize_param =
            std::make_shared<Parameter<int>>("NumPassesToOptimize", 15);
        NumPassesToKeepAfterPruning_param =
            std::make_shared<Parameter<int>>("NumPassesToKeepAfterPruning", 3);
        NumberOfGradientDescentStepsPerIter_param =
            std::make_shared<Parameter<int>>("NumberOfGradientDescentStepsPerIter", 10);
        PassEqualityMaxPositionDifferenceMeters_param =
            std::make_shared<Parameter<double>>("PassEqualityMaxPositionDifferenceMeters",
                                                0.05);
        PassEqualityMaxStartTimeDifferenceSeconds_param =
            std::make_shared<Parameter<double>>(
                "PassEqualityMaxStartTimeDifferenceSeconds", 0.1);
        PassEqualityMaxSpeedDifferenceMetersPerSecond_param =
            std::make_shared<Parameter<double>>(
                "PassEqualityMaxSpeedDifferenceMetersPerSecond", 0.3);
        mutable_internal_param_list = {
            StaticFieldPositionQualityXOffset_param,
            StaticFieldPositionQualityYOffset_param,
            StaticFieldPositionQualityFriendlyGoalDistanceWeight_param,
            EnemyProximityImportance_param,
            IdealMaxRotationToShootDegrees_param,
            MinPassSpeedMPerS_param,
            MaxPassSpeedMPerS_param,
            MinTimeOffsetForPassSeconds_param,
            MaxTimeOffsetForPassSeconds_param,
            EnemyReactionTime_param,
            NumPassesToOptimize_param,
            NumPassesToKeepAfterPruning_param,
            NumberOfGradientDescentStepsPerIter_param,
            PassEqualityMaxPositionDifferenceMeters_param,
            PassEqualityMaxStartTimeDifferenceSeconds_param,
            PassEqualityMaxSpeedDifferenceMetersPerSecond_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<double>>(
                StaticFieldPositionQualityXOffset_param),
            std::const_pointer_cast<const Parameter<double>>(
                StaticFieldPositionQualityYOffset_param),
            std::const_pointer_cast<const Parameter<double>>(
                StaticFieldPositionQualityFriendlyGoalDistanceWeight_param),
            std::const_pointer_cast<const Parameter<double>>(
                EnemyProximityImportance_param),
            std::const_pointer_cast<const Parameter<double>>(
                IdealMaxRotationToShootDegrees_param),
            std::const_pointer_cast<const Parameter<double>>(MinPassSpeedMPerS_param),
            std::const_pointer_cast<const Parameter<double>>(MaxPassSpeedMPerS_param),
            std::const_pointer_cast<const Parameter<double>>(
                MinTimeOffsetForPassSeconds_param),
            std::const_pointer_cast<const Parameter<double>>(
                MaxTimeOffsetForPassSeconds_param),
            std::const_pointer_cast<const Parameter<double>>(EnemyReactionTime_param),
            std::const_pointer_cast<const Parameter<int>>(NumPassesToOptimize_param),
            std::const_pointer_cast<const Parameter<int>>(
                NumPassesToKeepAfterPruning_param),
            std::const_pointer_cast<const Parameter<int>>(
                NumberOfGradientDescentStepsPerIter_param),
            std::const_pointer_cast<const Parameter<double>>(
                PassEqualityMaxPositionDifferenceMeters_param),
            std::const_pointer_cast<const Parameter<double>>(
                PassEqualityMaxStartTimeDifferenceSeconds_param),
            std::const_pointer_cast<const Parameter<double>>(
                PassEqualityMaxSpeedDifferenceMetersPerSecond_param)};
    }
    const std::shared_ptr<const Parameter<double>> StaticFieldPositionQualityXOffset()
        const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            StaticFieldPositionQualityXOffset_param);
    }

    const std::shared_ptr<Parameter<double>> mutableStaticFieldPositionQualityXOffset()
    {
        return StaticFieldPositionQualityXOffset_param;
    }

    const std::shared_ptr<const Parameter<double>> StaticFieldPositionQualityYOffset()
        const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            StaticFieldPositionQualityYOffset_param);
    }

    const std::shared_ptr<Parameter<double>> mutableStaticFieldPositionQualityYOffset()
    {
        return StaticFieldPositionQualityYOffset_param;
    }

    const std::shared_ptr<const Parameter<double>>
    StaticFieldPositionQualityFriendlyGoalDistanceWeight() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            StaticFieldPositionQualityFriendlyGoalDistanceWeight_param);
    }

    const std::shared_ptr<Parameter<double>>
    mutableStaticFieldPositionQualityFriendlyGoalDistanceWeight()
    {
        return StaticFieldPositionQualityFriendlyGoalDistanceWeight_param;
    }

    const std::shared_ptr<const Parameter<double>> EnemyProximityImportance() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            EnemyProximityImportance_param);
    }

    const std::shared_ptr<Parameter<double>> mutableEnemyProximityImportance()
    {
        return EnemyProximityImportance_param;
    }

    const std::shared_ptr<const Parameter<double>> IdealMaxRotationToShootDegrees() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            IdealMaxRotationToShootDegrees_param);
    }

    const std::shared_ptr<Parameter<double>> mutableIdealMaxRotationToShootDegrees()
    {
        return IdealMaxRotationToShootDegrees_param;
    }

    const std::shared_ptr<const Parameter<double>> MinPassSpeedMPerS() const
    {
        return std::const_pointer_cast<const Parameter<double>>(MinPassSpeedMPerS_param);
    }

    const std::shared_ptr<Parameter<double>> mutableMinPassSpeedMPerS()
    {
        return MinPassSpeedMPerS_param;
    }

    const std::shared_ptr<const Parameter<double>> MaxPassSpeedMPerS() const
    {
        return std::const_pointer_cast<const Parameter<double>>(MaxPassSpeedMPerS_param);
    }

    const std::shared_ptr<Parameter<double>> mutableMaxPassSpeedMPerS()
    {
        return MaxPassSpeedMPerS_param;
    }

    const std::shared_ptr<const Parameter<double>> MinTimeOffsetForPassSeconds() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            MinTimeOffsetForPassSeconds_param);
    }

    const std::shared_ptr<Parameter<double>> mutableMinTimeOffsetForPassSeconds()
    {
        return MinTimeOffsetForPassSeconds_param;
    }

    const std::shared_ptr<const Parameter<double>> MaxTimeOffsetForPassSeconds() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            MaxTimeOffsetForPassSeconds_param);
    }

    const std::shared_ptr<Parameter<double>> mutableMaxTimeOffsetForPassSeconds()
    {
        return MaxTimeOffsetForPassSeconds_param;
    }

    const std::shared_ptr<const Parameter<double>> EnemyReactionTime() const
    {
        return std::const_pointer_cast<const Parameter<double>>(EnemyReactionTime_param);
    }

    const std::shared_ptr<Parameter<double>> mutableEnemyReactionTime()
    {
        return EnemyReactionTime_param;
    }

    const std::shared_ptr<const Parameter<int>> NumPassesToOptimize() const
    {
        return std::const_pointer_cast<const Parameter<int>>(NumPassesToOptimize_param);
    }

    const std::shared_ptr<Parameter<int>> mutableNumPassesToOptimize()
    {
        return NumPassesToOptimize_param;
    }

    const std::shared_ptr<const Parameter<int>> NumPassesToKeepAfterPruning() const
    {
        return std::const_pointer_cast<const Parameter<int>>(
            NumPassesToKeepAfterPruning_param);
    }

    const std::shared_ptr<Parameter<int>> mutableNumPassesToKeepAfterPruning()
    {
        return NumPassesToKeepAfterPruning_param;
    }

    const std::shared_ptr<const Parameter<int>> NumberOfGradientDescentStepsPerIter()
        const
    {
        return std::const_pointer_cast<const Parameter<int>>(
            NumberOfGradientDescentStepsPerIter_param);
    }

    const std::shared_ptr<Parameter<int>> mutableNumberOfGradientDescentStepsPerIter()
    {
        return NumberOfGradientDescentStepsPerIter_param;
    }

    const std::shared_ptr<const Parameter<double>>
    PassEqualityMaxPositionDifferenceMeters() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            PassEqualityMaxPositionDifferenceMeters_param);
    }

    const std::shared_ptr<Parameter<double>>
    mutablePassEqualityMaxPositionDifferenceMeters()
    {
        return PassEqualityMaxPositionDifferenceMeters_param;
    }

    const std::shared_ptr<const Parameter<double>>
    PassEqualityMaxStartTimeDifferenceSeconds() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            PassEqualityMaxStartTimeDifferenceSeconds_param);
    }

    const std::shared_ptr<Parameter<double>>
    mutablePassEqualityMaxStartTimeDifferenceSeconds()
    {
        return PassEqualityMaxStartTimeDifferenceSeconds_param;
    }

    const std::shared_ptr<const Parameter<double>>
    PassEqualityMaxSpeedDifferenceMetersPerSecond() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            PassEqualityMaxSpeedDifferenceMetersPerSecond_param);
    }

    const std::shared_ptr<Parameter<double>>
    mutablePassEqualityMaxSpeedDifferenceMetersPerSecond()
    {
        return PassEqualityMaxSpeedDifferenceMetersPerSecond_param;
    }


    const std::string name() const
    {
        return "PassingConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help                                                   = false;
            double StaticFieldPositionQualityXOffset                    = 0.3;
            double StaticFieldPositionQualityYOffset                    = 0.3;
            double StaticFieldPositionQualityFriendlyGoalDistanceWeight = 0.3;
            double EnemyProximityImportance                             = 0.5;
            double IdealMaxRotationToShootDegrees                       = 60;
            double MinPassSpeedMPerS                                    = 3.5;
            double MaxPassSpeedMPerS                                    = 5.5;
            double MinTimeOffsetForPassSeconds                          = 0.7;
            double MaxTimeOffsetForPassSeconds                          = 4.0;
            double EnemyReactionTime                                    = 0.4;
            int NumPassesToOptimize                                     = 15;
            int NumPassesToKeepAfterPruning                             = 3;
            int NumberOfGradientDescentStepsPerIter                     = 10;
            double PassEqualityMaxPositionDifferenceMeters              = 0.05;
            double PassEqualityMaxStartTimeDifferenceSeconds            = 0.1;
            double PassEqualityMaxSpeedDifferenceMetersPerSecond        = 0.3;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()(
            "StaticFieldPositionQualityXOffset",
            boost::program_options::value<double>(
                &args.StaticFieldPositionQualityXOffset),
            "The offset from the sides of the field to place the rectangular sigmoid we use to determine what areas to pass to");
        desc.add_options()(
            "StaticFieldPositionQualityYOffset",
            boost::program_options::value<double>(
                &args.StaticFieldPositionQualityYOffset),
            "The offset from the sides of the field to place the rectangular sigmoid we use to determine what areas to pass to");
        desc.add_options()(
            "StaticFieldPositionQualityFriendlyGoalDistanceWeight",
            boost::program_options::value<double>(
                &args.StaticFieldPositionQualityFriendlyGoalDistanceWeight),
            "The weight that being close to the goal will have on the static position quality. Lower, more negative weights result in the distance to the goal having less of an effect.");
        desc.add_options()(
            "EnemyProximityImportance",
            boost::program_options::value<double>(&args.EnemyProximityImportance),
            "This controls how heavily we weight a robot being near the pass receiver when calculating enemy risk to a pass");
        desc.add_options()(
            "IdealMaxRotationToShootDegrees",
            boost::program_options::value<double>(&args.IdealMaxRotationToShootDegrees),
            "The maximum angle that we have to rotate after receiving a pass to shoot that we think would likely result in a goal. Note that we may try to take shots that require us to rotate more then this, it's more of a soft limit.");
        desc.add_options()("MinPassSpeedMPerS",
                           boost::program_options::value<double>(&args.MinPassSpeedMPerS),
                           "The minimum pass speed (in m/s)");
        desc.add_options()("MaxPassSpeedMPerS",
                           boost::program_options::value<double>(&args.MaxPassSpeedMPerS),
                           "The maximum pass speed (in m/s)");
        desc.add_options()(
            "MinTimeOffsetForPassSeconds",
            boost::program_options::value<double>(&args.MinTimeOffsetForPassSeconds),
            "Minimum time into the future at which the pass should occur. This is to ensure we have enough time to setup a robot to actually perform the pass. This is in seconds.");
        desc.add_options()(
            "MaxTimeOffsetForPassSeconds",
            boost::program_options::value<double>(&args.MaxTimeOffsetForPassSeconds),
            "Maximum time into the future at which the pass should occur. This gives the upper bound on the pass start time, relative to the current time. This is in seconds.");
        desc.add_options()(
            "EnemyReactionTime",
            boost::program_options::value<double>(&args.EnemyReactionTime),
            "How long we think the enemy will take to recognize we're passing and start moving to intercept");
        desc.add_options()("NumPassesToOptimize",
                           boost::program_options::value<int>(&args.NumPassesToOptimize),
                           "The number of passes to try to optimize at any given time");
        desc.add_options()(
            "NumPassesToKeepAfterPruning",
            boost::program_options::value<int>(&args.NumPassesToKeepAfterPruning),
            "The number of passes to keep after pruning");
        desc.add_options()(
            "NumberOfGradientDescentStepsPerIter",
            boost::program_options::value<int>(&args.NumberOfGradientDescentStepsPerIter),
            "The number of steps of gradient descent to perform in each iteration");
        desc.add_options()(
            "PassEqualityMaxPositionDifferenceMeters",
            boost::program_options::value<double>(
                &args.PassEqualityMaxPositionDifferenceMeters),
            "The maximum allowed difference between the receiver and passer point of two passes for which they are considered equal");
        desc.add_options()("PassEqualityMaxStartTimeDifferenceSeconds",
                           boost::program_options::value<double>(
                               &args.PassEqualityMaxStartTimeDifferenceSeconds),
                           "TODO: Add description as part of #149");
        desc.add_options()("PassEqualityMaxSpeedDifferenceMetersPerSecond",
                           boost::program_options::value<double>(
                               &args.PassEqualityMaxSpeedDifferenceMetersPerSecond),
                           "TODO: Add description as part of #149");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableStaticFieldPositionQualityXOffset()->setValue(
            args.StaticFieldPositionQualityXOffset);
        this->mutableStaticFieldPositionQualityYOffset()->setValue(
            args.StaticFieldPositionQualityYOffset);
        this->mutableStaticFieldPositionQualityFriendlyGoalDistanceWeight()->setValue(
            args.StaticFieldPositionQualityFriendlyGoalDistanceWeight);
        this->mutableEnemyProximityImportance()->setValue(args.EnemyProximityImportance);
        this->mutableIdealMaxRotationToShootDegrees()->setValue(
            args.IdealMaxRotationToShootDegrees);
        this->mutableMinPassSpeedMPerS()->setValue(args.MinPassSpeedMPerS);
        this->mutableMaxPassSpeedMPerS()->setValue(args.MaxPassSpeedMPerS);
        this->mutableMinTimeOffsetForPassSeconds()->setValue(
            args.MinTimeOffsetForPassSeconds);
        this->mutableMaxTimeOffsetForPassSeconds()->setValue(
            args.MaxTimeOffsetForPassSeconds);
        this->mutableEnemyReactionTime()->setValue(args.EnemyReactionTime);
        this->mutableNumPassesToOptimize()->setValue(args.NumPassesToOptimize);
        this->mutableNumPassesToKeepAfterPruning()->setValue(
            args.NumPassesToKeepAfterPruning);
        this->mutableNumberOfGradientDescentStepsPerIter()->setValue(
            args.NumberOfGradientDescentStepsPerIter);
        this->mutablePassEqualityMaxPositionDifferenceMeters()->setValue(
            args.PassEqualityMaxPositionDifferenceMeters);
        this->mutablePassEqualityMaxStartTimeDifferenceSeconds()->setValue(
            args.PassEqualityMaxStartTimeDifferenceSeconds);
        this->mutablePassEqualityMaxSpeedDifferenceMetersPerSecond()->setValue(
            args.PassEqualityMaxSpeedDifferenceMetersPerSecond);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<double>> StaticFieldPositionQualityXOffset_param;
    std::shared_ptr<Parameter<double>> StaticFieldPositionQualityYOffset_param;
    std::shared_ptr<Parameter<double>>
        StaticFieldPositionQualityFriendlyGoalDistanceWeight_param;
    std::shared_ptr<Parameter<double>> EnemyProximityImportance_param;
    std::shared_ptr<Parameter<double>> IdealMaxRotationToShootDegrees_param;
    std::shared_ptr<Parameter<double>> MinPassSpeedMPerS_param;
    std::shared_ptr<Parameter<double>> MaxPassSpeedMPerS_param;
    std::shared_ptr<Parameter<double>> MinTimeOffsetForPassSeconds_param;
    std::shared_ptr<Parameter<double>> MaxTimeOffsetForPassSeconds_param;
    std::shared_ptr<Parameter<double>> EnemyReactionTime_param;
    std::shared_ptr<Parameter<int>> NumPassesToOptimize_param;
    std::shared_ptr<Parameter<int>> NumPassesToKeepAfterPruning_param;
    std::shared_ptr<Parameter<int>> NumberOfGradientDescentStepsPerIter_param;
    std::shared_ptr<Parameter<double>> PassEqualityMaxPositionDifferenceMeters_param;
    std::shared_ptr<Parameter<double>> PassEqualityMaxStartTimeDifferenceSeconds_param;
    std::shared_ptr<Parameter<double>>
        PassEqualityMaxSpeedDifferenceMetersPerSecond_param;
};
class NavigatorConfig : public Config
{
   public:
    NavigatorConfig()
    {
        init();
    }
    void init()
    {
        TransitionSpeedFactor_param =
            std::make_shared<Parameter<double>>("TransitionSpeedFactor", 0.6);
        EnemyRobotProximityLimit_param =
            std::make_shared<Parameter<double>>("EnemyRobotProximityLimit", 2.0);
        mutable_internal_param_list   = {TransitionSpeedFactor_param,
                                       EnemyRobotProximityLimit_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<double>>(TransitionSpeedFactor_param),
            std::const_pointer_cast<const Parameter<double>>(
                EnemyRobotProximityLimit_param)};
    }
    const std::shared_ptr<const Parameter<double>> TransitionSpeedFactor() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            TransitionSpeedFactor_param);
    }

    const std::shared_ptr<Parameter<double>> mutableTransitionSpeedFactor()
    {
        return TransitionSpeedFactor_param;
    }

    const std::shared_ptr<const Parameter<double>> EnemyRobotProximityLimit() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            EnemyRobotProximityLimit_param);
    }

    const std::shared_ptr<Parameter<double>> mutableEnemyRobotProximityLimit()
    {
        return EnemyRobotProximityLimit_param;
    }


    const std::string name() const
    {
        return "NavigatorConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help                       = false;
            double TransitionSpeedFactor    = 0.6;
            double EnemyRobotProximityLimit = 2.0;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()(
            "TransitionSpeedFactor",
            boost::program_options::value<double>(&args.TransitionSpeedFactor),
            "Factor of max speed to use for transition speed calculations");
        desc.add_options()(
            "EnemyRobotProximityLimit",
            boost::program_options::value<double>(&args.EnemyRobotProximityLimit),
            "Distance to nearest robot when we stop slowing down to avoid collisions");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableTransitionSpeedFactor()->setValue(args.TransitionSpeedFactor);
        this->mutableEnemyRobotProximityLimit()->setValue(args.EnemyRobotProximityLimit);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<double>> TransitionSpeedFactor_param;
    std::shared_ptr<Parameter<double>> EnemyRobotProximityLimit_param;
};
class RobotNavigationObstacleFactoryConfig : public Config
{
   public:
    RobotNavigationObstacleFactoryConfig()
    {
        init();
    }
    void init()
    {
        SpeedScalingFactor_param =
            std::make_shared<Parameter<double>>("SpeedScalingFactor", 0.2);
        RobotObstacleInflationFactor_param =
            std::make_shared<Parameter<double>>("RobotObstacleInflationFactor", 2.0167);
        mutable_internal_param_list   = {SpeedScalingFactor_param,
                                       RobotObstacleInflationFactor_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<double>>(SpeedScalingFactor_param),
            std::const_pointer_cast<const Parameter<double>>(
                RobotObstacleInflationFactor_param)};
    }
    const std::shared_ptr<const Parameter<double>> SpeedScalingFactor() const
    {
        return std::const_pointer_cast<const Parameter<double>>(SpeedScalingFactor_param);
    }

    const std::shared_ptr<Parameter<double>> mutableSpeedScalingFactor()
    {
        return SpeedScalingFactor_param;
    }

    const std::shared_ptr<const Parameter<double>> RobotObstacleInflationFactor() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            RobotObstacleInflationFactor_param);
    }

    const std::shared_ptr<Parameter<double>> mutableRobotObstacleInflationFactor()
    {
        return RobotObstacleInflationFactor_param;
    }


    const std::string name() const
    {
        return "RobotNavigationObstacleFactoryConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help                           = false;
            double SpeedScalingFactor           = 0.2;
            double RobotObstacleInflationFactor = 2.0167;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()(
            "SpeedScalingFactor",
            boost::program_options::value<double>(&args.SpeedScalingFactor),
            "The factor to multiply object speed by to determine the length of the velocity obstacle in the objects direction of travel.");
        desc.add_options()(
            "RobotObstacleInflationFactor",
            boost::program_options::value<double>(&args.RobotObstacleInflationFactor),
            "Factor for robot obstacle size");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableSpeedScalingFactor()->setValue(args.SpeedScalingFactor);
        this->mutableRobotObstacleInflationFactor()->setValue(
            args.RobotObstacleInflationFactor);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<double>> SpeedScalingFactor_param;
    std::shared_ptr<Parameter<double>> RobotObstacleInflationFactor_param;
};
class GoalieTacticConfig : public Config
{
   public:
    GoalieTacticConfig()
    {
        init();
    }
    void init()
    {
        BlockConeBuffer_param =
            std::make_shared<Parameter<double>>("BlockConeBuffer", 0.0);
        GoalieFinalSpeed_param =
            std::make_shared<Parameter<double>>("GoalieFinalSpeed", 0.0);
        BallSpeedPanic_param = std::make_shared<Parameter<double>>("BallSpeedPanic", 0.2);
        BlockConeRadius_param =
            std::make_shared<Parameter<double>>("BlockConeRadius", 0.3);
        DefenseAreaDeflation_param =
            std::make_shared<Parameter<double>>("DefenseAreaDeflation", 0.2);
        mutable_internal_param_list   = {BlockConeBuffer_param, GoalieFinalSpeed_param,
                                       BallSpeedPanic_param, BlockConeRadius_param,
                                       DefenseAreaDeflation_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<double>>(BlockConeBuffer_param),
            std::const_pointer_cast<const Parameter<double>>(GoalieFinalSpeed_param),
            std::const_pointer_cast<const Parameter<double>>(BallSpeedPanic_param),
            std::const_pointer_cast<const Parameter<double>>(BlockConeRadius_param),
            std::const_pointer_cast<const Parameter<double>>(DefenseAreaDeflation_param)};
    }
    const std::shared_ptr<const Parameter<double>> BlockConeBuffer() const
    {
        return std::const_pointer_cast<const Parameter<double>>(BlockConeBuffer_param);
    }

    const std::shared_ptr<Parameter<double>> mutableBlockConeBuffer()
    {
        return BlockConeBuffer_param;
    }

    const std::shared_ptr<const Parameter<double>> GoalieFinalSpeed() const
    {
        return std::const_pointer_cast<const Parameter<double>>(GoalieFinalSpeed_param);
    }

    const std::shared_ptr<Parameter<double>> mutableGoalieFinalSpeed()
    {
        return GoalieFinalSpeed_param;
    }

    const std::shared_ptr<const Parameter<double>> BallSpeedPanic() const
    {
        return std::const_pointer_cast<const Parameter<double>>(BallSpeedPanic_param);
    }

    const std::shared_ptr<Parameter<double>> mutableBallSpeedPanic()
    {
        return BallSpeedPanic_param;
    }

    const std::shared_ptr<const Parameter<double>> BlockConeRadius() const
    {
        return std::const_pointer_cast<const Parameter<double>>(BlockConeRadius_param);
    }

    const std::shared_ptr<Parameter<double>> mutableBlockConeRadius()
    {
        return BlockConeRadius_param;
    }

    const std::shared_ptr<const Parameter<double>> DefenseAreaDeflation() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            DefenseAreaDeflation_param);
    }

    const std::shared_ptr<Parameter<double>> mutableDefenseAreaDeflation()
    {
        return DefenseAreaDeflation_param;
    }


    const std::string name() const
    {
        return "GoalieTacticConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help                   = false;
            double BlockConeBuffer      = 0.0;
            double GoalieFinalSpeed     = 0.0;
            double BallSpeedPanic       = 0.2;
            double BlockConeRadius      = 0.3;
            double DefenseAreaDeflation = 0.2;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()(
            "BlockConeBuffer",
            boost::program_options::value<double>(&args.BlockConeBuffer),
            "The block cone buffer is the extra distance to add on either side of the robot to allow to it be further back in the block cone");
        desc.add_options()(
            "GoalieFinalSpeed",
            boost::program_options::value<double>(&args.GoalieFinalSpeed),
            "Control the final speed of the goalie to be able to intercept shots better");
        desc.add_options()(
            "BallSpeedPanic", boost::program_options::value<double>(&args.BallSpeedPanic),
            "At what speed of the ball should the goalie panic and stop the ball");
        desc.add_options()(
            "BlockConeRadius",
            boost::program_options::value<double>(&args.BlockConeRadius),
            "The radius to wedge the robot into the cone, defaults to MAXROBOTRADIUS");
        desc.add_options()(
            "DefenseAreaDeflation",
            boost::program_options::value<double>(&args.DefenseAreaDeflation),
            "How much to deflate the defense area by, larger value means closer to the net");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableBlockConeBuffer()->setValue(args.BlockConeBuffer);
        this->mutableGoalieFinalSpeed()->setValue(args.GoalieFinalSpeed);
        this->mutableBallSpeedPanic()->setValue(args.BallSpeedPanic);
        this->mutableBlockConeRadius()->setValue(args.BlockConeRadius);
        this->mutableDefenseAreaDeflation()->setValue(args.DefenseAreaDeflation);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<double>> BlockConeBuffer_param;
    std::shared_ptr<Parameter<double>> GoalieFinalSpeed_param;
    std::shared_ptr<Parameter<double>> BallSpeedPanic_param;
    std::shared_ptr<Parameter<double>> BlockConeRadius_param;
    std::shared_ptr<Parameter<double>> DefenseAreaDeflation_param;
};
class ShootGoalTacticConfig : public Config
{
   public:
    ShootGoalTacticConfig()
    {
        init();
    }
    void init()
    {
        EnemyAboutToStealBallRectangleWidth_param = std::make_shared<Parameter<double>>(
            "EnemyAboutToStealBallRectangleWidth", 0.18);
        EnemyAboutToStealBallRectangleExtensionLength_param =
            std::make_shared<Parameter<double>>(
                "EnemyAboutToStealBallRectangleExtensionLength", 0.27);
        mutable_internal_param_list = {
            EnemyAboutToStealBallRectangleWidth_param,
            EnemyAboutToStealBallRectangleExtensionLength_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<double>>(
                EnemyAboutToStealBallRectangleWidth_param),
            std::const_pointer_cast<const Parameter<double>>(
                EnemyAboutToStealBallRectangleExtensionLength_param)};
    }
    const std::shared_ptr<const Parameter<double>> EnemyAboutToStealBallRectangleWidth()
        const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            EnemyAboutToStealBallRectangleWidth_param);
    }

    const std::shared_ptr<Parameter<double>> mutableEnemyAboutToStealBallRectangleWidth()
    {
        return EnemyAboutToStealBallRectangleWidth_param;
    }

    const std::shared_ptr<const Parameter<double>>
    EnemyAboutToStealBallRectangleExtensionLength() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            EnemyAboutToStealBallRectangleExtensionLength_param);
    }

    const std::shared_ptr<Parameter<double>>
    mutableEnemyAboutToStealBallRectangleExtensionLength()
    {
        return EnemyAboutToStealBallRectangleExtensionLength_param;
    }


    const std::string name() const
    {
        return "ShootGoalTacticConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help                                            = false;
            double EnemyAboutToStealBallRectangleWidth           = 0.18;
            double EnemyAboutToStealBallRectangleExtensionLength = 0.27;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()(
            "EnemyAboutToStealBallRectangleWidth",
            boost::program_options::value<double>(
                &args.EnemyAboutToStealBallRectangleWidth),
            "The total width ('sideways' of the robot) of the rectangle starting from the friendly robot inside which we think an enemy is about to");
        desc.add_options()(
            "EnemyAboutToStealBallRectangleExtensionLength",
            boost::program_options::value<double>(
                &args.EnemyAboutToStealBallRectangleExtensionLength),
            "The total length (in the direction the robot is facing) of the rectangle starting from the friendly robot inside which we think an enemy is about to steal the ball");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableEnemyAboutToStealBallRectangleWidth()->setValue(
            args.EnemyAboutToStealBallRectangleWidth);
        this->mutableEnemyAboutToStealBallRectangleExtensionLength()->setValue(
            args.EnemyAboutToStealBallRectangleExtensionLength);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<double>> EnemyAboutToStealBallRectangleWidth_param;
    std::shared_ptr<Parameter<double>>
        EnemyAboutToStealBallRectangleExtensionLength_param;
};
class DefenseShadowEnemyTacticConfig : public Config
{
   public:
    DefenseShadowEnemyTacticConfig()
    {
        init();
    }
    void init()
    {
        BallStealSpeed_param = std::make_shared<Parameter<double>>("BallStealSpeed", 0.3);
        mutable_internal_param_list   = {BallStealSpeed_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<double>>(BallStealSpeed_param)};
    }
    const std::shared_ptr<const Parameter<double>> BallStealSpeed() const
    {
        return std::const_pointer_cast<const Parameter<double>>(BallStealSpeed_param);
    }

    const std::shared_ptr<Parameter<double>> mutableBallStealSpeed()
    {
        return BallStealSpeed_param;
    }


    const std::string name() const
    {
        return "DefenseShadowEnemyTacticConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help             = false;
            double BallStealSpeed = 0.3;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()("BallStealSpeed",
                           boost::program_options::value<double>(&args.BallStealSpeed),
                           "Try to steal the passee's ball below this ball speed");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableBallStealSpeed()->setValue(args.BallStealSpeed);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<double>> BallStealSpeed_param;
};
class CornerKickPlayConfig : public Config
{
   public:
    CornerKickPlayConfig()
    {
        init();
    }
    void init()
    {
        MaxTimeCommitToPassSeconds_param =
            std::make_shared<Parameter<double>>("MaxTimeCommitToPassSeconds", 3.0);
        mutable_internal_param_list   = {MaxTimeCommitToPassSeconds_param};
        immutable_internal_param_list = {std::const_pointer_cast<const Parameter<double>>(
            MaxTimeCommitToPassSeconds_param)};
    }
    const std::shared_ptr<const Parameter<double>> MaxTimeCommitToPassSeconds() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            MaxTimeCommitToPassSeconds_param);
    }

    const std::shared_ptr<Parameter<double>> mutableMaxTimeCommitToPassSeconds()
    {
        return MaxTimeCommitToPassSeconds_param;
    }


    const std::string name() const
    {
        return "CornerKickPlayConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help                         = false;
            double MaxTimeCommitToPassSeconds = 3.0;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()(
            "MaxTimeCommitToPassSeconds",
            boost::program_options::value<double>(&args.MaxTimeCommitToPassSeconds),
            "The maximum time that we will wait before committing to a pass");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableMaxTimeCommitToPassSeconds()->setValue(
            args.MaxTimeCommitToPassSeconds);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<double>> MaxTimeCommitToPassSeconds_param;
};
class ShootOrPassPlayConfig : public Config
{
   public:
    ShootOrPassPlayConfig()
    {
        init();
    }
    void init()
    {
        AbsMinPassScore_param =
            std::make_shared<Parameter<double>>("AbsMinPassScore", 0.05);
        PassScoreRampDownDuration_param =
            std::make_shared<Parameter<double>>("PassScoreRampDownDuration", 2.0);
        MinOpenAngleForShotDeg_param =
            std::make_shared<Parameter<double>>("MinOpenAngleForShotDeg", 6);
        mutable_internal_param_list   = {AbsMinPassScore_param,
                                       PassScoreRampDownDuration_param,
                                       MinOpenAngleForShotDeg_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<double>>(AbsMinPassScore_param),
            std::const_pointer_cast<const Parameter<double>>(
                PassScoreRampDownDuration_param),
            std::const_pointer_cast<const Parameter<double>>(
                MinOpenAngleForShotDeg_param)};
    }
    const std::shared_ptr<const Parameter<double>> AbsMinPassScore() const
    {
        return std::const_pointer_cast<const Parameter<double>>(AbsMinPassScore_param);
    }

    const std::shared_ptr<Parameter<double>> mutableAbsMinPassScore()
    {
        return AbsMinPassScore_param;
    }

    const std::shared_ptr<const Parameter<double>> PassScoreRampDownDuration() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            PassScoreRampDownDuration_param);
    }

    const std::shared_ptr<Parameter<double>> mutablePassScoreRampDownDuration()
    {
        return PassScoreRampDownDuration_param;
    }

    const std::shared_ptr<const Parameter<double>> MinOpenAngleForShotDeg() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            MinOpenAngleForShotDeg_param);
    }

    const std::shared_ptr<Parameter<double>> mutableMinOpenAngleForShotDeg()
    {
        return MinOpenAngleForShotDeg_param;
    }


    const std::string name() const
    {
        return "ShootOrPassPlayConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help                        = false;
            double AbsMinPassScore           = 0.05;
            double PassScoreRampDownDuration = 2.0;
            double MinOpenAngleForShotDeg    = 6;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()("AbsMinPassScore",
                           boost::program_options::value<double>(&args.AbsMinPassScore),
                           "The absolute minimum pass quality we're willing to attempt");
        desc.add_options()(
            "PassScoreRampDownDuration",
            boost::program_options::value<double>(&args.PassScoreRampDownDuration),
            "When we're choosing a pass, we start by looking for a pass with a perfect score of 1, and then over time lower the score we're will to accept. This parameter how fast we ramp down to absMinPassScore. This is in seconds.");
        desc.add_options()(
            "MinOpenAngleForShotDeg",
            boost::program_options::value<double>(&args.MinOpenAngleForShotDeg),
            "The minimum open angle to the goal that we require before taking a shot");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableAbsMinPassScore()->setValue(args.AbsMinPassScore);
        this->mutablePassScoreRampDownDuration()->setValue(
            args.PassScoreRampDownDuration);
        this->mutableMinOpenAngleForShotDeg()->setValue(args.MinOpenAngleForShotDeg);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<double>> AbsMinPassScore_param;
    std::shared_ptr<Parameter<double>> PassScoreRampDownDuration_param;
    std::shared_ptr<Parameter<double>> MinOpenAngleForShotDeg_param;
};
class AIConfig : public Config
{
   public:
    AIConfig()
    {
        init();
    }
    void init()
    {
        PassingConfig_config = std::make_shared<PassingConfig>();

        NavigatorConfig_config = std::make_shared<NavigatorConfig>();

        RobotNavigationObstacleFactoryConfig_config =
            std::make_shared<RobotNavigationObstacleFactoryConfig>();

        GoalieTacticConfig_config = std::make_shared<GoalieTacticConfig>();

        ShootGoalTacticConfig_config = std::make_shared<ShootGoalTacticConfig>();

        DefenseShadowEnemyTacticConfig_config =
            std::make_shared<DefenseShadowEnemyTacticConfig>();

        CornerKickPlayConfig_config = std::make_shared<CornerKickPlayConfig>();

        ShootOrPassPlayConfig_config = std::make_shared<ShootOrPassPlayConfig>();

        mutable_internal_param_list   = {PassingConfig_config,
                                       NavigatorConfig_config,
                                       RobotNavigationObstacleFactoryConfig_config,
                                       GoalieTacticConfig_config,
                                       ShootGoalTacticConfig_config,
                                       DefenseShadowEnemyTacticConfig_config,
                                       CornerKickPlayConfig_config,
                                       ShootOrPassPlayConfig_config};
        immutable_internal_param_list = {
            std::const_pointer_cast<const PassingConfig>(PassingConfig_config),
            std::const_pointer_cast<const NavigatorConfig>(NavigatorConfig_config),
            std::const_pointer_cast<const RobotNavigationObstacleFactoryConfig>(
                RobotNavigationObstacleFactoryConfig_config),
            std::const_pointer_cast<const GoalieTacticConfig>(GoalieTacticConfig_config),
            std::const_pointer_cast<const ShootGoalTacticConfig>(
                ShootGoalTacticConfig_config),
            std::const_pointer_cast<const DefenseShadowEnemyTacticConfig>(
                DefenseShadowEnemyTacticConfig_config),
            std::const_pointer_cast<const CornerKickPlayConfig>(
                CornerKickPlayConfig_config),
            std::const_pointer_cast<const ShootOrPassPlayConfig>(
                ShootOrPassPlayConfig_config)};
    }
    const std::shared_ptr<const PassingConfig> getPassingConfig() const
    {
        return std::const_pointer_cast<const PassingConfig>(PassingConfig_config);
    }

    const std::shared_ptr<PassingConfig> getMutablePassingConfig()
    {
        return PassingConfig_config;
    }

    const std::shared_ptr<const NavigatorConfig> getNavigatorConfig() const
    {
        return std::const_pointer_cast<const NavigatorConfig>(NavigatorConfig_config);
    }

    const std::shared_ptr<NavigatorConfig> getMutableNavigatorConfig()
    {
        return NavigatorConfig_config;
    }

    const std::shared_ptr<const RobotNavigationObstacleFactoryConfig>
    getRobotNavigationObstacleFactoryConfig() const
    {
        return std::const_pointer_cast<const RobotNavigationObstacleFactoryConfig>(
            RobotNavigationObstacleFactoryConfig_config);
    }

    const std::shared_ptr<RobotNavigationObstacleFactoryConfig>
    getMutableRobotNavigationObstacleFactoryConfig()
    {
        return RobotNavigationObstacleFactoryConfig_config;
    }

    const std::shared_ptr<const GoalieTacticConfig> getGoalieTacticConfig() const
    {
        return std::const_pointer_cast<const GoalieTacticConfig>(
            GoalieTacticConfig_config);
    }

    const std::shared_ptr<GoalieTacticConfig> getMutableGoalieTacticConfig()
    {
        return GoalieTacticConfig_config;
    }

    const std::shared_ptr<const ShootGoalTacticConfig> getShootGoalTacticConfig() const
    {
        return std::const_pointer_cast<const ShootGoalTacticConfig>(
            ShootGoalTacticConfig_config);
    }

    const std::shared_ptr<ShootGoalTacticConfig> getMutableShootGoalTacticConfig()
    {
        return ShootGoalTacticConfig_config;
    }

    const std::shared_ptr<const DefenseShadowEnemyTacticConfig>
    getDefenseShadowEnemyTacticConfig() const
    {
        return std::const_pointer_cast<const DefenseShadowEnemyTacticConfig>(
            DefenseShadowEnemyTacticConfig_config);
    }

    const std::shared_ptr<DefenseShadowEnemyTacticConfig>
    getMutableDefenseShadowEnemyTacticConfig()
    {
        return DefenseShadowEnemyTacticConfig_config;
    }

    const std::shared_ptr<const CornerKickPlayConfig> getCornerKickPlayConfig() const
    {
        return std::const_pointer_cast<const CornerKickPlayConfig>(
            CornerKickPlayConfig_config);
    }

    const std::shared_ptr<CornerKickPlayConfig> getMutableCornerKickPlayConfig()
    {
        return CornerKickPlayConfig_config;
    }

    const std::shared_ptr<const ShootOrPassPlayConfig> getShootOrPassPlayConfig() const
    {
        return std::const_pointer_cast<const ShootOrPassPlayConfig>(
            ShootOrPassPlayConfig_config);
    }

    const std::shared_ptr<ShootOrPassPlayConfig> getMutableShootOrPassPlayConfig()
    {
        return ShootOrPassPlayConfig_config;
    }


    const std::string name() const
    {
        return "AIConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help = false;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");



        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);



        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<PassingConfig> PassingConfig_config;
    std::shared_ptr<NavigatorConfig> NavigatorConfig_config;
    std::shared_ptr<RobotNavigationObstacleFactoryConfig>
        RobotNavigationObstacleFactoryConfig_config;
    std::shared_ptr<GoalieTacticConfig> GoalieTacticConfig_config;
    std::shared_ptr<ShootGoalTacticConfig> ShootGoalTacticConfig_config;
    std::shared_ptr<DefenseShadowEnemyTacticConfig> DefenseShadowEnemyTacticConfig_config;
    std::shared_ptr<CornerKickPlayConfig> CornerKickPlayConfig_config;
    std::shared_ptr<ShootOrPassPlayConfig> ShootOrPassPlayConfig_config;
};
class AIControlConfig : public Config
{
   public:
    AIControlConfig()
    {
        init();
    }
    void init()
    {
        RunAI_param          = std::make_shared<Parameter<bool>>("RunAI", true);
        OverrideAIPlay_param = std::make_shared<Parameter<bool>>("OverrideAIPlay", true);
        CurrentAIPlay_param =
            std::make_shared<Parameter<std::string>>("CurrentAIPlay", "Halt Play");
        OverrideRefereeCommand_param =
            std::make_shared<Parameter<bool>>("OverrideRefereeCommand", true);
        CurrentRefereeCommand_param =
            std::make_shared<Parameter<std::string>>("CurrentRefereeCommand", "HALT");
        PreviousRefereeCommand_param =
            std::make_shared<Parameter<std::string>>("PreviousRefereeCommand", "HALT");
        mutable_internal_param_list   = {RunAI_param,
                                       OverrideAIPlay_param,
                                       CurrentAIPlay_param,
                                       OverrideRefereeCommand_param,
                                       CurrentRefereeCommand_param,
                                       PreviousRefereeCommand_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<bool>>(RunAI_param),
            std::const_pointer_cast<const Parameter<bool>>(OverrideAIPlay_param),
            std::const_pointer_cast<const Parameter<std::string>>(CurrentAIPlay_param),
            std::const_pointer_cast<const Parameter<bool>>(OverrideRefereeCommand_param),
            std::const_pointer_cast<const Parameter<std::string>>(
                CurrentRefereeCommand_param),
            std::const_pointer_cast<const Parameter<std::string>>(
                PreviousRefereeCommand_param)};
    }
    const std::shared_ptr<const Parameter<bool>> RunAI() const
    {
        return std::const_pointer_cast<const Parameter<bool>>(RunAI_param);
    }

    const std::shared_ptr<Parameter<bool>> mutableRunAI()
    {
        return RunAI_param;
    }

    const std::shared_ptr<const Parameter<bool>> OverrideAIPlay() const
    {
        return std::const_pointer_cast<const Parameter<bool>>(OverrideAIPlay_param);
    }

    const std::shared_ptr<Parameter<bool>> mutableOverrideAIPlay()
    {
        return OverrideAIPlay_param;
    }

    const std::shared_ptr<const Parameter<std::string>> CurrentAIPlay() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(CurrentAIPlay_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutableCurrentAIPlay()
    {
        return CurrentAIPlay_param;
    }

    const std::shared_ptr<const Parameter<bool>> OverrideRefereeCommand() const
    {
        return std::const_pointer_cast<const Parameter<bool>>(
            OverrideRefereeCommand_param);
    }

    const std::shared_ptr<Parameter<bool>> mutableOverrideRefereeCommand()
    {
        return OverrideRefereeCommand_param;
    }

    const std::shared_ptr<const Parameter<std::string>> CurrentRefereeCommand() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(
            CurrentRefereeCommand_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutableCurrentRefereeCommand()
    {
        return CurrentRefereeCommand_param;
    }

    const std::shared_ptr<const Parameter<std::string>> PreviousRefereeCommand() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(
            PreviousRefereeCommand_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutablePreviousRefereeCommand()
    {
        return PreviousRefereeCommand_param;
    }


    const std::string name() const
    {
        return "AIControlConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help                          = false;
            bool RunAI                         = true;
            bool OverrideAIPlay                = true;
            std::string CurrentAIPlay          = "Halt Play";
            bool OverrideRefereeCommand        = true;
            std::string CurrentRefereeCommand  = "HALT";
            std::string PreviousRefereeCommand = "HALT";
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()(
            "RunAI", boost::program_options::bool_switch(&args.RunAI),
            "Selecting will allow HL and Navigator to run, unselecting will stop new decisions from being made, but the robots will finish the last commands they were given. DO NOT USE in place of the e-stop.");
        desc.add_options()(
            "OverrideAIPlay", boost::program_options::bool_switch(&args.OverrideAIPlay),
            "Overrides the current play with the play specified by currentPlay parameter");
        desc.add_options()(
            "CurrentAIPlay",
            boost::program_options::value<std::string>(&args.CurrentAIPlay),
            "Specifies the ai play that should be in use");
        desc.add_options()(
            "OverrideRefereeCommand",
            boost::program_options::bool_switch(&args.OverrideRefereeCommand),
            "Overrides the current referee command with the play  specified by CurrentRefereeCommand and PreviousRefereeCommand parameter");
        desc.add_options()(
            "CurrentRefereeCommand",
            boost::program_options::value<std::string>(&args.CurrentRefereeCommand),
            "Specifies the referee command that should be in use");
        desc.add_options()(
            "PreviousRefereeCommand",
            boost::program_options::value<std::string>(&args.PreviousRefereeCommand),
            "Specifies the previous referee command to correctly set up  the referee command state machine when overriding the referee command. For example, if `PREPARE_KICKOFF_US` followed by `NORMAL_START`,  then `isOurRestart()` will be `true`, while  `PREPARE_KICKOFF_THEM` followed by `NORMAL_START` will set  `isOurRestart()` to `false`");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableRunAI()->setValue(args.RunAI);
        this->mutableOverrideAIPlay()->setValue(args.OverrideAIPlay);
        this->mutableCurrentAIPlay()->setValue(args.CurrentAIPlay);
        this->mutableOverrideRefereeCommand()->setValue(args.OverrideRefereeCommand);
        this->mutableCurrentRefereeCommand()->setValue(args.CurrentRefereeCommand);
        this->mutablePreviousRefereeCommand()->setValue(args.PreviousRefereeCommand);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<bool>> RunAI_param;
    std::shared_ptr<Parameter<bool>> OverrideAIPlay_param;
    std::shared_ptr<Parameter<std::string>> CurrentAIPlay_param;
    std::shared_ptr<Parameter<bool>> OverrideRefereeCommand_param;
    std::shared_ptr<Parameter<std::string>> CurrentRefereeCommand_param;
    std::shared_ptr<Parameter<std::string>> PreviousRefereeCommand_param;
};
class EnemyCapabilityConfig : public Config
{
   public:
    EnemyCapabilityConfig()
    {
        init();
    }
    void init()
    {
        EnemyTeamCanPass_param =
            std::make_shared<Parameter<bool>>("EnemyTeamCanPass", true);
        mutable_internal_param_list   = {EnemyTeamCanPass_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<bool>>(EnemyTeamCanPass_param)};
    }
    const std::shared_ptr<const Parameter<bool>> EnemyTeamCanPass() const
    {
        return std::const_pointer_cast<const Parameter<bool>>(EnemyTeamCanPass_param);
    }

    const std::shared_ptr<Parameter<bool>> mutableEnemyTeamCanPass()
    {
        return EnemyTeamCanPass_param;
    }


    const std::string name() const
    {
        return "EnemyCapabilityConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help             = false;
            bool EnemyTeamCanPass = true;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()(
            "EnemyTeamCanPass",
            boost::program_options::bool_switch(&args.EnemyTeamCanPass),
            "This value should be set based on whether or not the team we are playing against can pass the ball. This will affect how we defend against the team.");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableEnemyTeamCanPass()->setValue(args.EnemyTeamCanPass);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<bool>> EnemyTeamCanPass_param;
};
class FullSystemMainCommandLineArgs : public Config
{
   public:
    FullSystemMainCommandLineArgs()
    {
        init();
    }
    void init()
    {
        backend_param   = std::make_shared<Parameter<std::string>>("backend", "");
        interface_param = std::make_shared<Parameter<std::string>>("interface", "");
        headless_param  = std::make_shared<Parameter<bool>>("headless", false);
        proto_log_output_dir_param =
            std::make_shared<Parameter<std::string>>("proto_log_output_dir", "");
        replay_input_dir_param =
            std::make_shared<Parameter<std::string>>("replay_input_dir", "");
        mutable_internal_param_list   = {backend_param, interface_param, headless_param,
                                       proto_log_output_dir_param,
                                       replay_input_dir_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<std::string>>(backend_param),
            std::const_pointer_cast<const Parameter<std::string>>(interface_param),
            std::const_pointer_cast<const Parameter<bool>>(headless_param),
            std::const_pointer_cast<const Parameter<std::string>>(
                proto_log_output_dir_param),
            std::const_pointer_cast<const Parameter<std::string>>(
                replay_input_dir_param)};
    }
    const std::shared_ptr<const Parameter<std::string>> backend() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(backend_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutablebackend()
    {
        return backend_param;
    }

    const std::shared_ptr<const Parameter<std::string>> interface() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(interface_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutableinterface()
    {
        return interface_param;
    }

    const std::shared_ptr<const Parameter<bool>> headless() const
    {
        return std::const_pointer_cast<const Parameter<bool>>(headless_param);
    }

    const std::shared_ptr<Parameter<bool>> mutableheadless()
    {
        return headless_param;
    }

    const std::shared_ptr<const Parameter<std::string>> proto_log_output_dir() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(
            proto_log_output_dir_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutableproto_log_output_dir()
    {
        return proto_log_output_dir_param;
    }

    const std::shared_ptr<const Parameter<std::string>> replay_input_dir() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(
            replay_input_dir_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutablereplay_input_dir()
    {
        return replay_input_dir_param;
    }


    const std::string name() const
    {
        return "FullSystemMainCommandLineArgs";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help                        = false;
            std::string backend              = "";
            std::string interface            = "";
            bool headless                    = false;
            std::string proto_log_output_dir = "";
            std::string replay_input_dir     = "";
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()(
            "backend", boost::program_options::value<std::string>(&args.backend),
            "The name of the backend you would like to use. See /software/backend for options ex. class WifiBackend this argument would be WifiBackend");
        desc.add_options()(
            "interface", boost::program_options::value<std::string>(&args.interface),
            "The interface to send and receive packets over (can be found through ifconfig)");
        desc.add_options()("headless",
                           boost::program_options::bool_switch(&args.headless),
                           "Run without the FullSystemGui");
        desc.add_options()(
            "proto_log_output_dir",
            boost::program_options::value<std::string>(&args.proto_log_output_dir),
            "The directory to output logged Protobuf data to. Protobufs will not be logged if this argument is not used.");
        desc.add_options()(
            "replay_input_dir",
            boost::program_options::value<std::string>(&args.replay_input_dir),
            "The directory to replay logged data from, if the 'replay' backend is selected. This must be the `SensorMsg` folder outputted by `proto_log_output_dir`.");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutablebackend()->setValue(args.backend);
        this->mutableinterface()->setValue(args.interface);
        this->mutableheadless()->setValue(args.headless);
        this->mutableproto_log_output_dir()->setValue(args.proto_log_output_dir);
        this->mutablereplay_input_dir()->setValue(args.replay_input_dir);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<std::string>> backend_param;
    std::shared_ptr<Parameter<std::string>> interface_param;
    std::shared_ptr<Parameter<bool>> headless_param;
    std::shared_ptr<Parameter<std::string>> proto_log_output_dir_param;
    std::shared_ptr<Parameter<std::string>> replay_input_dir_param;
};
class HandheldControllerConfig : public Config
{
   public:
    HandheldControllerConfig()
    {
        init();
    }
    void init()
    {
        RobotId_param = std::make_shared<Parameter<int>>("RobotId", 0);
        KickSpeedMetersPerSecond_param =
            std::make_shared<Parameter<double>>("KickSpeedMetersPerSecond", 4.0);
        ChipDistanceMeters_param =
            std::make_shared<Parameter<double>>("ChipDistanceMeters", 1.0);
        DribblerRpm_param    = std::make_shared<Parameter<int>>("DribblerRpm", 1000);
        MaxLinearSpeed_param = std::make_shared<Parameter<double>>("MaxLinearSpeed", 1.0);
        MaxAngularSpeed_param =
            std::make_shared<Parameter<double>>("MaxAngularSpeed", 6.0);
        DevicePath_param =
            std::make_shared<Parameter<std::string>>("DevicePath", "/dev/input/js0");
        mutable_internal_param_list = {
            RobotId_param,     KickSpeedMetersPerSecond_param, ChipDistanceMeters_param,
            DribblerRpm_param, MaxLinearSpeed_param,           MaxAngularSpeed_param,
            DevicePath_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<int>>(RobotId_param),
            std::const_pointer_cast<const Parameter<double>>(
                KickSpeedMetersPerSecond_param),
            std::const_pointer_cast<const Parameter<double>>(ChipDistanceMeters_param),
            std::const_pointer_cast<const Parameter<int>>(DribblerRpm_param),
            std::const_pointer_cast<const Parameter<double>>(MaxLinearSpeed_param),
            std::const_pointer_cast<const Parameter<double>>(MaxAngularSpeed_param),
            std::const_pointer_cast<const Parameter<std::string>>(DevicePath_param)};
    }
    const std::shared_ptr<const Parameter<int>> RobotId() const
    {
        return std::const_pointer_cast<const Parameter<int>>(RobotId_param);
    }

    const std::shared_ptr<Parameter<int>> mutableRobotId()
    {
        return RobotId_param;
    }

    const std::shared_ptr<const Parameter<double>> KickSpeedMetersPerSecond() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            KickSpeedMetersPerSecond_param);
    }

    const std::shared_ptr<Parameter<double>> mutableKickSpeedMetersPerSecond()
    {
        return KickSpeedMetersPerSecond_param;
    }

    const std::shared_ptr<const Parameter<double>> ChipDistanceMeters() const
    {
        return std::const_pointer_cast<const Parameter<double>>(ChipDistanceMeters_param);
    }

    const std::shared_ptr<Parameter<double>> mutableChipDistanceMeters()
    {
        return ChipDistanceMeters_param;
    }

    const std::shared_ptr<const Parameter<int>> DribblerRpm() const
    {
        return std::const_pointer_cast<const Parameter<int>>(DribblerRpm_param);
    }

    const std::shared_ptr<Parameter<int>> mutableDribblerRpm()
    {
        return DribblerRpm_param;
    }

    const std::shared_ptr<const Parameter<double>> MaxLinearSpeed() const
    {
        return std::const_pointer_cast<const Parameter<double>>(MaxLinearSpeed_param);
    }

    const std::shared_ptr<Parameter<double>> mutableMaxLinearSpeed()
    {
        return MaxLinearSpeed_param;
    }

    const std::shared_ptr<const Parameter<double>> MaxAngularSpeed() const
    {
        return std::const_pointer_cast<const Parameter<double>>(MaxAngularSpeed_param);
    }

    const std::shared_ptr<Parameter<double>> mutableMaxAngularSpeed()
    {
        return MaxAngularSpeed_param;
    }

    const std::shared_ptr<const Parameter<std::string>> DevicePath() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(DevicePath_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutableDevicePath()
    {
        return DevicePath_param;
    }


    const std::string name() const
    {
        return "HandheldControllerConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help                       = false;
            int RobotId                     = 0;
            double KickSpeedMetersPerSecond = 4.0;
            double ChipDistanceMeters       = 1.0;
            int DribblerRpm                 = 1000;
            double MaxLinearSpeed           = 1.0;
            double MaxAngularSpeed          = 6.0;
            std::string DevicePath          = "/dev/input/js0";
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()("RobotId", boost::program_options::value<int>(&args.RobotId),
                           "The ID of the robot being controller by the game controller");
        desc.add_options()(
            "KickSpeedMetersPerSecond",
            boost::program_options::value<double>(&args.KickSpeedMetersPerSecond),
            "The robot's kick speed in meters per second");
        desc.add_options()(
            "ChipDistanceMeters",
            boost::program_options::value<double>(&args.ChipDistanceMeters),
            "How far in meters the robot will chip");
        desc.add_options()("DribblerRpm",
                           boost::program_options::value<int>(&args.DribblerRpm),
                           "The RPM of the robot's dribbler when turned on");
        desc.add_options()("MaxLinearSpeed",
                           boost::program_options::value<double>(&args.MaxLinearSpeed),
                           "The max allowed linear speed of the robot in m/s");
        desc.add_options()("MaxAngularSpeed",
                           boost::program_options::value<double>(&args.MaxAngularSpeed),
                           "The max allowed angular speed of the robot, in rad/s");
        desc.add_options()("DevicePath",
                           boost::program_options::value<std::string>(&args.DevicePath),
                           "The filepath to the controller on the system");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableRobotId()->setValue(args.RobotId);
        this->mutableKickSpeedMetersPerSecond()->setValue(args.KickSpeedMetersPerSecond);
        this->mutableChipDistanceMeters()->setValue(args.ChipDistanceMeters);
        this->mutableDribblerRpm()->setValue(args.DribblerRpm);
        this->mutableMaxLinearSpeed()->setValue(args.MaxLinearSpeed);
        this->mutableMaxAngularSpeed()->setValue(args.MaxAngularSpeed);
        this->mutableDevicePath()->setValue(args.DevicePath);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<int>> RobotId_param;
    std::shared_ptr<Parameter<double>> KickSpeedMetersPerSecond_param;
    std::shared_ptr<Parameter<double>> ChipDistanceMeters_param;
    std::shared_ptr<Parameter<int>> DribblerRpm_param;
    std::shared_ptr<Parameter<double>> MaxLinearSpeed_param;
    std::shared_ptr<Parameter<double>> MaxAngularSpeed_param;
    std::shared_ptr<Parameter<std::string>> DevicePath_param;
};
class SSLCommunicationConfig : public Config
{
   public:
    SSLCommunicationConfig()
    {
        init();
    }
    void init()
    {
        VisionIPv4Address_param =
            std::make_shared<Parameter<std::string>>("VisionIPv4Address", "224.5.23.2");
        VisionPort_param = std::make_shared<Parameter<int>>("VisionPort", 10020);
        GameControllerIPv4Address_param = std::make_shared<Parameter<std::string>>(
            "GameControllerIPv4Address", "224.5.23.1");
        GameControllerPort_param =
            std::make_shared<Parameter<int>>("GameControllerPort", 10003);
        mutable_internal_param_list   = {VisionIPv4Address_param, VisionPort_param,
                                       GameControllerIPv4Address_param,
                                       GameControllerPort_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<std::string>>(
                VisionIPv4Address_param),
            std::const_pointer_cast<const Parameter<int>>(VisionPort_param),
            std::const_pointer_cast<const Parameter<std::string>>(
                GameControllerIPv4Address_param),
            std::const_pointer_cast<const Parameter<int>>(GameControllerPort_param)};
    }
    const std::shared_ptr<const Parameter<std::string>> VisionIPv4Address() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(
            VisionIPv4Address_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutableVisionIPv4Address()
    {
        return VisionIPv4Address_param;
    }

    const std::shared_ptr<const Parameter<int>> VisionPort() const
    {
        return std::const_pointer_cast<const Parameter<int>>(VisionPort_param);
    }

    const std::shared_ptr<Parameter<int>> mutableVisionPort()
    {
        return VisionPort_param;
    }

    const std::shared_ptr<const Parameter<std::string>> GameControllerIPv4Address() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(
            GameControllerIPv4Address_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutableGameControllerIPv4Address()
    {
        return GameControllerIPv4Address_param;
    }

    const std::shared_ptr<const Parameter<int>> GameControllerPort() const
    {
        return std::const_pointer_cast<const Parameter<int>>(GameControllerPort_param);
    }

    const std::shared_ptr<Parameter<int>> mutableGameControllerPort()
    {
        return GameControllerPort_param;
    }


    const std::string name() const
    {
        return "SSLCommunicationConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help                             = false;
            std::string VisionIPv4Address         = "224.5.23.2";
            int VisionPort                        = 10020;
            std::string GameControllerIPv4Address = "224.5.23.1";
            int GameControllerPort                = 10003;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()(
            "VisionIPv4Address",
            boost::program_options::value<std::string>(&args.VisionIPv4Address),
            "The IPv4 address to listen on for vision data");
        desc.add_options()("VisionPort",
                           boost::program_options::value<int>(&args.VisionPort),
                           "The port to listen on for vision data");
        desc.add_options()(
            "GameControllerIPv4Address",
            boost::program_options::value<std::string>(&args.GameControllerIPv4Address),
            "The IPv4 address to listen on for gamecontroller data");
        desc.add_options()("GameControllerPort",
                           boost::program_options::value<int>(&args.GameControllerPort),
                           "The port to listen on for gamecontroller data");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableVisionIPv4Address()->setValue(args.VisionIPv4Address);
        this->mutableVisionPort()->setValue(args.VisionPort);
        this->mutableGameControllerIPv4Address()->setValue(
            args.GameControllerIPv4Address);
        this->mutableGameControllerPort()->setValue(args.GameControllerPort);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<std::string>> VisionIPv4Address_param;
    std::shared_ptr<Parameter<int>> VisionPort_param;
    std::shared_ptr<Parameter<std::string>> GameControllerIPv4Address_param;
    std::shared_ptr<Parameter<int>> GameControllerPort_param;
};
class NetworkConfig : public Config
{
   public:
    NetworkConfig()
    {
        init();
    }
    void init()
    {
        SSLCommunicationConfig_config = std::make_shared<SSLCommunicationConfig>();
        Channel_param                 = std::make_shared<Parameter<int>>("Channel", 0);
        NetworkInterface_param =
            std::make_shared<Parameter<std::string>>("NetworkInterface", "eth0");
        mutable_internal_param_list   = {Channel_param, NetworkInterface_param,
                                       SSLCommunicationConfig_config};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<int>>(Channel_param),
            std::const_pointer_cast<const Parameter<std::string>>(NetworkInterface_param),
            std::const_pointer_cast<const SSLCommunicationConfig>(
                SSLCommunicationConfig_config)};
    }
    const std::shared_ptr<const Parameter<int>> Channel() const
    {
        return std::const_pointer_cast<const Parameter<int>>(Channel_param);
    }

    const std::shared_ptr<Parameter<int>> mutableChannel()
    {
        return Channel_param;
    }

    const std::shared_ptr<const Parameter<std::string>> NetworkInterface() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(
            NetworkInterface_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutableNetworkInterface()
    {
        return NetworkInterface_param;
    }
    const std::shared_ptr<const SSLCommunicationConfig> getSSLCommunicationConfig() const
    {
        return std::const_pointer_cast<const SSLCommunicationConfig>(
            SSLCommunicationConfig_config);
    }

    const std::shared_ptr<SSLCommunicationConfig> getMutableSSLCommunicationConfig()
    {
        return SSLCommunicationConfig_config;
    }


    const std::string name() const
    {
        return "NetworkConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help                    = false;
            int Channel                  = 0;
            std::string NetworkInterface = "eth0";
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()(
            "Channel", boost::program_options::value<int>(&args.Channel),
            "The multicast channel to join to communicate with the robots");
        desc.add_options()(
            "NetworkInterface",
            boost::program_options::value<std::string>(&args.NetworkInterface),
            "The network interface that is connected to the thunderbots router. Can be found using ifconfig on ubuntu.");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableChannel()->setValue(args.Channel);
        this->mutableNetworkInterface()->setValue(args.NetworkInterface);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<int>> Channel_param;
    std::shared_ptr<Parameter<std::string>> NetworkInterface_param;
    std::shared_ptr<SSLCommunicationConfig> SSLCommunicationConfig_config;
};
class RobotCapabilitiesConfig : public Config
{
   public:
    RobotCapabilitiesConfig()
    {
        init();
    }
    void init()
    {
        BrokenDribblers_param =
            std::make_shared<Parameter<std::string>>("BrokenDribblers", "");
        BrokenChippers_param =
            std::make_shared<Parameter<std::string>>("BrokenChippers", "");
        BrokenKickers_param =
            std::make_shared<Parameter<std::string>>("BrokenKickers", "");
        mutable_internal_param_list   = {BrokenDribblers_param, BrokenChippers_param,
                                       BrokenKickers_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<std::string>>(BrokenDribblers_param),
            std::const_pointer_cast<const Parameter<std::string>>(BrokenChippers_param),
            std::const_pointer_cast<const Parameter<std::string>>(BrokenKickers_param)};
    }
    const std::shared_ptr<const Parameter<std::string>> BrokenDribblers() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(
            BrokenDribblers_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutableBrokenDribblers()
    {
        return BrokenDribblers_param;
    }

    const std::shared_ptr<const Parameter<std::string>> BrokenChippers() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(
            BrokenChippers_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutableBrokenChippers()
    {
        return BrokenChippers_param;
    }

    const std::shared_ptr<const Parameter<std::string>> BrokenKickers() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(BrokenKickers_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutableBrokenKickers()
    {
        return BrokenKickers_param;
    }


    const std::string name() const
    {
        return "RobotCapabilitiesConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help                   = false;
            std::string BrokenDribblers = "";
            std::string BrokenChippers  = "";
            std::string BrokenKickers   = "";
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()(
            "BrokenDribblers",
            boost::program_options::value<std::string>(&args.BrokenDribblers),
            "Comma-separated list of numbers of robots with broken dribblers");
        desc.add_options()(
            "BrokenChippers",
            boost::program_options::value<std::string>(&args.BrokenChippers),
            "Comma-separated list of numbers of robots with broken chippers");
        desc.add_options()(
            "BrokenKickers",
            boost::program_options::value<std::string>(&args.BrokenKickers),
            "Comma-separated list of numbers of robots with broken kickers");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableBrokenDribblers()->setValue(args.BrokenDribblers);
        this->mutableBrokenChippers()->setValue(args.BrokenChippers);
        this->mutableBrokenKickers()->setValue(args.BrokenKickers);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<std::string>> BrokenDribblers_param;
    std::shared_ptr<Parameter<std::string>> BrokenChippers_param;
    std::shared_ptr<Parameter<std::string>> BrokenKickers_param;
};
class RobotDiagnosticsMainCommandLineArgs : public Config
{
   public:
    RobotDiagnosticsMainCommandLineArgs()
    {
        init();
    }
    void init()
    {
        backend_param   = std::make_shared<Parameter<std::string>>("backend", "");
        interface_param = std::make_shared<Parameter<std::string>>("interface", "");
        mutable_internal_param_list   = {backend_param, interface_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<std::string>>(backend_param),
            std::const_pointer_cast<const Parameter<std::string>>(interface_param)};
    }
    const std::shared_ptr<const Parameter<std::string>> backend() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(backend_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutablebackend()
    {
        return backend_param;
    }

    const std::shared_ptr<const Parameter<std::string>> interface() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(interface_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutableinterface()
    {
        return interface_param;
    }


    const std::string name() const
    {
        return "RobotDiagnosticsMainCommandLineArgs";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help             = false;
            std::string backend   = "";
            std::string interface = "";
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()(
            "backend", boost::program_options::value<std::string>(&args.backend),
            "The name of the backend you would like to use. See /software/backend for options ex. class WifiBackend this argument would be WifiBackend");
        desc.add_options()(
            "interface", boost::program_options::value<std::string>(&args.interface),
            "The interface to send and receive packets over (can be found through ifconfig)");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutablebackend()->setValue(args.backend);
        this->mutableinterface()->setValue(args.interface);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<std::string>> backend_param;
    std::shared_ptr<Parameter<std::string>> interface_param;
};
class SensorFusionConfig : public Config
{
   public:
    SensorFusionConfig()
    {
        init();
    }
    void init()
    {
        VisionFlippingFilterEnabled_param =
            std::make_shared<Parameter<bool>>("VisionFlippingFilterEnabled", true);
        IgnoreInvalidCameraData_param =
            std::make_shared<Parameter<bool>>("IgnoreInvalidCameraData", false);
        MinValidX_param        = std::make_shared<Parameter<double>>("MinValidX", -10.0);
        MaxValidX_param        = std::make_shared<Parameter<double>>("MaxValidX", 10.0);
        FriendlyGoalieId_param = std::make_shared<Parameter<int>>("FriendlyGoalieId", 0);
        EnemyGoalieId_param    = std::make_shared<Parameter<int>>("EnemyGoalieId", 0);
        OverrideGameControllerDefendingSide_param = std::make_shared<Parameter<bool>>(
            "OverrideGameControllerDefendingSide", true);
        DefendingPositiveSide_param =
            std::make_shared<Parameter<bool>>("DefendingPositiveSide", false);
        FriendlyColorYellow_param =
            std::make_shared<Parameter<bool>>("FriendlyColorYellow", true);
        OverrideGameControllerFriendlyGoalieID_param = std::make_shared<Parameter<bool>>(
            "OverrideGameControllerFriendlyGoalieID", true);
        OverrideGameControllerEnemyGoalieID_param = std::make_shared<Parameter<bool>>(
            "OverrideGameControllerEnemyGoalieID", true);
        mutable_internal_param_list   = {VisionFlippingFilterEnabled_param,
                                       IgnoreInvalidCameraData_param,
                                       MinValidX_param,
                                       MaxValidX_param,
                                       FriendlyGoalieId_param,
                                       EnemyGoalieId_param,
                                       OverrideGameControllerDefendingSide_param,
                                       DefendingPositiveSide_param,
                                       FriendlyColorYellow_param,
                                       OverrideGameControllerFriendlyGoalieID_param,
                                       OverrideGameControllerEnemyGoalieID_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<bool>>(
                VisionFlippingFilterEnabled_param),
            std::const_pointer_cast<const Parameter<bool>>(IgnoreInvalidCameraData_param),
            std::const_pointer_cast<const Parameter<double>>(MinValidX_param),
            std::const_pointer_cast<const Parameter<double>>(MaxValidX_param),
            std::const_pointer_cast<const Parameter<int>>(FriendlyGoalieId_param),
            std::const_pointer_cast<const Parameter<int>>(EnemyGoalieId_param),
            std::const_pointer_cast<const Parameter<bool>>(
                OverrideGameControllerDefendingSide_param),
            std::const_pointer_cast<const Parameter<bool>>(DefendingPositiveSide_param),
            std::const_pointer_cast<const Parameter<bool>>(FriendlyColorYellow_param),
            std::const_pointer_cast<const Parameter<bool>>(
                OverrideGameControllerFriendlyGoalieID_param),
            std::const_pointer_cast<const Parameter<bool>>(
                OverrideGameControllerEnemyGoalieID_param)};
    }
    const std::shared_ptr<const Parameter<bool>> VisionFlippingFilterEnabled() const
    {
        return std::const_pointer_cast<const Parameter<bool>>(
            VisionFlippingFilterEnabled_param);
    }

    const std::shared_ptr<Parameter<bool>> mutableVisionFlippingFilterEnabled()
    {
        return VisionFlippingFilterEnabled_param;
    }

    const std::shared_ptr<const Parameter<bool>> IgnoreInvalidCameraData() const
    {
        return std::const_pointer_cast<const Parameter<bool>>(
            IgnoreInvalidCameraData_param);
    }

    const std::shared_ptr<Parameter<bool>> mutableIgnoreInvalidCameraData()
    {
        return IgnoreInvalidCameraData_param;
    }

    const std::shared_ptr<const Parameter<double>> MinValidX() const
    {
        return std::const_pointer_cast<const Parameter<double>>(MinValidX_param);
    }

    const std::shared_ptr<Parameter<double>> mutableMinValidX()
    {
        return MinValidX_param;
    }

    const std::shared_ptr<const Parameter<double>> MaxValidX() const
    {
        return std::const_pointer_cast<const Parameter<double>>(MaxValidX_param);
    }

    const std::shared_ptr<Parameter<double>> mutableMaxValidX()
    {
        return MaxValidX_param;
    }

    const std::shared_ptr<const Parameter<int>> FriendlyGoalieId() const
    {
        return std::const_pointer_cast<const Parameter<int>>(FriendlyGoalieId_param);
    }

    const std::shared_ptr<Parameter<int>> mutableFriendlyGoalieId()
    {
        return FriendlyGoalieId_param;
    }

    const std::shared_ptr<const Parameter<int>> EnemyGoalieId() const
    {
        return std::const_pointer_cast<const Parameter<int>>(EnemyGoalieId_param);
    }

    const std::shared_ptr<Parameter<int>> mutableEnemyGoalieId()
    {
        return EnemyGoalieId_param;
    }

    const std::shared_ptr<const Parameter<bool>> OverrideGameControllerDefendingSide()
        const
    {
        return std::const_pointer_cast<const Parameter<bool>>(
            OverrideGameControllerDefendingSide_param);
    }

    const std::shared_ptr<Parameter<bool>> mutableOverrideGameControllerDefendingSide()
    {
        return OverrideGameControllerDefendingSide_param;
    }

    const std::shared_ptr<const Parameter<bool>> DefendingPositiveSide() const
    {
        return std::const_pointer_cast<const Parameter<bool>>(
            DefendingPositiveSide_param);
    }

    const std::shared_ptr<Parameter<bool>> mutableDefendingPositiveSide()
    {
        return DefendingPositiveSide_param;
    }

    const std::shared_ptr<const Parameter<bool>> FriendlyColorYellow() const
    {
        return std::const_pointer_cast<const Parameter<bool>>(FriendlyColorYellow_param);
    }

    const std::shared_ptr<Parameter<bool>> mutableFriendlyColorYellow()
    {
        return FriendlyColorYellow_param;
    }

    const std::shared_ptr<const Parameter<bool>> OverrideGameControllerFriendlyGoalieID()
        const
    {
        return std::const_pointer_cast<const Parameter<bool>>(
            OverrideGameControllerFriendlyGoalieID_param);
    }

    const std::shared_ptr<Parameter<bool>> mutableOverrideGameControllerFriendlyGoalieID()
    {
        return OverrideGameControllerFriendlyGoalieID_param;
    }

    const std::shared_ptr<const Parameter<bool>> OverrideGameControllerEnemyGoalieID()
        const
    {
        return std::const_pointer_cast<const Parameter<bool>>(
            OverrideGameControllerEnemyGoalieID_param);
    }

    const std::shared_ptr<Parameter<bool>> mutableOverrideGameControllerEnemyGoalieID()
    {
        return OverrideGameControllerEnemyGoalieID_param;
    }


    const std::string name() const
    {
        return "SensorFusionConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help                                   = false;
            bool VisionFlippingFilterEnabled            = true;
            bool IgnoreInvalidCameraData                = false;
            double MinValidX                            = -10.0;
            double MaxValidX                            = 10.0;
            int FriendlyGoalieId                        = 0;
            int EnemyGoalieId                           = 0;
            bool OverrideGameControllerDefendingSide    = true;
            bool DefendingPositiveSide                  = false;
            bool FriendlyColorYellow                    = true;
            bool OverrideGameControllerFriendlyGoalieID = true;
            bool OverrideGameControllerEnemyGoalieID    = true;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()(
            "VisionFlippingFilterEnabled",
            boost::program_options::bool_switch(&args.VisionFlippingFilterEnabled),
            "Ignores frames if our goalie appears in the opponent defense area");
        desc.add_options()(
            "IgnoreInvalidCameraData",
            boost::program_options::bool_switch(&args.IgnoreInvalidCameraData),
            "Whether or not to ignore invalid camera data. If this value is true, any ball or robot detections that are not within the min and max valid x coordinates will be ignored. If this value is false, all data is collected as normal and not ignored.");
        desc.add_options()(
            "MinValidX", boost::program_options::value<double>(&args.MinValidX),
            "When ignoreInvalidCameraData is true, any robot or ball detection with an x-coordinate less than this value is ignored.");
        desc.add_options()(
            "MaxValidX", boost::program_options::value<double>(&args.MaxValidX),
            "When ignoreInvalidCameraData is true, any robot or ball detection with an x-coordinate greater than this value is ignored.");
        desc.add_options()("FriendlyGoalieId",
                           boost::program_options::value<int>(&args.FriendlyGoalieId),
                           "The id of the friendly goalie");
        desc.add_options()("EnemyGoalieId",
                           boost::program_options::value<int>(&args.EnemyGoalieId),
                           "The id of the enemy goalie");
        desc.add_options()(
            "OverrideGameControllerDefendingSide",
            boost::program_options::bool_switch(
                &args.OverrideGameControllerDefendingSide),
            "Overrides the defending side provided by the referee, with defendingPositiveSide parameter");
        desc.add_options()(
            "DefendingPositiveSide",
            boost::program_options::bool_switch(&args.DefendingPositiveSide),
            "Positive if selected, Negative if unselected");
        desc.add_options()("FriendlyColorYellow",
                           boost::program_options::bool_switch(&args.FriendlyColorYellow),
                           "Yellow if selected, Blue if unselected");
        desc.add_options()(
            "OverrideGameControllerFriendlyGoalieID",
            boost::program_options::bool_switch(
                &args.OverrideGameControllerFriendlyGoalieID),
            "Overrides the friendly goalie id provided by the game controller, with FriendlyGoalieId parameter");
        desc.add_options()(
            "OverrideGameControllerEnemyGoalieID",
            boost::program_options::bool_switch(
                &args.OverrideGameControllerEnemyGoalieID),
            "Overrides the enemy goalie id provided by the game controller, with EnemyGoalieId parameter");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableVisionFlippingFilterEnabled()->setValue(
            args.VisionFlippingFilterEnabled);
        this->mutableIgnoreInvalidCameraData()->setValue(args.IgnoreInvalidCameraData);
        this->mutableMinValidX()->setValue(args.MinValidX);
        this->mutableMaxValidX()->setValue(args.MaxValidX);
        this->mutableFriendlyGoalieId()->setValue(args.FriendlyGoalieId);
        this->mutableEnemyGoalieId()->setValue(args.EnemyGoalieId);
        this->mutableOverrideGameControllerDefendingSide()->setValue(
            args.OverrideGameControllerDefendingSide);
        this->mutableDefendingPositiveSide()->setValue(args.DefendingPositiveSide);
        this->mutableFriendlyColorYellow()->setValue(args.FriendlyColorYellow);
        this->mutableOverrideGameControllerFriendlyGoalieID()->setValue(
            args.OverrideGameControllerFriendlyGoalieID);
        this->mutableOverrideGameControllerEnemyGoalieID()->setValue(
            args.OverrideGameControllerEnemyGoalieID);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<bool>> VisionFlippingFilterEnabled_param;
    std::shared_ptr<Parameter<bool>> IgnoreInvalidCameraData_param;
    std::shared_ptr<Parameter<double>> MinValidX_param;
    std::shared_ptr<Parameter<double>> MaxValidX_param;
    std::shared_ptr<Parameter<int>> FriendlyGoalieId_param;
    std::shared_ptr<Parameter<int>> EnemyGoalieId_param;
    std::shared_ptr<Parameter<bool>> OverrideGameControllerDefendingSide_param;
    std::shared_ptr<Parameter<bool>> DefendingPositiveSide_param;
    std::shared_ptr<Parameter<bool>> FriendlyColorYellow_param;
    std::shared_ptr<Parameter<bool>> OverrideGameControllerFriendlyGoalieID_param;
    std::shared_ptr<Parameter<bool>> OverrideGameControllerEnemyGoalieID_param;
};
class SimulatedTestMainCommandLineArgs : public Config
{
   public:
    SimulatedTestMainCommandLineArgs()
    {
        init();
    }
    void init()
    {
        enable_visualizer_param =
            std::make_shared<Parameter<bool>>("enable_visualizer", false);
        mutable_internal_param_list   = {enable_visualizer_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<bool>>(enable_visualizer_param)};
    }
    const std::shared_ptr<const Parameter<bool>> enable_visualizer() const
    {
        return std::const_pointer_cast<const Parameter<bool>>(enable_visualizer_param);
    }

    const std::shared_ptr<Parameter<bool>> mutableenable_visualizer()
    {
        return enable_visualizer_param;
    }


    const std::string name() const
    {
        return "SimulatedTestMainCommandLineArgs";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help              = false;
            bool enable_visualizer = false;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()("enable_visualizer",
                           boost::program_options::bool_switch(&args.enable_visualizer),
                           "Displays simulated test on visualizer");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableenable_visualizer()->setValue(args.enable_visualizer);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<bool>> enable_visualizer_param;
};
class SimulatorConfig : public Config
{
   public:
    SimulatorConfig()
    {
        init();
    }
    void init()
    {
        SlidingFrictionAcceleration_param =
            std::make_shared<Parameter<double>>("SlidingFrictionAcceleration", 0.0);
        RollingFrictionAcceleration_param =
            std::make_shared<Parameter<double>>("RollingFrictionAcceleration", 0.0);
        BallRestitution_param =
            std::make_shared<Parameter<double>>("BallRestitution", 1.0);
        mutable_internal_param_list   = {SlidingFrictionAcceleration_param,
                                       RollingFrictionAcceleration_param,
                                       BallRestitution_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<double>>(
                SlidingFrictionAcceleration_param),
            std::const_pointer_cast<const Parameter<double>>(
                RollingFrictionAcceleration_param),
            std::const_pointer_cast<const Parameter<double>>(BallRestitution_param)};
    }
    const std::shared_ptr<const Parameter<double>> SlidingFrictionAcceleration() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            SlidingFrictionAcceleration_param);
    }

    const std::shared_ptr<Parameter<double>> mutableSlidingFrictionAcceleration()
    {
        return SlidingFrictionAcceleration_param;
    }

    const std::shared_ptr<const Parameter<double>> RollingFrictionAcceleration() const
    {
        return std::const_pointer_cast<const Parameter<double>>(
            RollingFrictionAcceleration_param);
    }

    const std::shared_ptr<Parameter<double>> mutableRollingFrictionAcceleration()
    {
        return RollingFrictionAcceleration_param;
    }

    const std::shared_ptr<const Parameter<double>> BallRestitution() const
    {
        return std::const_pointer_cast<const Parameter<double>>(BallRestitution_param);
    }

    const std::shared_ptr<Parameter<double>> mutableBallRestitution()
    {
        return BallRestitution_param;
    }


    const std::string name() const
    {
        return "SimulatorConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help                          = false;
            double SlidingFrictionAcceleration = 0.0;
            double RollingFrictionAcceleration = 0.0;
            double BallRestitution             = 1.0;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()(
            "SlidingFrictionAcceleration",
            boost::program_options::value<double>(&args.SlidingFrictionAcceleration),
            "The scalar friction acceleration in m/s^2 that is applied to the ball  while the ball is sliding");
        desc.add_options()(
            "RollingFrictionAcceleration",
            boost::program_options::value<double>(&args.RollingFrictionAcceleration),
            "The scalar friction acceleration in m/s^2 that is applied to the ball  while the ball is rolling");
        desc.add_options()(
            "BallRestitution",
            boost::program_options::value<double>(&args.BallRestitution),
            "The restitution is the amount of energy retained when bouncing off walls and robots, 0.0 means perfectly inelastic and 1.0 means perfectly elastic collision.");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableSlidingFrictionAcceleration()->setValue(
            args.SlidingFrictionAcceleration);
        this->mutableRollingFrictionAcceleration()->setValue(
            args.RollingFrictionAcceleration);
        this->mutableBallRestitution()->setValue(args.BallRestitution);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<double>> SlidingFrictionAcceleration_param;
    std::shared_ptr<Parameter<double>> RollingFrictionAcceleration_param;
    std::shared_ptr<Parameter<double>> BallRestitution_param;
};
class StandaloneSimulatorConfig : public Config
{
   public:
    StandaloneSimulatorConfig()
    {
        init();
    }
    void init()
    {
        YellowTeamChannel_param =
            std::make_shared<Parameter<int>>("YellowTeamChannel", 0);
        BlueTeamChannel_param = std::make_shared<Parameter<int>>("BlueTeamChannel", 1);
        NetworkInterface_param =
            std::make_shared<Parameter<std::string>>("NetworkInterface", "wlp3s0");
        VisionIPv4Address_param =
            std::make_shared<Parameter<std::string>>("VisionIPv4Address", "224.5.23.2");
        VisionPort_param = std::make_shared<Parameter<int>>("VisionPort", 10020);
        mutable_internal_param_list   = {YellowTeamChannel_param, BlueTeamChannel_param,
                                       NetworkInterface_param, VisionIPv4Address_param,
                                       VisionPort_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<int>>(YellowTeamChannel_param),
            std::const_pointer_cast<const Parameter<int>>(BlueTeamChannel_param),
            std::const_pointer_cast<const Parameter<std::string>>(NetworkInterface_param),
            std::const_pointer_cast<const Parameter<std::string>>(
                VisionIPv4Address_param),
            std::const_pointer_cast<const Parameter<int>>(VisionPort_param)};
    }
    const std::shared_ptr<const Parameter<int>> YellowTeamChannel() const
    {
        return std::const_pointer_cast<const Parameter<int>>(YellowTeamChannel_param);
    }

    const std::shared_ptr<Parameter<int>> mutableYellowTeamChannel()
    {
        return YellowTeamChannel_param;
    }

    const std::shared_ptr<const Parameter<int>> BlueTeamChannel() const
    {
        return std::const_pointer_cast<const Parameter<int>>(BlueTeamChannel_param);
    }

    const std::shared_ptr<Parameter<int>> mutableBlueTeamChannel()
    {
        return BlueTeamChannel_param;
    }

    const std::shared_ptr<const Parameter<std::string>> NetworkInterface() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(
            NetworkInterface_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutableNetworkInterface()
    {
        return NetworkInterface_param;
    }

    const std::shared_ptr<const Parameter<std::string>> VisionIPv4Address() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(
            VisionIPv4Address_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutableVisionIPv4Address()
    {
        return VisionIPv4Address_param;
    }

    const std::shared_ptr<const Parameter<int>> VisionPort() const
    {
        return std::const_pointer_cast<const Parameter<int>>(VisionPort_param);
    }

    const std::shared_ptr<Parameter<int>> mutableVisionPort()
    {
        return VisionPort_param;
    }


    const std::string name() const
    {
        return "StandaloneSimulatorConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help                     = false;
            int YellowTeamChannel         = 0;
            int BlueTeamChannel           = 1;
            std::string NetworkInterface  = "wlp3s0";
            std::string VisionIPv4Address = "224.5.23.2";
            int VisionPort                = 10020;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()(
            "YellowTeamChannel",
            boost::program_options::value<int>(&args.YellowTeamChannel),
            "The multicast channel to use to communicate with the yellow robots in the simulator");
        desc.add_options()(
            "BlueTeamChannel", boost::program_options::value<int>(&args.BlueTeamChannel),
            "The multicast channel to use to communicate with the blue robots in the simulator");
        desc.add_options()(
            "NetworkInterface",
            boost::program_options::value<std::string>(&args.NetworkInterface),
            "The network interface to use for multicast network communication. Can be found using the `ifconfig` command.");
        desc.add_options()(
            "VisionIPv4Address",
            boost::program_options::value<std::string>(&args.VisionIPv4Address),
            "The IPv4 address to send vision data to");
        desc.add_options()("VisionPort",
                           boost::program_options::value<int>(&args.VisionPort),
                           "The port to send vision data to");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableYellowTeamChannel()->setValue(args.YellowTeamChannel);
        this->mutableBlueTeamChannel()->setValue(args.BlueTeamChannel);
        this->mutableNetworkInterface()->setValue(args.NetworkInterface);
        this->mutableVisionIPv4Address()->setValue(args.VisionIPv4Address);
        this->mutableVisionPort()->setValue(args.VisionPort);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<int>> YellowTeamChannel_param;
    std::shared_ptr<Parameter<int>> BlueTeamChannel_param;
    std::shared_ptr<Parameter<std::string>> NetworkInterface_param;
    std::shared_ptr<Parameter<std::string>> VisionIPv4Address_param;
    std::shared_ptr<Parameter<int>> VisionPort_param;
};
class StandaloneSimulatorMainCommandLineArgs : public Config
{
   public:
    StandaloneSimulatorMainCommandLineArgs()
    {
        init();
    }
    void init()
    {
        interface_param = std::make_shared<Parameter<std::string>>("interface", "");
        mutable_internal_param_list   = {interface_param};
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<std::string>>(interface_param)};
    }
    const std::shared_ptr<const Parameter<std::string>> interface() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(interface_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutableinterface()
    {
        return interface_param;
    }


    const std::string name() const
    {
        return "StandaloneSimulatorMainCommandLineArgs";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help             = false;
            std::string interface = "";
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        desc.add_options()(
            "interface", boost::program_options::value<std::string>(&args.interface),
            "The interface to send and receive packets over (can be found through ifconfig)");


        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableinterface()->setValue(args.interface);


        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<std::string>> interface_param;
};
class ThunderbotsConfig : public Config
{
   public:
    ThunderbotsConfig()
    {
        init();
    }
    void init()
    {
        AIConfig_config = std::make_shared<AIConfig>();

        AIControlConfig_config = std::make_shared<AIControlConfig>();

        EnemyCapabilityConfig_config = std::make_shared<EnemyCapabilityConfig>();

        FullSystemMainCommandLineArgs_config =
            std::make_shared<FullSystemMainCommandLineArgs>();

        HandheldControllerConfig_config = std::make_shared<HandheldControllerConfig>();

        NetworkConfig_config = std::make_shared<NetworkConfig>();

        RobotCapabilitiesConfig_config = std::make_shared<RobotCapabilitiesConfig>();

        RobotDiagnosticsMainCommandLineArgs_config =
            std::make_shared<RobotDiagnosticsMainCommandLineArgs>();

        SensorFusionConfig_config = std::make_shared<SensorFusionConfig>();

        SimulatedTestMainCommandLineArgs_config =
            std::make_shared<SimulatedTestMainCommandLineArgs>();

        SimulatorConfig_config = std::make_shared<SimulatorConfig>();

        StandaloneSimulatorConfig_config = std::make_shared<StandaloneSimulatorConfig>();

        StandaloneSimulatorMainCommandLineArgs_config =
            std::make_shared<StandaloneSimulatorMainCommandLineArgs>();

        mutable_internal_param_list   = {AIConfig_config,
                                       AIControlConfig_config,
                                       EnemyCapabilityConfig_config,
                                       FullSystemMainCommandLineArgs_config,
                                       HandheldControllerConfig_config,
                                       NetworkConfig_config,
                                       RobotCapabilitiesConfig_config,
                                       RobotDiagnosticsMainCommandLineArgs_config,
                                       SensorFusionConfig_config,
                                       SimulatedTestMainCommandLineArgs_config,
                                       SimulatorConfig_config,
                                       StandaloneSimulatorConfig_config,
                                       StandaloneSimulatorMainCommandLineArgs_config};
        immutable_internal_param_list = {
            std::const_pointer_cast<const AIConfig>(AIConfig_config),
            std::const_pointer_cast<const AIControlConfig>(AIControlConfig_config),
            std::const_pointer_cast<const EnemyCapabilityConfig>(
                EnemyCapabilityConfig_config),
            std::const_pointer_cast<const FullSystemMainCommandLineArgs>(
                FullSystemMainCommandLineArgs_config),
            std::const_pointer_cast<const HandheldControllerConfig>(
                HandheldControllerConfig_config),
            std::const_pointer_cast<const NetworkConfig>(NetworkConfig_config),
            std::const_pointer_cast<const RobotCapabilitiesConfig>(
                RobotCapabilitiesConfig_config),
            std::const_pointer_cast<const RobotDiagnosticsMainCommandLineArgs>(
                RobotDiagnosticsMainCommandLineArgs_config),
            std::const_pointer_cast<const SensorFusionConfig>(SensorFusionConfig_config),
            std::const_pointer_cast<const SimulatedTestMainCommandLineArgs>(
                SimulatedTestMainCommandLineArgs_config),
            std::const_pointer_cast<const SimulatorConfig>(SimulatorConfig_config),
            std::const_pointer_cast<const StandaloneSimulatorConfig>(
                StandaloneSimulatorConfig_config),
            std::const_pointer_cast<const StandaloneSimulatorMainCommandLineArgs>(
                StandaloneSimulatorMainCommandLineArgs_config)};
    }
    const std::shared_ptr<const AIConfig> getAIConfig() const
    {
        return std::const_pointer_cast<const AIConfig>(AIConfig_config);
    }

    const std::shared_ptr<AIConfig> getMutableAIConfig()
    {
        return AIConfig_config;
    }

    const std::shared_ptr<const AIControlConfig> getAIControlConfig() const
    {
        return std::const_pointer_cast<const AIControlConfig>(AIControlConfig_config);
    }

    const std::shared_ptr<AIControlConfig> getMutableAIControlConfig()
    {
        return AIControlConfig_config;
    }

    const std::shared_ptr<const EnemyCapabilityConfig> getEnemyCapabilityConfig() const
    {
        return std::const_pointer_cast<const EnemyCapabilityConfig>(
            EnemyCapabilityConfig_config);
    }

    const std::shared_ptr<EnemyCapabilityConfig> getMutableEnemyCapabilityConfig()
    {
        return EnemyCapabilityConfig_config;
    }

    const std::shared_ptr<const FullSystemMainCommandLineArgs>
    getFullSystemMainCommandLineArgs() const
    {
        return std::const_pointer_cast<const FullSystemMainCommandLineArgs>(
            FullSystemMainCommandLineArgs_config);
    }

    const std::shared_ptr<FullSystemMainCommandLineArgs>
    getMutableFullSystemMainCommandLineArgs()
    {
        return FullSystemMainCommandLineArgs_config;
    }

    const std::shared_ptr<const HandheldControllerConfig> getHandheldControllerConfig()
        const
    {
        return std::const_pointer_cast<const HandheldControllerConfig>(
            HandheldControllerConfig_config);
    }

    const std::shared_ptr<HandheldControllerConfig> getMutableHandheldControllerConfig()
    {
        return HandheldControllerConfig_config;
    }

    const std::shared_ptr<const NetworkConfig> getNetworkConfig() const
    {
        return std::const_pointer_cast<const NetworkConfig>(NetworkConfig_config);
    }

    const std::shared_ptr<NetworkConfig> getMutableNetworkConfig()
    {
        return NetworkConfig_config;
    }

    const std::shared_ptr<const RobotCapabilitiesConfig> getRobotCapabilitiesConfig()
        const
    {
        return std::const_pointer_cast<const RobotCapabilitiesConfig>(
            RobotCapabilitiesConfig_config);
    }

    const std::shared_ptr<RobotCapabilitiesConfig> getMutableRobotCapabilitiesConfig()
    {
        return RobotCapabilitiesConfig_config;
    }

    const std::shared_ptr<const RobotDiagnosticsMainCommandLineArgs>
    getRobotDiagnosticsMainCommandLineArgs() const
    {
        return std::const_pointer_cast<const RobotDiagnosticsMainCommandLineArgs>(
            RobotDiagnosticsMainCommandLineArgs_config);
    }

    const std::shared_ptr<RobotDiagnosticsMainCommandLineArgs>
    getMutableRobotDiagnosticsMainCommandLineArgs()
    {
        return RobotDiagnosticsMainCommandLineArgs_config;
    }

    const std::shared_ptr<const SensorFusionConfig> getSensorFusionConfig() const
    {
        return std::const_pointer_cast<const SensorFusionConfig>(
            SensorFusionConfig_config);
    }

    const std::shared_ptr<SensorFusionConfig> getMutableSensorFusionConfig()
    {
        return SensorFusionConfig_config;
    }

    const std::shared_ptr<const SimulatedTestMainCommandLineArgs>
    getSimulatedTestMainCommandLineArgs() const
    {
        return std::const_pointer_cast<const SimulatedTestMainCommandLineArgs>(
            SimulatedTestMainCommandLineArgs_config);
    }

    const std::shared_ptr<SimulatedTestMainCommandLineArgs>
    getMutableSimulatedTestMainCommandLineArgs()
    {
        return SimulatedTestMainCommandLineArgs_config;
    }

    const std::shared_ptr<const SimulatorConfig> getSimulatorConfig() const
    {
        return std::const_pointer_cast<const SimulatorConfig>(SimulatorConfig_config);
    }

    const std::shared_ptr<SimulatorConfig> getMutableSimulatorConfig()
    {
        return SimulatorConfig_config;
    }

    const std::shared_ptr<const StandaloneSimulatorConfig> getStandaloneSimulatorConfig()
        const
    {
        return std::const_pointer_cast<const StandaloneSimulatorConfig>(
            StandaloneSimulatorConfig_config);
    }

    const std::shared_ptr<StandaloneSimulatorConfig> getMutableStandaloneSimulatorConfig()
    {
        return StandaloneSimulatorConfig_config;
    }

    const std::shared_ptr<const StandaloneSimulatorMainCommandLineArgs>
    getStandaloneSimulatorMainCommandLineArgs() const
    {
        return std::const_pointer_cast<const StandaloneSimulatorMainCommandLineArgs>(
            StandaloneSimulatorMainCommandLineArgs_config);
    }

    const std::shared_ptr<StandaloneSimulatorMainCommandLineArgs>
    getMutableStandaloneSimulatorMainCommandLineArgs()
    {
        return StandaloneSimulatorMainCommandLineArgs_config;
    }


    const std::string name() const
    {
        return "ThunderbotsConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help = false;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");



        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);



        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<AIConfig> AIConfig_config;
    std::shared_ptr<AIControlConfig> AIControlConfig_config;
    std::shared_ptr<EnemyCapabilityConfig> EnemyCapabilityConfig_config;
    std::shared_ptr<FullSystemMainCommandLineArgs> FullSystemMainCommandLineArgs_config;
    std::shared_ptr<HandheldControllerConfig> HandheldControllerConfig_config;
    std::shared_ptr<NetworkConfig> NetworkConfig_config;
    std::shared_ptr<RobotCapabilitiesConfig> RobotCapabilitiesConfig_config;
    std::shared_ptr<RobotDiagnosticsMainCommandLineArgs>
        RobotDiagnosticsMainCommandLineArgs_config;
    std::shared_ptr<SensorFusionConfig> SensorFusionConfig_config;
    std::shared_ptr<SimulatedTestMainCommandLineArgs>
        SimulatedTestMainCommandLineArgs_config;
    std::shared_ptr<SimulatorConfig> SimulatorConfig_config;
    std::shared_ptr<StandaloneSimulatorConfig> StandaloneSimulatorConfig_config;
    std::shared_ptr<StandaloneSimulatorMainCommandLineArgs>
        StandaloneSimulatorMainCommandLineArgs_config;
};

// TODO remove this as part of https://github.com/UBC-Thunderbots/Software/issues/960
extern const std::shared_ptr<ThunderbotsConfig> MutableDynamicParameters;
extern const std::shared_ptr<const ThunderbotsConfig> DynamicParameters;
