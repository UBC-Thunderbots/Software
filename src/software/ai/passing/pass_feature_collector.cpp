
#include "pass_feature_collector.h"

#include <utility>

#include "cost_function.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/contains.h"
#include "software/logger/compat_flags.h"

PassFeatureCollector::PassFeatureCollector(const std::string& log_dir)
{
    fs::create_directories(log_dir);
    std::string log_file_path = log_dir + PASS_FEATURE_FILE;

    log_file_ = gzopen(log_file_path.c_str(), "wb");
    if (!log_file_)
    {
        throw std::runtime_error("PassFeatureCollector: Logger failed to initialize");
    }
}

void PassFeatureCollector::logPassFeatures(const Pass& pass, const World& world,
                                           TbotsProto::PassingConfig passing_config) const
{
    int score = getPassScore(pass, world, passing_config);

    std::string logEntry = createDatasetEntry(pass, world, score);

    if (!logDatasetEntry(logEntry))
    {
        std::cerr << "PassFeatureCollector: Failed to write to log file: " << std::endl;
    }
}

bool PassFeatureCollector::logDatasetEntry(std::string entry) const
{
    int num_bytes_written = gzwrite(log_file_, entry.c_str(),
                                        static_cast<unsigned>(entry.size()));

    // Check if write was successful
    return num_bytes_written == static_cast<int>(entry.size());
}


std::string PassFeatureCollector::createDatasetEntry(const Pass& pass, const World& world,
                                                     int score)
{
    std::stringstream dataset_entry_ss;

    dataset_entry_ss << createPointFeatureEntry(pass.passerPoint())
                     << PASS_FEATURE_DELIMITER
                     << createPointFeatureEntry(pass.receiverPoint())
                     << PASS_FEATURE_DELIMITER
                     << createPointFeatureEntry(world.ball().position())
                     << PASS_FEATURE_DELIMITER
                     << createTeamFeatureEntry(world.friendlyTeam())
                     << PASS_FEATURE_DELIMITER
                     << createTeamFeatureEntry(world.enemyTeam())
                     << PASS_FEATURE_DELIMITER << score << "\n";

    return dataset_entry_ss.str();
}

std::string PassFeatureCollector::createPointFeatureEntry(const Point& point)
{
    std::stringstream point_ss;
    point_ss << point.x() << PASS_FEATURE_DELIMITER << point.y();
    return point_ss.str();
}

std::string PassFeatureCollector::createTeamFeatureEntry(const Team& team)
{
    std::stringstream team_ss;

    if (auto teamRobots = team.getAllRobots(); !teamRobots.empty())
    {
        auto iterator = teamRobots.begin();
        team_ss << createPointFeatureEntry(iterator->position());
        ++iterator;

        for (; iterator != teamRobots.end(); ++iterator)
        {
            team_ss << PASS_FEATURE_DELIMITER
                    << createPointFeatureEntry(iterator->position());
        }
    }

    return team_ss.str();
}

int PassFeatureCollector::getPassScore(const Pass& pass, const World& world,
                                       TbotsProto::PassingConfig passing_config)
{
    // if the pass has 0 speed
    if (pass.speed() == 0)
        return DEFINITELY_BAD_SCORE;

    // if there are no receivers on the friendly team
    if (world.friendlyTeam().getAllRobots().size() <= 0)
        return DEFINITELY_BAD_SCORE;

    // if pass is not far enough from the passer
    auto passLength = (pass.receiverPoint() - pass.passerPoint()).length();
    if (passLength < passing_config.receiver_ideal_min_distance_meters())
        return DEFINITELY_BAD_SCORE;

    const auto field = world.field();

    // Make a slightly smaller field, and positive weight values in this reduced field
    // if pass receive point not in the reduced field boundaries
    if (const auto reduced_size_field = getReducedField(field, passing_config);
        !contains(reduced_size_field, pass.receiverPoint()))
        return DEFINITELY_BAD_SCORE;

    // if pass receive point is in the enemy defense area (illegal)
    if (!contains(field.enemyDefenseArea(), pass.receiverPoint()))
        return DEFINITELY_BAD_SCORE;

    auto friendlyReceiveCapabilitySigmoid =
        ratePassFriendlyCapability(world.friendlyTeam(), pass, passing_config);

    // robot arrives / turns too late
    if (friendlyReceiveCapabilitySigmoid < 0.5)
        return DEFINITELY_BAD_SCORE;

    auto passForwardSigmoid = ratePassForwardQuality(pass, passing_config);
    if (passForwardSigmoid < 0.25)
        return DEFINITELY_BAD_SCORE;
    if (passForwardSigmoid < 0.5)
        return SLIGHTLY_BAD_SCORE;

    auto shot_opt = calcBestShotOnGoal(
        Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg()), pass.receiverPoint(),
        world.enemyTeam().getAllRobots(), TeamType::ENEMY);

    // if there's no shot on goal, rate it very slightly bad
    if (!shot_opt)
        return VERY_SLIGHTLY_BAD_SCORE;

    return NEUTRAL_SCORE;
}

PassFeatureCollector::~PassFeatureCollector()
{
    gzclose(log_file_);
}
