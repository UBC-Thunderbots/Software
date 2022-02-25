#include "software/ai/hl/stp/tactic/tactic_factory.h"

#include "proto/message_translation/tbots_geometry.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "software/logger/logger.h"

std::shared_ptr<Tactic> createTactic(const TbotsProto::AttackerTactic& tactic_proto)
{
    auto config = std::make_shared<AttackerTacticConfig>();
    config->loadFromProto(tactic_proto.attacker_tactic_config());
    auto tactic = std::make_shared<AttackerTactic>(config);

    if (tactic_proto.has_best_pass_so_far())
    {
        tactic->updateControlParams(
            Pass(createPoint(tactic_proto.best_pass_so_far().passer_point()),
                 createPoint(tactic_proto.best_pass_so_far().receiver_point()),
                 tactic_proto.best_pass_so_far().pass_speed_m_per_s()),
            tactic_proto.pass_committed());
    }
    if (tactic_proto.has_chip_target())
    {
        tactic->updateControlParams(createPoint(tactic_proto.chip_target()));
    }

    return tactic;
}
