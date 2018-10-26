#include "ai/hl/stp/tactic/move.h"

#include "ai/hl/stp/play/play.h"

class MovePlay : public Play
{
    std::string name() override
    {
        return "Move Play";
    }

    bool isApplicable(const World &world) override
    {
        return true;
    }

    bool invariantHolds(const World &world) override
    {
        return true;
    }

    bool hasFailed(const World &world) override
    {
        return false;
    }

    std::vector<std::unique_ptr<Tactic>> getTactics(const World &world) override
    {
        auto tactics = std::vector<std::unique_ptr<Tactic>>();

        tactics.emplace_back(std::make_unique<MoveTactic>(Point()));

        return tactics;
    }
};

static TPlayFactory<MovePlay> factory;
