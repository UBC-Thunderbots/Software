#include "ai/hl/stp/play/play.h"
#include "ai/hl/stp/tactic/move_tactic.h"

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

        return tactics;
    }
};

static TPlayFactory<MovePlay> factory;
