#include "ai/hl/stp/play/play.h"
#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"

class MovePlay : public Play
{
   public:
    MovePlay() : Play(){};

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

    std::vector<std::shared_ptr<Tactic>> getNextTactics(
        tactic_coroutine::push_type &yield, const World &world) override
    {
        auto move_tactic_1 = std::make_shared<MoveTactic>();
        auto move_tactic_2 = std::make_shared<MoveTactic>();
        auto move_tactic_3 = std::make_shared<MoveTactic>();
        auto move_tactic_4 = std::make_shared<MoveTactic>();
        auto move_tactic_5 = std::make_shared<MoveTactic>();
        auto move_tactic_6 = std::make_shared<MoveTactic>();

        do
        {
            move_tactic_1->updateParams(Point::createFromAngle(Angle::full() / 6 * 1),
                                        Angle::zero(), 0);
            move_tactic_2->updateParams(Point::createFromAngle(Angle::full() / 6 * 2),
                                        Angle::zero(), 0);
            move_tactic_3->updateParams(Point::createFromAngle(Angle::full() / 6 * 3),
                                        Angle::zero(), 0);
            move_tactic_4->updateParams(Point::createFromAngle(Angle::full() / 6 * 4),
                                        Angle::zero(), 0);
            move_tactic_5->updateParams(Point::createFromAngle(Angle::full() / 6 * 5),
                                        Angle::zero(), 0);
            move_tactic_6->updateParams(Point::createFromAngle(Angle::full() / 6 * 6),
                                        Angle::zero(), 0);

            yield({move_tactic_1, move_tactic_2, move_tactic_3, move_tactic_4,
                   move_tactic_5, move_tactic_6});
        } while (true);
    }
};

static TPlayFactory<MovePlay> factory;
