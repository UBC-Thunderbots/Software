
std::vector<EighteenZoneId> rankZones(
    std::shared_ptr<const EighteenZonePitchDivision> pitch_division,
    const Point &ball_location, Zones &cherry_pick_region_1, Zones &cherry_pick_region_2)
{
    // If the passing is coming from the friendly end, we split the cherry-pickers
    // across the x-axis in the enemy half
    //
    //                FRIENDLY          ENEMY
    //        ┌──────┬──────┬──────┬──────┬──────┬─────┐
    //        │1     │4     │7     │10    │13    │16   │
    //        │      │      │      │      │      │     │
    //        │      │      │      │      │      │     │
    //        ├──────┼──────┼──────┼──────┼──────┼─────┤
    //      ┌─┤2     │5     │8     │11    │14    │17   ├─┐
    //      │ │      │      │      │      │      │     │ │
    //      │ │      │      │      │      │      │     │ │
    //      └─┤      │      │      │      │      │     ├─┘
    //        ├──────┼──────┼──────┼──────┼──────┼─────┤
    //        │3     │6     │9     │12    │15    │18   │
    //        │      │      │      │      │      │     │
    //        │      │      │      │      │      │     │
    //        └──────┴──────┴──────┴──────┴──────┴─────┘
    //
    cherry_pick_region_1 = {EighteenZoneId::ZONE_8, EighteenZoneId::ZONE_9,
                            EighteenZoneId::ZONE_6};
    cherry_pick_region_2 = {EighteenZoneId::ZONE_7};

    // Otherwise, the pass is coming from the enemy end, put the two cherry-pickers
    // on the opposite side of the x-axis to wherever the pass is coming from
    EighteenZoneId zone_with_ball = pitch_division->getZoneId(ball_location);

    if (zone_with_ball == EighteenZoneId::ZONE_1 ||
        zone_with_ball == EighteenZoneId::ZONE_3)
    {
        cherry_pick_region_1 = {EighteenZoneId::ZONE_4, EighteenZoneId::ZONE_5};
        cherry_pick_region_2 = {EighteenZoneId::ZONE_8, EighteenZoneId::ZONE_9};
    }
    else if (zone_with_ball == EighteenZoneId::ZONE_4 ||
             zone_with_ball == EighteenZoneId::ZONE_5 ||
             zone_with_ball == EighteenZoneId::ZONE_6 ||
             zone_with_ball == EighteenZoneId::ZONE_7 ||
             zone_with_ball == EighteenZoneId::ZONE_8 ||
             zone_with_ball == EighteenZoneId::ZONE_9)
    {
        cherry_pick_region_1 = {EighteenZoneId::ZONE_10, EighteenZoneId::ZONE_13,
                                EighteenZoneId::ZONE_16};
        cherry_pick_region_2 = {EighteenZoneId::ZONE_12, EighteenZoneId::ZONE_15,
                                EighteenZoneId::ZONE_18};
    }
    else
    {
        cherry_pick_region_1 = {EighteenZoneId::ZONE_13, EighteenZoneId::ZONE_14,
                                EighteenZoneId::ZONE_16};
        cherry_pick_region_2 = {EighteenZoneId::ZONE_18, EighteenZoneId::ZONE_15};
    }
}
