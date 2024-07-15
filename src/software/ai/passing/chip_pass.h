#pragma once

#include "software/ai/passing/base_pass.h"
#include "shared/constants.h"
#include "software/geom/algorithms/contains.h"

class ChipPass: public BasePass
{
    public:
    ChipPass() = delete;

    /**
     * Creates a chip pass from the given destination point
     *
     * @param passer_point the starting point of the pass
     * @param pass_destination the end point of the pass
     * @return the ChipPass constructed from the start and end points
     */
    ChipPass(Point passer_point, Point receiver_point);

    double firstBounceRange();

    bool isSkipped(const Point& point) const;

    virtual Duration estimatePassDuration() const;

    virtual Duration estimateTimeToPoint(Point& point) const;

    friend std::ostream& operator<<(std::ostream& output_stream, const ChipPass& pass);

    virtual bool operator==(const ChipPass& other) const;

    virtual PassType type() const;

    private:

    double calcFirstBounceRange();

    Polygon calcSkipArea();

    double getBounceHeightFromDistanceTraveled(double distance_traveled);

    double getBounceRangeFromBounceHeight(double bounce_height);

    double first_bounce_range_m;
    double pass_length;
    std::vector<std::pair<double, double>> bounce_heights_and_ranges;
    Polygon skip_area;
};