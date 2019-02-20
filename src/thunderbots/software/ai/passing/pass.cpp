// TODO: start of file comment

#include "pass.h"

using namespace AI::Passing;

//Pass::Pass()  :
//receiver_point(0, 0),
//passer_point(0, 0),
//pass_speed_m_per_s(0),
//pass_start_time(Timestamp::fromSeconds(0))
//{}

Pass::Pass(Point passer_point, Point receiver_point, double pass_speed_m_per_s, Timestamp pass_start_time):
receiver_point(receiver_point),
passer_point(passer_point),
pass_speed_m_per_s(pass_speed_m_per_s),
pass_start_time(pass_start_time)
{

}

Point & Pass::receiverPoint() {
    return receiver_point;
}

Point &Pass::passerPoint() {
    return passer_point;
}

double & Pass::passSpeed() {
    return pass_speed_m_per_s;
}

Timestamp & Pass::passStartTime() {
    return pass_start_time;
}

