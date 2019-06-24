#include "mrf_backend.h"

#include <chrono>

#include "util/logger/init.h"

MRFBackend::MRFBackend(unsigned int config, ros::NodeHandle &node_handle)
    : annunciator(Annunciator(node_handle)), dongle(MRFDongle(config, annunciator))
{
}

MRFBackend::~MRFBackend() {}

void MRFBackend::sendPrimitives(const std::vector<std::unique_ptr<Primitive>> &primitives)
{
    dongle.send_drive_packet(primitives);
}

void MRFBackend::send_vision_packet(std::vector<std::tuple<uint8_t, Point, Angle>> robots,
                                    Ball ball)
{
    uint64_t timestamp = static_cast<uint64_t>(ball.lastUpdateTimestamp().getSeconds());
    dongle.send_camera_packet(robots, ball.position() * MILLIMETERS_PER_METER, timestamp);
}
