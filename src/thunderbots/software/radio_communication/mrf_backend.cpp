#include "mrf_backend.h"

#include <chrono>

#include "util/logger/init.h"

MRFBackend::MRFBackend()
    : dongle(MRFDongle()),
      ball(Ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0)))
{
}

MRFBackend::~MRFBackend() {}

void MRFBackend::sendPrimitives(const std::vector<std::unique_ptr<Primitive>>& primitives)
{
    dongle.send_drive_packet(primitives);
}

void MRFBackend::update_robots(std::vector<std::tuple<uint8_t, Point, Angle>> robots)
{
    this->robots = robots;
}

void MRFBackend::update_ball(Ball b)
{
    ball = b;
}

void MRFBackend::send_vision_packet()
{
    uint64_t timestamp = static_cast<uint64_t>(ball.lastUpdateTimestamp().getSeconds());

    LOG(DEBUG) << "Calling dongle.send_camera_packet with: ";
    for (std::size_t i = 0; i < robots.size(); ++i)
    {
        LOG(DEBUG) << "bot number = " << unsigned(std::get<0>(robots[i])) << ", ";
        LOG(DEBUG) << "x = " << (std::get<1>(robots[i])).x() << ", ";
        LOG(DEBUG) << "y = " << (std::get<1>(robots[i])).y() << ", ";
        LOG(DEBUG) << "time capture = " << timestamp << ", ";
        LOG(DEBUG) << "theta = " << (std::get<2>(robots[i])).toDegrees() << std::endl;
    }
    LOG(DEBUG) << "ball x = " << ball.position().x() * MILLIMETERS_PER_METER << ", ";
    LOG(DEBUG) << "ball y = " << ball.position().y() * MILLIMETERS_PER_METER << std::endl;

    dongle.send_camera_packet(robots, ball.position() * MILLIMETERS_PER_METER, timestamp);
}

void MRFBackend::update_dongle_events()
{
    dongle.handle_libusb_events();
}
