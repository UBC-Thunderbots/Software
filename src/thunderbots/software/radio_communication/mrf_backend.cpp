#include "mrf_backend.h"

#include <chrono>

MRFBackend::MRFBackend()
    : dongle(MRFDongle()),
      ball(Ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(1)))
{
}

MRFBackend::~MRFBackend() {}

void MRFBackend::sendPrimitives(const std::vector<std::unique_ptr<Primitive>>& primitives)
{
    dongle.send_drive_packet(primitives);
}

void MRFBackend::update_detbots(std::vector<std::tuple<uint8_t, Point, Angle>> ft)
{
    detbots = ft;
}

void MRFBackend::update_ball(Ball b)
{
    ball = b;
}

void MRFBackend::send_vision_packet()
{
    /* TODO: Change handling of timestamp depending on age of team vs ball */
    uint64_t timestamp = static_cast<uint64_t>(ball.lastUpdateTimestamp().getSeconds());
    std::cout << "Calling dongle.send_camera_packet with: ";
    for (std::size_t i = 0; i < detbots.size(); ++i)
    {
        std::cout << "bot number = " << unsigned(std::get<0>(detbots[i])) << ", ";
        std::cout << "x = " << (std::get<1>(detbots[i])).x() << ", ";
        std::cout << "y = " << (std::get<1>(detbots[i])).y() << ", ";
        std::cout << "time capture = " << timestamp << ", ";
        std::cout << "theta = " << (std::get<2>(detbots[i])).toDegrees() << std::endl;
    }
    std::cout << "ball x = " << ball.position().x() << ", ";
    std::cout << "ball y = " << ball.position().y() << std::endl;

    dongle.send_camera_packet(detbots, ball.position() * 1000, timestamp);
}

void MRFBackend::update_dongle_events()
{
    dongle.handle_libusb_events();
}
