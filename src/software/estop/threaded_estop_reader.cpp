
#include "threaded_estop_reader.h"

#include <boost/bind/bind.hpp>
#include <thread>
#include <utility>

#include "software/logger/logger.h"

ThreadedEstopReader::ThreadedEstopReader(std::unique_ptr<UartCommunication> uart_reader)
    : estop_state(EstopState::STOP),
      timer(io_service, boost::posix_time::milliseconds(INTERVAL_BETWEEN_READS_MS)),
      uart_reader(std::move(uart_reader))
{
    estop_thread = std::thread(boost::bind(&ThreadedEstopReader::continousRead, this));
}

void ThreadedEstopReader::continousRead()
{
    timer.async_wait(
        boost::bind(&ThreadedEstopReader::tick, this, boost::asio::placeholders::error));
    io_service.run();
}

bool ThreadedEstopReader::isEstopPlay()
{
    return estop_state == EstopState::PLAY;
}

void ThreadedEstopReader::tick(const boost::system::error_code& error)
{
    if (in_destructor)
    {
        timer.cancel();
    }
    else
    {
        std::vector<unsigned char> estop_msg;
        try
        {
            uart_reader->flushSerialPort(uart_reader->flush_receive);
            estop_msg = uart_reader->serialRead(ESTOP_MESSAGE_SIZE_BYTES);
        }
        catch (const std::exception& e)
        {
            LOG(FATAL)
                << "crashing system and timing out robots as we have lost connection to ESTOP source : "
                << e.what();
        }

        EstopState new_state;

        switch (static_cast<int>(estop_msg.at(0)))
        {
            case ESTOP_PLAY_MSG:
            {
                new_state                    = EstopState::PLAY;
                num_consecutive_status_error = 0;
                break;
            }
            case ESTOP_STOP_MSG:
            {
                new_state                    = EstopState::STOP;
                num_consecutive_status_error = 0;
                break;
            }
            default:
            {
                new_state = EstopState::STATUS_ERROR;
                LOG(WARNING) << "read unexpected estop message";
                num_consecutive_status_error++;
                break;
            }
        }

        if (new_state != estop_state)
        {
            LOG(INFO) << "ESTOP changed from " << estop_state << " to " << new_state;
            estop_state = new_state;
        }

        CHECK(num_consecutive_status_error <= MAXIMUM_CONSECUTIVE_STATUS_ERROR)
            << "ESTOP Consecutive Unexpected messages";

        boost::posix_time::milliseconds next_interval(INTERVAL_BETWEEN_READS_MS);

        // Reschedule the timer for interval seconds in the future:
        timer.expires_from_now(next_interval);
        // Posts the timer event
        timer.async_wait(boost::bind(&ThreadedEstopReader::tick, this,
                                     boost::asio::placeholders::error));
    }
}


ThreadedEstopReader::~ThreadedEstopReader()
{
    in_destructor = true;
    io_service.stop();

    // We must wait for the thread to stop, as if we destroy it while it's still
    // running we will segfault
    estop_thread.join();
}
