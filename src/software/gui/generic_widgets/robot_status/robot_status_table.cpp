#include "software/gui/generic_widgets/robot_status/robot_status_table.h"

#include <math.h>

RobotStatusTable::RobotStatusTable(QWidget* parent, Duration message_expiry_age)
    : QTableWidget(parent), message_expiry_age(message_expiry_age), age_update_timer(this)
{
    connect(&age_update_timer, &QTimer::timeout, this,
            &RobotStatusTable::updateStatusAge);
    connect(&age_update_timer, &QTimer::timeout, this,
            &RobotStatusTable::removeOldStatusMessages);
    unsigned int timer_interval_milliseconds = 100;
    age_update_timer.start(timer_interval_milliseconds);
}

void RobotStatusTable::updateTbotsRobotMsg(const TbotsRobotMsg& tbots_robot_msg)
{
    for (const auto& error_code : tbots_robot_msg.error_code())
    {
        std::string message =
            ErrorCode_descriptor()->FindValueByNumber(error_code)->name();
        updateStatusMessageString(message);
    }
    updateTableView();
}

void RobotStatusTable::updateRobotStatus(const RobotStatus& robot_status)
{
    for (const auto& message : robot_status.robot_messages)
    {
        updateStatusMessageString(message);
    }
    updateTableView();
}

void RobotStatusTable::updateStatusMessageString(const std::string& message)
{
    auto iter = status_messages.find(message);
    if (iter == status_messages.end())
    {
        status_messages.insert(std::make_pair(message, Duration::fromSeconds(0)));
    }
    else
    {
        iter->second = Duration::fromMilliseconds(0);
    }
}

void RobotStatusTable::updateStatusAge()
{
    auto current_timestamp = std::chrono::steady_clock::now();
    auto milliseconds_since_last_update =
        std::chrono::duration_cast<std::chrono::milliseconds>(current_timestamp -
                                                              last_age_update_timestamp);
    last_age_update_timestamp = current_timestamp;

    for (auto iter = status_messages.begin(); iter != status_messages.end(); iter++)
    {
        iter->second = iter->second +
                       Duration::fromMilliseconds(milliseconds_since_last_update.count());
    }
    updateTableView();
}

void RobotStatusTable::removeOldStatusMessages()
{
    for (auto iter = status_messages.begin(); iter != status_messages.end();
         /* no increment */)
    {
        auto age = iter->second;
        if (age > message_expiry_age)
        {
            status_messages.erase(iter);
        }
        else
        {
            iter++;
        }
    }
    updateTableView();
}

std::map<std::string, Duration> RobotStatusTable::getStatusMessages() const
{
    return status_messages;
}

void RobotStatusTable::updateTableView()
{
    // Resize the number of rows to only have as many rows as we have messages. This will
    // automatically delete any extra rows / messages for us, and then we overwrite the
    // existing rows with new messages
    setRowCount(status_messages.size());
    int row = 0;
    for (const auto& status_message : status_messages)
    {
        auto [message, age_duration] = status_message;
        QString age = QString::number(std::floor(age_duration.getSeconds()));
        setItem(row, 0, new QTableWidgetItem(age));
        setItem(row, 1, new QTableWidgetItem(QString::fromStdString(message)));
        row++;
    }
}
