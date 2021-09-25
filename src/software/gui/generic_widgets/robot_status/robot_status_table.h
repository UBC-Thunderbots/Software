#pragma once

#include <QtCore/QTimer>
#include <QtWidgets/QTableWidget>
#include <chrono>

#include "proto/robot_status_msg.pb.h"
#include "software/time/duration.h"

/**
 * A custom TableWidget designed to display robot status. This class manages the ordering
 * and age of the provided statuses while displaying them
 */
class RobotStatusTable : public QTableWidget
{
    Q_OBJECT
   public:
    /**
     * Creates a new RobotStatusTable
     *
     * NOTE: The reason the message_expiry_age is the second parameter with a default
     * argument is because this RobotStatusTable is being used as a "promoted custom
     * widget" in the qtcreator editor (which is what makes the .ui file that
     * autogenerates GUI code). You cannot define extra arguments to be passed to the
     * generated code, so the promoted widget will always be constructed with the
     * parent_ptr as the only parameter. Therefore the expiry age has to be the second
     * parameter to not interfere with the parent argument, and the default value is used
     * so our code can define what the value is in a somewhat reasonable way (slightly
     * better than hardcoding inside the implementation).
     *
     * @param parent A pointer to the parent widget for this RobotStatusTable
     * @param message_expiry_age How old messages must be before they are automatically
     * removed from the table
     */
    explicit RobotStatusTable(QWidget* parent             = nullptr,
                              Duration message_expiry_age = Duration::fromSeconds(30));

    /**
     * Returns a map of the status messages and ages stored in this table
     *
     * @return a map of the status messages and ages stored in this table
     */
    std::map<std::string, Duration> getStatusMessages() const;

   public slots:
    /**
     * Updates the table with a new robot status. If the status message already exists,
     * the message age is refreshed to 0. Otherwise, a new message is added with an age of
     * 0.
     *
     * @param robot_status_msg The status to add to the table
     */
    void updateRobotStatus(const TbotsProto::RobotStatus& robot_status_msg);

   private:
    /**
     * Updates the ages for all status messages in the table
     */
    void updateStatusAge();

    /**
     * Removes any status messages from the table whose age is greater than the expiry age
     */
    void removeOldStatusMessages();

    /**
     * Updates status_messages with a new status message string. If the status message
     * already exists, the message age is refreshed to 0. Otherwise, a new message is
     * added with an age of 0.
     *
     * @param robot_status The status to add to the table
     */
    void updateStatusMessageString(const std::string& message);

    /**
     * Updates the table view (the actual rendered table the user sees) with the latest
     * values stored in the status_messages
     */
    void updateTableView();

    std::map<std::string, Duration> status_messages;
    // The timestamp for when message ages were last updated
    std::chrono::time_point<std::chrono::steady_clock> last_age_update_timestamp;
    // How old messages must be before they are automatically removed from the table
    Duration message_expiry_age;
    QTimer age_update_timer;
};
