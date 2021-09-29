
#include "software/proto/message_translation/robot_log.h"

std::unique_ptr<TbotsProto::RobotLog> createRobotLog(g3::LogMessageMover log_entry, int robot_id){
    auto log_msg_proto = std::make_unique<TbotsProto::RobotLog>();

    log_msg_proto->set_log_msg(log_entry.get().message());
    log_msg_proto->set_robot_id(robot_id);
    log_msg_proto->set_log_level((*createLogLevel(log_entry.get().level())));
    log_msg_proto->set_file_name(log_entry.get().file());
    log_msg_proto->set_line_number(
            static_cast<uint32_t>(std::stoul(log_entry.get().line())));

    return log_msg_proto;
}

std::unique_ptr<TbotsProto::LogLevel> createLogLevel(std::string level){

    auto log_level_proto = TbotsProto::LogLevel();
    TbotsProto::LogLevel_Parse(level, &log_level_proto);
    return std::make_unique<TbotsProto::LogLevel>(log_level_proto);
}


