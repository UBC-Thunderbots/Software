#pragma once

class SslSimulationProtocolNetworkModule
{
   public:
    explicit SslSimulationProtocolNetworkModule(
        int team_ip, int primitive_port, int feedback_port, int simulator_ip,
        int simulator_port,
        std::function<void(SSLProto::SSL_WrapperPacket)> received_vision_callback);

    // Sends robot commands via UDP
    void sendRobotCommand(SslSimulationProto::RobotCommand);

    // Register callback function when feedback is received
    void registerOnRobotFeedbackReceive(
        const std::function<void(SslSimulationProto::RobotFeedback)>& callback);


   private:
    int blue_team_port;
    int yellow_team_port;
};
