#pragma once

class SslSimulationProtocolNetworkModule {
  public:
    // Sends robot commands via UDP
    void sendBlueRobotCommand(SslSimulationProto::RobotCommand);
    void sendYellowRobotCommand(SslSimulationProto::RobotCommand);

    // Register callback function when feedback is received
    void registerOnBlueRobotFeedbackReceive(
        const std::function<void(SslSimulationProto::RobotFeedback)>& callback);
    void registerOnYellowRobotFeedbackReceive(
        const std::function<void(SslSimulationProto::RobotFeedback)>& callback);

    // Send and receive simulation control
    void sendSimulationControlCommand(SslSimulationProto::SimulatorCommand)
    void registerOnSimulatorResponseReceive(
        const std::function<void(SslSimulationProto::RobotFeedback)>& callback);

  private:
    int blue_team_port;
    int yellow_team_port;
}