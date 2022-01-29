#include "software/gui/generic_widgets/play_info/play_info_widget.h"

PlayInfoWidget::PlayInfoWidget(QWidget* parent) : QTextEdit(parent) {}

void PlayInfoWidget::updatePlayInfo(const TbotsProto::PlayInfo& play_info)
{
    QString referee_command_string =
        QString("Referee Command: %1\n")
            .arg(QString::fromStdString(play_info.game_state().referee_command_name()));
    QString play_name_string =
        QString("Play Name: %1\n")
            .arg(QString::fromStdString(play_info.play().play_name()));
    QString tactics_string = QString("Tactics:\n");
    std::vector<std::string> tactics;
    for (const auto& [robot_id, tactic] : play_info.robot_tactic_assignment())
    {
        tactics.emplace_back(std::to_string(robot_id)
                                 .append("- ")
                                 .append(tactic.tactic_name())
                                 .append(" - ")
                                 .append(tactic.tactic_fsm_state())
                                 .append("\n"));
    }
    std::sort(tactics.begin(), tactics.end());
    for (const auto& next_tactic : tactics)
    {
        tactics_string.append(QString::fromStdString(next_tactic));
    }

    QString play_info_string =
        QString("%1\n%2\n%3")
            .arg(referee_command_string, play_name_string, tactics_string);

    setText(play_info_string);
}
