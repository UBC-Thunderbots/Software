#include "software/gui/generic_widgets/play_info/play_info_widget.h"

PlayInfoWidget::PlayInfoWidget(QWidget* parent) : QTextEdit(parent) {}

void PlayInfoWidget::updatePlayInfo(const PlayInfoProto& play_info)
{
    QString referee_command_string =
        QString("Referee Command: %1\n")
            .arg(QString::fromStdString(play_info.game_state().referee_command_name()));
    QString play_name_string =
        QString("Play Name: %1\n")
            .arg(QString::fromStdString(play_info.play().play_name()));
    QString tactics_string = QString("Tactics:\n");
    for (const auto& tactic_string : play_info.robot_tactic_assignment())
    {
        tactics_string.append(
            QString::fromStdString(std::to_string(tactic_string.first))
                .append("- ")
                .append(QString::fromStdString(tactic_string.second.tactic_name()))
                .append("\n"));
    }

    QString play_info_string =
        QString("%1\n%2\n%3")
            .arg(referee_command_string, play_name_string, tactics_string);

    setText(play_info_string);
}
