#include "software/gui/generic_widgets/play_info/play_info_widget.h"

#include <algorithm>

PlayInfoWidget::PlayInfoWidget(QWidget* parent) : QTextEdit(parent) {}

void PlayInfoWidget::updatePlayInfo(const PlayInfo& play_info)
{
    QString referee_command_string =
        QString("Referee Command: %1")
            .arg(QString::fromStdString(play_info.getRefereeCommandName()));
    QString play_name_string =
        QString("Play Name: %1").arg(QString::fromStdString(play_info.getPlayName()));
    QString tactics_string = QString("Tactics:\n");

auto asst = play_info.getRobotTacticAssignment();
std::sort(asst.begin(), asst.end());
    for (const auto& tactic_string : asst)
    {
        tactics_string.append(QString::fromStdString(tactic_string)).append("\n");
    }

    QString play_info_string =
        QString("%1\n%2\n%3")
            .arg(referee_command_string, play_name_string, tactics_string);

    setText(play_info_string);
}
