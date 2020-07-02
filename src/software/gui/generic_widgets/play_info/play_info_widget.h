#pragma once

#include <QtWidgets/QWidget>
#include <QtWidgets/QTextEdit>
#include "software/ai/hl/stp/play_info.h"

class PlayInfoWidget : public QTextEdit {
    Q_OBJECT

public:
    PlayInfoWidget(QWidget* parent = 0);

    void updatePlayInfo(const PlayInfo& play_info);
};