#pragma once

#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>

#include "software/ai/hl/stp/play_info.h"

/**
 * This class abstracts how we display PlayInfo in the GUI
 */
class PlayInfoWidget : public QTextEdit
{
    Q_OBJECT

   public:
    PlayInfoWidget(QWidget* parent = nullptr);

    /**
     * Updates the PlayInfo being displayed by this widget
     *
     * @param play_info The new PlayInfo to display
     */
    void updatePlayInfo(const PlayInfo& play_info);
};
