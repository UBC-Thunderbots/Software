#pragma once

#include <QtWidgets/QWidget>

// This include is autogenerated by the .ui file in the same folder
// The generated version will be names 'ui_<filename>.h'
#include "software/gui/ui/ui_main_widget.h"

/**
 * Sets up all the widgets that will be used to control the AI, such as starting or
 * stopping it, or choosing the colour of the friendly team
 */
void setupAIControls(Ui::AutoGeneratedMainWidget *widget);

/**
 * Sets up the buttons that will start and top the AI
 */
void setupAIStartAndStopButtons(Ui::AutoGeneratedMainWidget *widget);

/**
 * Sets up the ComboBox widget that will be used to select the friendly team colour
 */
void setupTeamColourComboBox(Ui::AutoGeneratedMainWidget *widget);

/**
 * Sets up the ComboBox widget that will be used to select the side the friendly team
 * is defending
 */
void setupDefendingSideComboBox(Ui::AutoGeneratedMainWidget *widget);

/**
 * Sets up the ComboBox widget that will be used to override the AI GameState
 */
void setupGameStateOverrideComboBox(Ui::AutoGeneratedMainWidget *widget);

/**
 * Sets up the ComboBox widget that will be used to override the current AI Play
 */
void setupPlayOverrideComboBox(Ui::AutoGeneratedMainWidget *widget);
