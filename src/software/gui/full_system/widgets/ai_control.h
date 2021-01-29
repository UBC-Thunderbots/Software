#pragma once

#include <QtWidgets/QWidget>

#include "software/parameter/dynamic_parameters.h"

// This include is autogenerated by the .ui file in the same folder
// The generated version will be names 'ui_<filename>.h'
#include "software/gui/full_system/ui/ui_main_widget.h"

/**
 * Sets up all the widgets that will be used to control the AI, such as starting or
 * stopping it, or choosing the colour of the friendly team
 *
 * @param widget The widget containing the AI controls to set up
 * @param config The config to use to set up the parameters
 */
void setupAIControls(Ui::AutogeneratedFullSystemMainWidget *widget,
                     std::shared_ptr<ThunderbotsConfig> config);

/**
 * Sets up the buttons that will start and top the AI
 *
 * @param widget The widget containing the buttons to set up
 * @param run_ai_parameter The parameter that controls whether or not the AI is running
 */
void setupAIStartAndStopButtons(Ui::AutogeneratedFullSystemMainWidget *widget,
                                std::shared_ptr<Parameter<bool>> run_ai_parameter);

/**
 * Sets up the ComboBox widget that will be used to select the friendly team colour
 *
 * @param team_colour_combo_box The ComboBox to set up
 * @param friendly_colour_yellow_parameter The parameter that determines if the friendly
 * team is yellow or not
 */
void setupTeamColourComboBox(
    QComboBox *team_colour_combo_box,
    std::shared_ptr<Parameter<bool>> friendly_colour_yellow_parameter);

/**
 * Sets up the ComboBox widget that will be used to select the side the friendly team
 * is defending
 *
 * @param defending_side_combo_box The ComboBox to set up
 * @param defending_side_override_parameter The parameter that controls whether or not
 * we override the defending side
 * @param defending_positive_side_parameter The parameter that determines if the we
 * are defending the positive side of the field or not
 */
void setupDefendingSideComboBox(
    QComboBox *defending_side_combo_box,
    std::shared_ptr<Parameter<bool>> defending_side_override_parameter,
    std::shared_ptr<Parameter<bool>> defending_positive_side_parameter);

/**
 * Sets up the ComboBox widget that will be used to override the AI GameState
 *
 * @param gamestate_override_combo_box The ComboBox to set up
 * @param gamestate_override_parameter The parameter that controls whether or not
 * we override the gamestate
 * @param previous_game_state_parameter The parameter that holds the previous gamestate
 * @param current_game_state_parameter The parameter that holds the current gamestate
 */
void setupGameStateOverrideComboBox(
    QComboBox *gamestate_override_combo_box,
    std::shared_ptr<Parameter<bool>> gamestate_override_parameter,
    std::shared_ptr<Parameter<std::string>> previous_game_state_parameter,
    std::shared_ptr<Parameter<std::string>> current_game_state_parameter);

/**
 * Sets up the ComboBox widget that will be used to override the current AI Play
 *
 * @param play_override_combo_box The ComboBox to set up
 * @param play_override_parameter The parameter that controls whether or not
 * we override the AI Play
 * @param current_play_parameter The parameter that holds the current AI Play
 */
void setupPlayOverrideComboBox(
    QComboBox *play_override_combo_box,
    std::shared_ptr<Parameter<bool>> play_override_parameter,
    std::shared_ptr<Parameter<std::string>> current_play_parameter);

/**
 * Sets up the ComboBox widget that will be used to set the communication channel
 *
 * @param channel_spin_box The SpinBox to set up
 * @param channel_parameter The parameter that controls the communication channel
 */
void setupChannelSpinBox(QSpinBox *channel_spin_box,
                         std::shared_ptr<NumericParameter<int>> channel_parameter);

/**
 * Sets up a ComboBox widget that will be used to override either the
 * friendly or enemy goalie ID
 *
 * @param id_comboBox The ComboBox to set up
 * @param override_goalie_id_parameter the parameter that controls whether or not
 * we override the goalie id
 * @param goalie_id_parameter The parameter that controls the goalie id
 */
void setupGoalieIDComboBox(
    QComboBox *id_comboBox, std::shared_ptr<Parameter<bool>> override_goalie_id_parameter,
    std::shared_ptr<NumericParameter<int>> goalie_id_parameter);
