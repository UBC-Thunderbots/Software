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
 * @param widget The widget containing the ComboBox to set up
 * @param override_friendly_colour_parameter The parameter that controls whether or not
 * we override the friendly colour
 * @param friendly_colour_yellow_parameter The parameter that determines if the friendly
 * team is yellow or not
 */
void setupTeamColourComboBox(
    Ui::AutogeneratedFullSystemMainWidget *widget,
    std::shared_ptr<Parameter<bool>> override_friendly_colour_parameter,
    std::shared_ptr<Parameter<bool>> friendly_colour_yellow_parameter);

/**
 * Sets up the ComboBox widget that will be used to select the side the friendly team
 * is defending
 *
 * @param widget The widget containing the ComboBox to set up
 * @param defending_side_override_parameter The parameter that controls whether or not
 * we override the defending side
 * @param defending_positive_side_parameter The parameter that determines if the we
 * are defending the positive side of the field or not
 */
void setupDefendingSideComboBox(
    Ui::AutogeneratedFullSystemMainWidget *widget,
    std::shared_ptr<Parameter<bool>> defending_side_override_parameter,
    std::shared_ptr<Parameter<bool>> defending_positive_side_parameter);

/**
 * Sets up the ComboBox widget that will be used to override the AI GameState
 *
 * @param widget The widget containing the ComboBox to set up
 * @param gamestate_override_parameter The parameter that controls whether or not
 * we override the gamestate
 * @param previous_game_state_parameter The parameter that holds the previous gamestate
 * @param current_game_state_parameter The parameter that holds the current gamestate
 */
void setupGameStateOverrideComboBox(
    Ui::AutogeneratedFullSystemMainWidget *widget,
    std::shared_ptr<Parameter<bool>> gamestate_override_parameter,
    std::shared_ptr<Parameter<std::string>> previous_game_state_parameter,
    std::shared_ptr<Parameter<std::string>> current_game_state_parameter);

/**
 * Sets up the ComboBox widget that will be used to override the current AI Play
 *
 * @param widget The widget containing the ComboBox to set up
 * @param play_override_parameter The parameter that controls whether or not
 * we override the AI Play
 * @param current_play_parameter The parameter that holds the current AI Play
 */
void setupPlayOverrideComboBox(
    Ui::AutogeneratedFullSystemMainWidget *widget,
    std::shared_ptr<Parameter<bool>> play_override_parameter,
    std::shared_ptr<Parameter<std::string>> current_play_parameter);

/**
 * Sets up the ComboBox widget that will be used to set the communication channel
 *
 * @param widget The widget containing the SpinBox to set up
 * @param channel_parameter The parameter that controls the communication channel
 */
void setupChannelSpinBox(Ui::AutogeneratedFullSystemMainWidget *widget,
                         std::shared_ptr<Parameter<int>> channel_parameter);

/**
 * Sets up the ComboBox widget that will be used to override the friendly goalie ID
 *
 * @param widget The widget containing the ComboBox to set up
 * @param override_friendly_goalie_id_parameter the parameter that controls whether or not
 * we override the friendly goalie id
 * @param friendly_goalie_id_parameter The parameter that controls the friendly goalie id
 */
void setupFriendlyGoalieIDComboBox(
        Ui::AutogeneratedFullSystemMainWidget *widget,
        std::shared_ptr<Parameter<bool>> override_friendly_goalie_id_parameter,
        std::shared_ptr<Parameter<int>> friendly_goalie_id_parameter);

/**
 * Sets up the ComboBox widget that will be used to override the enemy goalie ID
 *
 * @param widget The widget containing the ComboBox to set up
 * @param override_enemy_goalie_id_parameter the parameter that controls whether or not
 * we override the enemy goalie id
 * @param enemy_goalie_id_parameter The parameter that controls the enemy goalie id
 */
void setupEnemyGoalieIDComboBox(
        Ui::AutogeneratedFullSystemMainWidget *widget,
        std::shared_ptr<Parameter<bool>> override_enemy_goalie_id_parameter,
        std::shared_ptr<Parameter<int>> enemy_goalie_id_parameter);

/**
 * Sets up a ComboBox widget that will be used to override either the
 * friendly or enemy goalie ID
 *
 * @param widget The widget containing the ComboBox to set up
 * @param override_goalie_id_parameter the parameter that controls whether or not
 * we override the goalie id
 * @param goalie_id_parameter The parameter that controls the goalie id
 * @param friendly_or_enemy The parameter that indicates which goalie id
 * combobox to set up
 */
void setupGoalieIDComboBox(
        QComboBox *cb,
        std::shared_ptr<Parameter<bool>> override_goalie_id_parameter,
        std::shared_ptr<Parameter<int>> goalie_id_parameter);
