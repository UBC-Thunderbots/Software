#include "software/gui/full_system/widgets/ai_control.h"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QPushButton>

#include "software/ai/hl/stp/play/play.h"
#include "software/gui/shared/parameters_spinbox.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/util/design_patterns/generic_factory.h"

void setupAIControls(Ui::AutogeneratedFullSystemMainWidget *widget,
                     std::shared_ptr<ThunderbotsConfig> config)
{
    setupAIStartAndStopButtons(widget,
                               config->getMutableAIControlConfig()->mutableRunAI());
    setupTeamColourComboBox(
        widget->team_colour_combo_box,
        config->getMutableSensorFusionConfig()->mutableFriendlyColorYellow());
    setupDefendingSideComboBox(
        widget->defending_side_combo_box,
        config->getMutableSensorFusionConfig()
            ->mutableOverrideGameControllerDefendingSide(),
        config->getMutableSensorFusionConfig()->mutableDefendingPositiveSide());
    setupGameStateOverrideComboBox(
        widget->gamestate_override_combo_box,
        config->getMutableAIControlConfig()->mutableOverrideRefereeCommand(),
        config->getMutableAIControlConfig()->mutablePreviousRefereeCommand(),
        config->getMutableAIControlConfig()->mutableCurrentRefereeCommand());
    setupPlayOverrideComboBox(
        widget->play_override_combo_box,
        config->getMutableAIControlConfig()->mutableOverrideAIPlay(),
        config->getMutableAIControlConfig()->mutableCurrentAIPlay());
    setupChannelSpinBox(widget->channel_spin_box,
                        config->getMutableNetworkConfig()->mutableChannel());
    setupGoalieIDComboBox(
        widget->friendly_goalie_id_combo_box,
        config->getMutableSensorFusionConfig()
            ->mutableOverrideGameControllerFriendlyGoalieID(),
        config->getMutableSensorFusionConfig()->mutableFriendlyGoalieId());
    setupGoalieIDComboBox(widget->enemy_goalie_id_combo_box,
                          config->getMutableSensorFusionConfig()
                              ->mutableOverrideGameControllerEnemyGoalieID(),
                          config->getMutableSensorFusionConfig()->mutableEnemyGoalieId());
}

void setupAIStartAndStopButtons(Ui::AutogeneratedFullSystemMainWidget *widget,
                                std::shared_ptr<Parameter<bool>> run_ai_parameter)
{
    auto start_ai_func = [run_ai_parameter]() { run_ai_parameter->setValue(true); };
    QWidget::connect(widget->start_ai_button, &QPushButton::clicked, start_ai_func);
    auto stop_ai_func = [run_ai_parameter]() { run_ai_parameter->setValue(false); };
    QWidget::connect(widget->stop_ai_button, &QPushButton::clicked, stop_ai_func);
}

void setupTeamColourComboBox(
    QComboBox *team_colour_combo_box,
    std::shared_ptr<Parameter<bool>> friendly_colour_yellow_parameter)
{
    team_colour_combo_box->insertItem(0, "Yellow");
    team_colour_combo_box->insertItem(1, "Blue");
    auto on_team_colour_changed =
        [friendly_colour_yellow_parameter](const QString &text) {
            if (text == "Yellow")
            {
                friendly_colour_yellow_parameter->setValue(true);
            }
            else if (text == "Blue")
            {
                friendly_colour_yellow_parameter->setValue(false);
            }
            else
            {
                LOG(FATAL) << "Tried to set the team colour with an invalid value: '"
                           << text.toStdString() << "'" << std::endl;
            }
        };
    QWidget::connect(team_colour_combo_box, &QComboBox::currentTextChanged,
                     on_team_colour_changed);
}

void setupDefendingSideComboBox(
    QComboBox *defending_side_combo_box,
    std::shared_ptr<Parameter<bool>> defending_side_override_parameter,
    std::shared_ptr<Parameter<bool>> defending_positive_side_parameter)
{
    // See issue #811 for getting these value from an enum / factory
    defending_side_combo_box->insertItem(0, "Use GameController");
    defending_side_combo_box->insertItem(1, "Positive");
    defending_side_combo_box->insertItem(2, "Negative");
    // TODO: this value is hardcoded and it should take the default value from dynamic
    // parameters once https://github.com/UBC-Thunderbots/Software/issues/1299 is done and
    // integrated
    defending_side_combo_box->setCurrentText(QString::fromStdString("Negative"));

    auto on_defending_side_changed =
        [defending_side_override_parameter,
         defending_positive_side_parameter](const QString &text) {
            if (text == "Use GameController")
            {
                defending_side_override_parameter->setValue(false);
            }
            else if (text == "Positive")
            {
                defending_side_override_parameter->setValue(true);
                defending_positive_side_parameter->setValue(true);
            }
            else if (text == "Negative")
            {
                defending_side_override_parameter->setValue(true);
                defending_positive_side_parameter->setValue(false);
            }
            else
            {
                LOG(FATAL) << "Tried to set the defending side with an invalid value: '"
                           << text.toStdString() << "'" << std::endl;
            }
        };
    QWidget::connect(defending_side_combo_box, &QComboBox::currentTextChanged,
                     on_defending_side_changed);
}

void setupGameStateOverrideComboBox(
    QComboBox *gamestate_override_combo_box,
    std::shared_ptr<Parameter<bool>> gamestate_override_parameter,
    std::shared_ptr<Parameter<std::string>> previous_game_state_parameter,
    std::shared_ptr<Parameter<std::string>> current_game_state_parameter)
{
    gamestate_override_combo_box->addItem(QString::fromStdString("Use GameController"));
    for (const auto &state : allValuesRefereeCommand())
    {
        gamestate_override_combo_box->addItem(QString::fromStdString(toString(state)));
    }
    // TODO: this value is hardcoded and it should take the default value from dynamic
    // parameters once https://github.com/UBC-Thunderbots/Software/issues/1299 is done and
    // integrated
    gamestate_override_combo_box->setCurrentText(
        QString::fromStdString(toString(RefereeCommand::HALT)));

    auto on_game_state_changed = [gamestate_override_parameter,
                                  previous_game_state_parameter,
                                  current_game_state_parameter](const QString &text) {
        if (text == "Use GameController")
        {
            gamestate_override_parameter->setValue(false);
        }
        else
        {
            gamestate_override_parameter->setValue(true);
            previous_game_state_parameter->setValue(
                current_game_state_parameter->value());
            current_game_state_parameter->setValue(text.toStdString());
        }
    };
    QWidget::connect(gamestate_override_combo_box, &QComboBox::currentTextChanged,
                     on_game_state_changed);
}

void setupPlayOverrideComboBox(
    QComboBox *play_override_combo_box,
    std::shared_ptr<Parameter<bool>> play_override_parameter,
    std::shared_ptr<Parameter<std::string>> current_play_parameter)
{
    auto play_names = GenericFactory<std::string, Play>::getRegisteredNames();

    // Sort the entries in alphabetical order from a-z
    std::sort(play_names.begin(), play_names.end());

    // Create a new list with all the play names converted to QStrings
    QList<QString> qt_play_names;
    std::transform(play_names.begin(), play_names.end(),
                   std::back_inserter(qt_play_names),
                   [](std::string name) { return QString::fromStdString(name); });

    play_override_combo_box->insertItem(0, "Use AI Selection");
    play_override_combo_box->insertItems(1, qt_play_names);
    // TODO: this value is hardcoded and it should take the default value from dynamic
    // parameters once https://github.com/UBC-Thunderbots/Software/issues/1299 is done and
    // integrated
    play_override_combo_box->setCurrentText(QString::fromStdString("HaltPlay"));

    auto on_play_changed = [play_override_parameter,
                            current_play_parameter](const QString &text) {
        if (text == "Use AI Selection")
        {
            play_override_parameter->setValue(false);
        }
        else
        {
            play_override_parameter->setValue(true);
            current_play_parameter->setValue(text.toStdString());
        }
    };
    QWidget::connect(play_override_combo_box, &QComboBox::currentTextChanged,
                     on_play_changed);
}

void setupChannelSpinBox(QSpinBox *channel_spin_box,
                         std::shared_ptr<Parameter<int>> channel_parameter)
{
    channel_spin_box->setValue(channel_parameter->value());
    channel_spin_box->setMinimum(0);
    channel_spin_box->setMaximum(MAX_MULTICAST_CHANNELS);

    auto on_channel_changed = [channel_parameter](int new_value) {
        channel_parameter->setValue(new_value);
    };

    // QSpinBox has 2 "valueChanged" signals that each provide different info (string vs
    // int), so we need to static_cast to specify the integer version
    QWidget::connect(channel_spin_box,
                     static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     on_channel_changed);

    setupSpinBox(channel_spin_box, channel_parameter);
}

void setupGoalieIDComboBox(QComboBox *id_comboBox,
                           std::shared_ptr<Parameter<bool>> override_goalie_id_parameter,
                           std::shared_ptr<Parameter<int>> goalie_id_parameter)
{
    QList<QString> robot_ids;
    for (unsigned int id = 0; id < MAX_ROBOT_IDS; id++)
    {
        QString str = QString::number(id);
        robot_ids.insert(id, str);
    }
    id_comboBox->insertItem(0, "Use GameController");
    id_comboBox->insertItems(1, (robot_ids));
    // TODO: this value is hardcoded and it should take the default value from dynamic
    // parameters once https://github.com/UBC-Thunderbots/Software/issues/1299 is done and
    // integrated
    id_comboBox->setCurrentText(QString::fromStdString("0"));

    auto on_goalie_id_changed = [override_goalie_id_parameter, goalie_id_parameter,
                                 robot_ids](const QString &text) {
        if (text == "Use GameController")
        {
            override_goalie_id_parameter->setValue(false);
        }
        else if (robot_ids.contains(text))
        {
            override_goalie_id_parameter->setValue(true);
            goalie_id_parameter->setValue(text.toInt());
        }
        else
        {
            LOG(FATAL) << "Tried to set the goalie ID with an invalid value: '"
                       << text.toStdString() << "'" << std::endl;
        }
    };

    QWidget::connect(id_comboBox, &QComboBox::currentTextChanged, on_goalie_id_changed);
}
