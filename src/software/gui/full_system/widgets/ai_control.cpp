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
        widget,
        config->getMutableSensorFusionConfig()
            ->mutableOverrideGameControllerFriendlyTeamColor(),
        config->getMutableSensorFusionConfig()->mutableFriendlyColorYellow());
    setupDefendingSideComboBox(
        widget,
        config->getMutableSensorFusionConfig()
            ->mutableOverrideGameControllerDefendingSide(),
        config->getMutableSensorFusionConfig()->mutableDefendingPositiveSide());
    setupGameStateOverrideComboBox(
        widget, config->getMutableAIControlConfig()->mutableOverrideRefereeCommand(),
        config->getMutableAIControlConfig()->mutablePreviousRefereeCommand(),
        config->getMutableAIControlConfig()->mutableCurrentRefereeCommand());
    setupPlayOverrideComboBox(
        widget, config->getMutableAIControlConfig()->mutableOverrideAIPlay(),
        config->getMutableAIControlConfig()->mutableCurrentAIPlay());
    setupChannelSpinBox(widget, config->getMutableNetworkConfig()->mutableChannel());
    setupFriendlyGoalieIDComboBox(
        widget, config->getMutableSensorFusionConfig()->mutableOverrideGameControllerFriendlyGoalieID(),
        config->getMutableSensorFusionConfig()->mutableFriendlyGoalieId());
    setupEnemyGoalieIDComboBox(
        widget, config->getMutableSensorFusionConfig()->mutableOverrideGameControllerEnemyGoalieID(),
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
    Ui::AutogeneratedFullSystemMainWidget *widget,
    std::shared_ptr<Parameter<bool>> override_friendly_colour_parameter,
    std::shared_ptr<Parameter<bool>> friendly_colour_yellow_parameter)
{
    widget->team_colour_combo_box->insertItem(0, "Yellow");
    widget->team_colour_combo_box->insertItem(1, "Blue");
    widget->team_colour_combo_box->insertItem(2, "Use GameController");
    auto on_team_colour_changed =
        [override_friendly_colour_parameter,
         friendly_colour_yellow_parameter](const QString &text) {
            if (text == "Yellow")
            {
                override_friendly_colour_parameter->setValue(true);
                friendly_colour_yellow_parameter->setValue(true);
            }
            else if (text == "Blue")
            {
                override_friendly_colour_parameter->setValue(true);
                friendly_colour_yellow_parameter->setValue(false);
            }
            else if (text == "Use GameController")
            {
                override_friendly_colour_parameter->setValue(false);
            }
            else
            {
                LOG(FATAL) << "Tried to set the team colour with an invalid value: '"
                           << text.toStdString() << "'" << std::endl;
            }
        };
    QWidget::connect(widget->team_colour_combo_box, &QComboBox::currentTextChanged,
                     on_team_colour_changed);
}

void setupDefendingSideComboBox(
    Ui::AutogeneratedFullSystemMainWidget *widget,
    std::shared_ptr<Parameter<bool>> defending_side_override_parameter,
    std::shared_ptr<Parameter<bool>> defending_positive_side_parameter)
{
    // See issue #811 for getting these value from an enum / factory
    widget->defending_side_combo_box->insertItem(0, "Use GameController");
    widget->defending_side_combo_box->insertItem(1, "Positive");
    widget->defending_side_combo_box->insertItem(2, "Negative");

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
    QWidget::connect(widget->defending_side_combo_box, &QComboBox::currentTextChanged,
                     on_defending_side_changed);
}

void setupGameStateOverrideComboBox(
    Ui::AutogeneratedFullSystemMainWidget *widget,
    std::shared_ptr<Parameter<bool>> gamestate_override_parameter,
    std::shared_ptr<Parameter<std::string>> previous_game_state_parameter,
    std::shared_ptr<Parameter<std::string>> current_game_state_parameter)
{
    widget->gamestate_override_combo_box->addItem(
        QString::fromStdString("Use GameController"));
    for (const auto &state : allValuesRefereeCommand())
    {
        widget->gamestate_override_combo_box->addItem(
            QString::fromStdString(toString(state)));
    }

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
    QWidget::connect(widget->gamestate_override_combo_box, &QComboBox::currentTextChanged,
                     on_game_state_changed);
}

void setupPlayOverrideComboBox(
    Ui::AutogeneratedFullSystemMainWidget *widget,
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

    widget->play_override_combo_box->insertItem(0, "Use AI Selection");
    widget->play_override_combo_box->insertItems(1, qt_play_names);

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
    QWidget::connect(widget->play_override_combo_box, &QComboBox::currentTextChanged,
                     on_play_changed);
}

void setupChannelSpinBox(Ui::AutogeneratedFullSystemMainWidget *widget,
                         std::shared_ptr<Parameter<int>> channel_parameter)
{
    auto spin_box = widget->channel_spin_box;

    spin_box->setValue(channel_parameter->value());
    spin_box->setMinimum(0);
    spin_box->setMaximum(MAX_MULTICAST_CHANNELS);

    auto on_channel_changed = [channel_parameter](int new_value) {
        channel_parameter->setValue(new_value);
    };

    // QSpinBox has 2 "valueChanged" signals that each provide different info (string vs
    // int), so we need to static_cast to specify the integer version
    QWidget::connect(spin_box,
                     static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     on_channel_changed);

    setupSpinBox(spin_box, channel_parameter);
}

void setupFriendlyGoalieIDComboBox(
        Ui::AutogeneratedFullSystemMainWidget *widget,
        std::shared_ptr<Parameter<bool>> override_friendly_goalie_id_parameter,
        std::shared_ptr<Parameter<int>> friendly_goalie_id_parameter)
{
    setupGoalieIDComboBox(widget, override_friendly_goalie_id_parameter, friendly_goalie_id_parameter, "friendly");
}

void setupEnemyGoalieIDComboBox(
        Ui::AutogeneratedFullSystemMainWidget *widget,
        std::shared_ptr<Parameter<bool>> override_enemy_goalie_id_parameter,
        std::shared_ptr<Parameter<int>> enemy_goalie_id_parameter
        )
{
    setupGoalieIDComboBox(widget, override_enemy_goalie_id_parameter, enemy_goalie_id_parameter, "enemy");

}

void setupGoalieIDComboBox(
        Ui::AutogeneratedFullSystemMainWidget *widget,
        std::shared_ptr<Parameter<bool>> override_goalie_id_parameter,
        std::shared_ptr<Parameter<int>> goalie_id_parameter,
        std::string friendly_or_enemy
        )
{
    QList<QString> robot_ids;
    for (int id = 0; id < MAX_ROBOT_IDS; id++)
    {
        QString str = QString::number(id);
        robot_ids.insert(id, str);
    }

    if (friendly_or_enemy == "friendly") {
        widget->friendly_goalie_id_combo_box->insertItem(0, "Use GameController");
        widget->friendly_goalie_id_combo_box->insertItems(1, (robot_ids));
    }
    if (friendly_or_enemy == "enemy") {
        widget->enemy_goalie_id_combo_box->insertItem(0, "Use GameController");
        widget->enemy_goalie_id_combo_box->insertItems(1, (robot_ids));
    }

    auto on_goalie_id_changed =
            [override_goalie_id_parameter,
                    goalie_id_parameter](const QString &text) {

                bool override = false;
                int goalie_id;
                for (int id = 0; id < MAX_ROBOT_IDS; id++)
                {
                    if (text == id)
                    {
                        override = true;
                        goalie_id = id;
                    }
                }

                if (override)
                {
                    override_goalie_id_parameter->setValue(true);
                    goalie_id_parameter->setValue(goalie_id);
                }
                else if (text == "Use GameController")
                {
                    override_goalie_id_parameter->setValue(false);
                }
                else
                {
                        LOG(FATAL) << "Tried to set the goalie ID with an invalid value: '"
                                   << text.toStdString() << "'" << std::endl;
                }
            };
    if (friendly_or_enemy == "friendly") {
        QWidget::connect(widget->friendly_goalie_id_combo_box, &QComboBox::currentTextChanged,
                         on_goalie_id_changed);
    }
    if (friendly_or_enemy == "enemy") {
        QWidget::connect(widget->enemy_goalie_id_combo_box, &QComboBox::currentTextChanged,
                         on_goalie_id_changed);
    }
}
