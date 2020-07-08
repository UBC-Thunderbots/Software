#include "software/gui/full_system/widgets/ai_control.h"

#include <QtWidgets/QComboBox>
#include <QtWidgets/QPushButton>

#include "software/ai/hl/stp/play/play.h"
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
        config->getMutableSensorFusionConfig()->mutableOverrideRefboxFriendlyTeamColor(),
        config->getMutableSensorFusionConfig()->mutableFriendlyColorYellow());
    setupDefendingSideComboBox(
        widget,
        config->getMutableSensorFusionConfig()->mutableOverrideRefboxDefendingSide(),
        config->getMutableSensorFusionConfig()->mutableDefendingPositiveSide());
    setupGameStateOverrideComboBox(
        widget, config->getMutableAIControlConfig()->mutableOverrideRefboxGameState(),
        config->getMutableAIControlConfig()->mutablePreviousRefboxGameState(),
        config->getMutableAIControlConfig()->mutableCurrentRefboxGameState());
    setupPlayOverrideComboBox(
        widget, config->getMutableAIControlConfig()->mutableOverrideAIPlay(),
        config->getMutableAIControlConfig()->mutableCurrentAIPlay());
    setupChannelSpinBox(widget, config->getMutableNetworkConfig()->mutableChannel());
    setupFriendlyGoalieIDSpinBox(
        widget, config->getMutableSensorFusionConfig()->mutableFriendlyGoalieId());
    setupEnemyGoalieIDSpinBox(
        widget, config->getMutableSensorFusionConfig()->mutableEnemyGoalieId());
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
    widget->team_colour_combo_box->insertItem(2, "Use Refbox");
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
            else if (text == "Use Refbox")
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
    widget->defending_side_combo_box->insertItem(0, "Use Refbox");
    widget->defending_side_combo_box->insertItem(1, "Positive");
    widget->defending_side_combo_box->insertItem(2, "Negative");

    auto on_defending_side_changed =
        [defending_side_override_parameter,
         defending_positive_side_parameter](const QString &text) {
            if (text == "Use Refbox")
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
    widget->gamestate_override_combo_box->addItem(QString::fromStdString("Use Refbox"));
    for (const auto &state : allValuesRefboxGameState())
    {
        widget->gamestate_override_combo_box->addItem(
            QString::fromStdString(toString(state)));
    }

    auto on_game_state_changed = [gamestate_override_parameter,
                                  previous_game_state_parameter,
                                  current_game_state_parameter](const QString &text) {
        if (text == "Use Refbox")
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

    auto on_parameter_value_changed = [spin_box](int new_value) {
        // We block signals while setting the value of the spinbox so that we don't
        // trigger the `on_spinbox_value_changed` function, which would set the
        // parameter value again and deadlock on the parameter's internal mutex
        spin_box->blockSignals(true);
        spin_box->setValue(new_value);
        spin_box->blockSignals(false);
    };
    channel_parameter->registerCallbackFunction(on_parameter_value_changed);
}

void setupFriendlyGoalieIDSpinBox(
    Ui::AutogeneratedFullSystemMainWidget *widget,
    std::shared_ptr<Parameter<int>> friendly_goalie_id_parameter)
{
    auto parameter = friendly_goalie_id_parameter;
    auto spin_box  = widget->friendly_goalie_id_spin_box;

    spin_box->setValue(parameter->value());
    spin_box->setMinimum(0);
    spin_box->setMaximum(std::numeric_limits<uint8_t>::max());

    auto on_channel_changed = [parameter](int new_value) {
        parameter->setValue(new_value);
    };

    // QSpinBox has 2 "valueChanged" signals that each provide different info (string vs
    // int), so we need to static_cast to specify the integer version
    QWidget::connect(spin_box,
                     static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     on_channel_changed);

    auto on_parameter_value_changed = [spin_box](int new_value) {
        // We block signals while setting the value of the spinbox so that we don't
        // trigger the `on_spinbox_value_changed` function, which would set the
        // parameter value again and deadlock on the parameter's internal mutex
        spin_box->blockSignals(true);
        spin_box->setValue(new_value);
        spin_box->blockSignals(false);
    };
    parameter->registerCallbackFunction(on_parameter_value_changed);
}

void setupEnemyGoalieIDSpinBox(Ui::AutogeneratedFullSystemMainWidget *widget,
                               std::shared_ptr<Parameter<int>> enemy_goalie_id_parameter)
{
    auto parameter = enemy_goalie_id_parameter;
    auto spin_box  = widget->enemy_goalie_id_spin_box;

    spin_box->setValue(parameter->value());
    spin_box->setMinimum(0);
    spin_box->setMaximum(std::numeric_limits<uint8_t>::max());

    auto on_channel_changed = [parameter](int new_value) {
        parameter->setValue(new_value);
    };

    // QSpinBox has 2 "valueChanged" signals that each provide different info (string vs
    // int), so we need to static_cast to specify the integer version
    QWidget::connect(spin_box,
                     static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                     on_channel_changed);

    auto on_parameter_value_changed = [spin_box](int new_value) {
        // We block signals while setting the value of the spinbox so that we don't
        // trigger the `on_spinbox_value_changed` function, which would set the
        // parameter value again and deadlock on the parameter's internal mutex
        spin_box->blockSignals(true);
        spin_box->setValue(new_value);
        spin_box->blockSignals(false);
    };
    parameter->registerCallbackFunction(on_parameter_value_changed);
}
