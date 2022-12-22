from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.Qt.QtWidgets import *


def create_button(text: list):
    """Creates QPushButton objects inside a QGroupBox object.
    The default color of button will be white with black background.

    :param text: type:list - list of text for all buttons
    :return group_box, buttons:
            QGroupBox object - add this to the widget
            list of QPushButton objects - use this to perform tasks on the buttons

    """
    group_box = QGroupBox()
    num_buttons = len(text)
    buttons = []

    for i in range(num_buttons):
        button = QPushButton(text[i])
        buttons.append(button)

    hbox = QHBoxLayout()

    for button in buttons:
        hbox.addWidget(button)
    group_box.setLayout(hbox)

    return group_box, buttons


def create_radio(text: list, radio_group):
    """Creates QRadioButton objects inside a QGroupBox object.
    The default color of button background will be white.

    :param text: - list of text for all buttons
    :param radio_group: QButtonGroup to add these buttons to
    :return group_box, buttons: 
                QGroupBox object - add this to the widget
                list of QRadioButton object - use this to perform tasks on the buttons
    """
    group_box = QGroupBox()
    num_buttons = len(text)
    radios = []

    for i in range(num_buttons):
        radio = QRadioButton(text[i])
        # this is so that the button is properly visible in black background
        radio_group.addButton(radio)
        radios.append(radio)

    hbox = QHBoxLayout()

    for radio in radios:
        hbox.addWidget(radio)
    group_box.setLayout(hbox)

    return group_box, radios


def create_slider(text, min_val, max_val, tick_spacing):
    """Creates a QSlider object inside a QGroupBox object, along with a value
    label on the right. The slider orientation will be horizontal.

    :param text: text to display above the slider
    :param min_val: lowest value of the slider
    :param max_val: highest value of the slider
    :param tick_spacing: interval between two ticks on the slider
    :return vbox, slider, value_label:
            QVBoxLayout object - add this to the widget
            QSlider object - use this to perform tasks on the button
            displays value of slider, update this when value is changed
    """
    slider = QSlider(Qt.Orientation.Horizontal)
    slider.setMinimum(min_val)
    slider.setMaximum(max_val)
    slider.setTickPosition(QSlider.TickPosition.NoTicks)
    slider.setTickInterval(tick_spacing)

    value_label = QLabel(str(slider.value()))
    slider_label = QLabel(str(text))

    vbox = QVBoxLayout()
    vbox.addWidget(slider_label)
    vbox.addWidget(value_label)
    vbox.addWidget(slider)

    return vbox, slider, value_label


def create_push_button(title):
    """Create a push button

    :param title: the name of the button

    """
    push_button = QPushButton(title)
    push_button.setFixedWidth(150)

    return push_button


def change_button_state(button, enable):
    """Change button color and clickable state.

    :param button: button to change the state of
    :param enable: bool: if True: enable this button, if False: disable
    :returns: None

    """
    if enable:
        button.setStyleSheet("background-color: White")
        button.setCheckable(True)
    else:
        button.setStyleSheet("background-color: Grey")
        button.setCheckable(False)


def disable_slider(slider):
    """
    Disables a slider by getting the current value and setting the slider to that
    value upon every value change

    This results in slider value not changing even when slider is moved

    :param slider: slider widget to be disabled
    """
    old_val = slider.value()
    slider.valueChanged.connect(lambda: slider.setValue(old_val))
    slider.setStyleSheet(
        "QSlider::sub-page:horizontal"
        "{"
        "background-color: grey"
        "}"
        "QSlider::handle"
        "{"
        "background: grey;"
        "color: grey;"
        "border-radius: 5px;"
        "}"
    )


def enable_slider(slider, label, get_value):
    """
    Enables a slider by connecting a function to update label upon value change
    :param slider: slider widget to be enabled
    :param label: label widget corresponding to the slider
    :param get_value: function to translate slider value into label text
    """
    slider.valueChanged.connect(lambda: label.setText(get_value(slider.value())))
    slider.setStyleSheet(
        "QSlider::groove:horizontal"
        "{"
        "border-width: 0px"
        "}"
    )


def disable_radio_button(button_group):
    button_group.setExclusive(False)
    for button in button_group.buttons():
        button.setChecked(False)
        button.clicked.disconnect()
        button.clicked.connect(
            lambda state, curr=button: curr.setChecked(False)
        )


