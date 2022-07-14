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
    :return: vbox_layout: QVBoxLayout object - add this to the widget - see example
    :return: slider: QSlider object - use this to perform tasks on the button
    :return: value_label: displays value of slider, update this when value is changed
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
