from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QGridLayout,
    QGroupBox,
    QButtonGroup,
    QMenu,
    QPushButton,
    QRadioButton,
    QVBoxLayout,
    QHBoxLayout,
    QWidget,
    QSlider,
)


def create_button(text, checkable_initially=True):
    """
    Creates a QPushButton object inside a QGroupBox object.
    See line 47: src/software/thunderscope/chicker/chicker.py for an example.
    The default color of button will be white with black background.

    :param text: text to display on the button
    :param: checkable_initially: by default button will be checkable, pass False to change this
    :return: group_box: QGroupBox object - add this to the widget
    :return: button: QPushButton object - use this to perform tasks on the button
    """
    group_box = QGroupBox()
    button = QPushButton(text)
    group_box.setStyleSheet("color: black")
    button.setCheckable(True)
    if not checkable_initially:
        button.setCheckable(False)
        button.setStyleSheet("background-color: Grey")
    vbox = QVBoxLayout()
    vbox.addWidget(button)
    vbox.addStretch(1)
    group_box.setLayout(vbox)
    return group_box, button


def create_radio(text, radio_group):
    """
    Creates a QRadioButton object inside a QGroupBox object.
    See line 59: src/software/thunderscope/chicker/chicker.py for an example.
    The default color of button background will be white.

    :param text: text to display beside the button
    :param radio_group: QButtonGroup to add this button to
    :return: group_box: QGroupBox object - add this to the widget
    :return: button: QRadioButton object - use this to perform tasks on the button
    """
    group_box = QGroupBox()
    radio = QRadioButton(text)
    group_box.setStyleSheet("color: black")
    # this is so that the button is properly visible in black background
    radio.setStyleSheet("background-color: white")
    radio_group.addButton(radio)
    vbox = QVBoxLayout()
    vbox.addWidget(radio)
    vbox.addStretch(1)
    group_box.setLayout(vbox)
    return group_box, radio


def create_slider(text, min_val, max_val, tick_spacing):
    """
    Creates a QSlider object inside a QGroupBox object.
    See line 71: src/software/thunderscope/chicker/chicker.py for an example.
    The slider orientation will be horizontal.

    :param text: text to display above the slider
    :param min_val: lowest value of the slider
    :param max_val: highest value of the slider
    :param tick_spacing: interval between two ticks on the slider
    :return: group_box: QGroupBox object - add this to the widget - see example
    :return: slider: QSlider object - use this to perform tasks on the button
    """
    group_box = QGroupBox(text)
    slider = QSlider(Qt.Horizontal)
    slider.setMinimum(min_val)
    slider.setMaximum(max_val)
    slider.setTickPosition(QSlider.TicksBothSides)
    slider.setTickInterval(tick_spacing)
    group_box.setStyleSheet("color: white")
    vbox = QVBoxLayout()
    vbox.addWidget(slider)
    vbox.addStretch(1)
    group_box.setLayout(vbox)
    return group_box, slider
