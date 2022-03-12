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

def create_button(text):
    group_box = QGroupBox()
    button = QPushButton(text)
    group_box.setStyleSheet("color: black")
    button.setCheckable(True)
    if text != "Charge":
        button.setCheckable(False)
        button.setStyleSheet("background-color: Grey")
    vbox = QVBoxLayout()
    vbox.addWidget(button)
    vbox.addStretch(1)
    group_box.setLayout(vbox)
    return group_box, button

def create_radio(text, radio_group):
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