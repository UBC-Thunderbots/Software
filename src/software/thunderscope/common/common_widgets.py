from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt.QtCore import *
from software.py_constants import *


class FloatSlider(QSlider):
    """
    This class extends QSlider to offer support to float values instead of just ints
    """

    floatValueChanged = pyqtSignal(float)

    def __init__(self, decimals=1, *args, **kwargs):
        """
        Creates a FloatSlider with the given number of decimal places
        :param decimals: number of decimal places that value of slider should have
        """
        super(FloatSlider, self).__init__(*args, **kwargs)
        self.decimals = 10 ** decimals

        # slider now emits a float value signal every time its value changes
        self.valueChanged.connect(self.emitFloatValueChanged)

    def emitFloatValueChanged(self):
        """
        Emits a signal with the slider's float value
        """
        self.floatValueChanged.emit(self.value())

    def value(self):
        """
        Gets the actual value of the slider and converts it to the float value
        of corresponding decimal places
        :return: the float value of the slider
        """
        return float(super(FloatSlider, self).value()) / self.decimals

    def setMinimum(self, min_val):
        """
        Sets a minimum float value for this slider
        :param min_val: value to set as the minimum
        """
        return super(FloatSlider, self).setMinimum(min_val * self.decimals)

    def setMaximum(self, max_val):
        """
        Sets a maximum float value for this slider
        :param max_val: value to set as the maximum
        """
        return super(FloatSlider, self).setMaximum(max_val * self.decimals)

    def setValue(self, value):
        """
        Sets a float value as the value for this slider
        :param value: value to set as the slider's value
        """
        super(FloatSlider, self).setValue(int(value * self.decimals))


class ColorProgressBar(QProgressBar):
    """
    This class extends QSlider to support floats instead of ints
    Also changes slider color based on percentage filled
    """

    floatValueChanged = pyqtSignal(float)

    def __init__(self, min_val, max_val, decimals=2):
        """
        Creates a ColorProgressBar with the specified min, max and decimals
        Sets initial slider color to grey
        :param min_val: min value of slider
        :param max_val: max value of slider
        :param decimals: number of decimal places to be used
        """
        super(ColorProgressBar, self).__init__()

        self.decimals = 10 ** decimals

        super(ColorProgressBar, self).setRange(
            int(min_val * self.decimals), int(max_val * self.decimals)
        )

        super(ColorProgressBar, self).setStyleSheet(
            "QProgressBar::chunk" "{" "background: grey" "color: black" "}"
        )

        self.valueChanged.connect(self.emitFloatValueChanged)

    def emitFloatValueChanged(self):
        """
        Emits a signal with the slider's float value
        """
        self.floatValueChanged.emit(self.value())

    def setValue(self, value):
        """
        Sets the value of the slider to the given float value
        Sets the color of the slider based on the percentage filled
            - 100% to 50% -> Green to Yellow
            - 50% to 0% -> Yellow to Red
        :param value:
        :return:
        """
        super(ColorProgressBar, self).setValue(value * self.decimals)

        # clamp percent to make sure it's between 0% and 100%
        percent = min(
            1,
            max(0, (self.value() - self.minimum()) / (self.maximum() - self.minimum())),
        )

        if percent < 0.5:
            super(ColorProgressBar, self).setStyleSheet(
                "QProgressBar::chunk"
                "{"
                f"background: rgb(255, {255 * (2 * percent)}, 0)"
                "}"
            )
        else:
            super(ColorProgressBar, self).setStyleSheet(
                "QProgressBar::chunk"
                "{"
                f"background: rgb({255 * 2 * (1 - percent)}, 255, 0)"
                "}"
            )

    def maximum(self):
        return float(super(ColorProgressBar, self).maximum()) / self.decimals

    def minimum(self):
        return float(super(ColorProgressBar, self).minimum()) / self.decimals

    def value(self):
        return float(super(ColorProgressBar, self).value()) / self.decimals


def create_buttons(text: list):
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


def create_slider_abs(slider, text, min_val, max_val, tick_spacing):
    """Set a given QSlider or extended slider inside a QGroupBox object, along with a value
    label on the right. The slider orientation will be horizontal.

    Allows support for classes that extend QSlider to support floats, etc.

    :param slider: slider of type QSlider or a child class of QSlider
    :param text: text to display above the slider
    :param min_val: lowest value of the slider
    :param max_val: highest value of the slider
    :param tick_spacing: interval between two ticks on the slider
    :return vbox, slider, value_label:
            QVBoxLayout object - add this to the widget
            QSlider object - use this to perform tasks on the button
            displays value of slider, update this when value is changed
    """
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


def create_slider(text, min_val, max_val, tick_spacing):
    """Creates a QSlider object and returns a QGroupBox object with the slider and a label

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

    return create_slider_abs(slider, text, min_val, max_val, tick_spacing)


def create_float_slider(text, decimals, min_val, max_val, tick_spacing):
    """Creates a FloatSlider object and returns a QGroupBox object with the slider and a label

    :param decimals: number of decimals for slider values
    :param text: text to display above the slider
    :param min_val: lowest value of the slider
    :param max_val: highest value of the slider
    :param tick_spacing: interval between two ticks on the slider
    :return vbox, slider, value_label:
            QVBoxLayout object - add this to the widget
            QSlider object - use this to perform tasks on the button
            displays value of slider, update this when value is changed
    """

    slider = FloatSlider(decimals, Qt.Orientation.Horizontal)

    return create_slider_abs(slider, text, min_val, max_val, tick_spacing)


def create_push_button(title):
    """Create a push button

    :param title: the name of the button

    """
    push_button = QPushButton(title)
    push_button.setFixedWidth(150)

    return push_button


def set_table_data(
    data, table, header_size_hint_width_expansion, item_size_hint_width_expansion
):
    """Set data in a table

    :param data: dict containing {"column_name": [column_items]}
    :param table: table widget that will contain the data
    :param header_size_hint_width_expansion: the factor multiplied by the length of the header
    :param item_size_hint_width_expansion: the factor multiplied by the length of the item

    """
    horizontal_headers = []

    for n, key in enumerate(data.keys()):
        horizontal_headers.append(key)

        for m, item in enumerate(data[key]):
            str_item = str(item)
            newitem = QTableWidgetItem(str_item)
            newitem.setSizeHint(
                QtCore.QSize(
                    max(
                        len(key) * header_size_hint_width_expansion,
                        len(str_item) * item_size_hint_width_expansion,
                    ),
                    1,
                )
            )
            table.setItem(m, n, newitem)

    table.setHorizontalHeaderLabels(horizontal_headers)


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
    slider.setStyleSheet("QSlider::groove:horizontal" "{" "border-width: 0px" "}")


def disable_radio_button(button_group):
    """
    Disables a whole radio button group
    Sets all buttons to unselected and disables their onClick function
    :param button_group: button group to disable
    """
    button_group.setExclusive(False)
    for button in button_group.buttons():
        button.setChecked(False)
        button.clicked.disconnect()
        button.clicked.connect(lambda state, curr=button: curr.setChecked(False))


def draw_robot(painter, rect, start_angle_degree, span_angle_degree):
    """
    Draws a robot bounded by the given rectangle with a chord defined by the given angles
    :param painter: the QPainter object that is drawing in thunderscope
    :param rect: the rectangle that is bounding the robot's circle
    :param start_angle_degree: the start of the chord, measured anti-clockwise from the horizontal middle in degrees
    :param span_angle_degree: the end of the chord, measured anti-clockwise from the horizontal middle in degrees
    """
    convert_degree = -16

    painter.drawChord(
        rect, start_angle_degree * convert_degree, span_angle_degree * convert_degree,
    )


def display_tooltip(event, tooltip_text):
    """
    Checks given event to see if it is an Enter or Leave event
    Upon Enter, displays a tooltip with the given text
    Upon Leave, hides the tooltip
    :param event: event to check
    :param tooltip_text: the text to display in the tooltip
    """
    if str(event.type()) == "Type.Enter":
        QToolTip.showText(
            QPoint(int(event.globalPosition().x()), int(event.globalPosition().y()),),
            tooltip_text,
            msecShowTime=20 * MILLISECONDS_PER_SECOND,
        )
    elif str(event.type()) == "Type.Leave":
        QToolTip.hideText()
