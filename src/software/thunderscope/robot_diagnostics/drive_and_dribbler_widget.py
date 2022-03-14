import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QGridLayout,
    QGroupBox,
    QMenu,
    QPushButton,
    QRadioButton,
    QHBoxLayout,
    QVBoxLayout,
    QWidget,
    QSlider,
    QLabel,
    QTabWidget,
)


class DriveAndDribblerWidget(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        layout = QVBoxLayout()

        # Initialize tab screen
        tabs = QTabWidget()
        tab1 = QWidget()
        tab2 = QWidget()
        tabs.setStyleSheet("background-color: black; color:white")

        # Add tabs
        tabs.addTab(tab1, "Direct Per-Wheel Control")
        tabs.addTab(tab2, "Direct Velocity Control")

        # Create first tab
        tab1.grid = QVBoxLayout()
        tab1.grid.addWidget(self.setUpDirectPerWheel("Drive"))
        tab1.grid.addStretch(1)
        tab1.setLayout(tab1.grid)

        # Create second tab
        tab2.grid2 = QVBoxLayout()
        tab2.grid2.addWidget(self.setUpDirectVelocity("Drive"))
        tab2.grid2.addStretch(1)
        tab2.setLayout(tab2.grid2)

        # Add tabs to widget
        layout.addWidget(tabs)
        layout.addWidget(self.setUpDribbler("  Dribbler"))
        self.setLayout(layout)

    def setSlider(self, title):
        groupBox = QGroupBox(title)

        # set up the slide
        slider = QSlider(Qt.Horizontal)
        slider.setStyleSheet("font:white")
        slider.setFocusPolicy(Qt.StrongFocus)
        slider.setSingleStep(1)
        slider.setMinimum(-100)
        slider.setMaximum(100)

        # create a label to display slide's value
        label = QLabel()
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font:white;")
        slider.valueChanged.connect(label.setNum)

        # add widgets
        vbox = QGridLayout()
        vbox.addWidget(slider, 0, 0)
        vbox.addWidget(label, 0, 1)
        groupBox.setLayout(vbox)

        return groupBox, slider

    def resetBtn(self):
        wheelBtn = QPushButton("Stop and Reset")
        wheelBtn.setFixedWidth(150)

        return wheelBtn

    def setUpDirectPerWheel(self, title):

        groupBox = QGroupBox(title)
        dbox = QVBoxLayout()

        # set up the sliders
        frontLeft, slidefl = self.setSlider("Front Left")
        frontRight, slidefr = self.setSlider("Front Right")
        backLeft, slidebl = self.setSlider("Back Left")
        backRight, slidebr = self.setSlider("Back Right")

        # set up the stop and reset button
        stopAndreset = self.resetBtn()
        stopAndreset.clicked.connect(lambda: slidefl.setValue(0))
        stopAndreset.clicked.connect(lambda: slidefr.setValue(0))
        stopAndreset.clicked.connect(lambda: slidebl.setValue(0))
        stopAndreset.clicked.connect(lambda: slidebr.setValue(0))

        # add widget
        dbox.addWidget(frontLeft)
        dbox.addWidget(frontRight)
        dbox.addWidget(backLeft)
        dbox.addWidget(backRight)
        dbox.addWidget(stopAndreset, alignment=Qt.AlignCenter)

        groupBox.setLayout(dbox)

        return groupBox

    def setUpDirectVelocity(self, title):

        groupBox = QGroupBox(title)
        dbox = QVBoxLayout()

        xms, slidexms = self.setSlider("X (m/s)")
        yms, slideyms = self.setSlider("Y (m/s)")
        degree, slidedegree = self.setSlider("θ (°/s)")

        stopAndreset = self.resetBtn()

        stopAndreset.clicked.connect(lambda: slidexms.setValue(0))
        stopAndreset.clicked.connect(lambda: slideyms.setValue(0))
        stopAndreset.clicked.connect(lambda: slidedegree.setValue(0))

        dbox.addWidget(xms)
        dbox.addWidget(yms)
        dbox.addWidget(degree)
        dbox.addWidget(stopAndreset, alignment=Qt.AlignCenter)

        groupBox.setLayout(dbox)

        return groupBox

    def setUpDribbler(self, title):

        groupBox = QGroupBox(title)
        dbox = QVBoxLayout()

        dribbler, sliderDribbler = self.setSlider("RPM")

        stopAndreset = self.resetBtn()
        stopAndreset.clicked.connect(lambda: sliderDribbler.setValue(0))

        dbox.addWidget(dribbler)
        dbox.addWidget(stopAndreset, alignment=Qt.AlignCenter)
        groupBox.setStyleSheet("background-color: black; color:white")
        groupBox.setLayout(dbox)

        return groupBox
