from pyqtgraph.dockarea.Dock import Dock, DockLabel

def updateDockStylePatched(self):
    """
    Overrides the style of the dock.
    """
    border = "#4f5b62"
    border_width = "2px"
    border_radius = "5px"
    
    if self.labelHidden:
        self.nStyle = """
        Dock > QWidget {
            border: %s solid %s;
            border-radius: %s;
        }""" % (
            border_width,
            border,
            border_radius
        )
        self.widgetArea.setStyleSheet(self.nStyle)

    elif self.orientation == 'vertical':
        self.label.setOrientation('vertical')
        if self.moveLabel:
            self.topLayout.addWidget(self.label, 1, 0)
        self.vStyle = """
        Dock > QWidget {
            border: %s solid %s;
            border-radius: %s;
            border-top-left-radius: 0px;
            border-bottom-left-radius: 0px;
        }""" % (
            border_width,
            border,
            border_radius
        )
        self.widgetArea.setStyleSheet(self.vStyle)

    else:
        self.label.setOrientation('horizontal')
        if self.moveLabel:
            self.topLayout.addWidget(self.label, 0, 1)
        self.hStyle = """
        Dock > QWidget {
            border: %s solid %s;
            border-radius: %s;
            border-top-left-radius: 0px;
            border-top-right-radius: 0px;
        }""" % (
            border_width,
            border,
            border_radius
        )
        self.widgetArea.setStyleSheet(self.hStyle)

def updateDockLabelStylePatched(self):
    """
    Overrides the style of the label.
    """
    r = "2px"
    if self.dim:
        fg = "#a0a0a0"
        bg = "#2b303a"
        border = "#2b303a"
    else:
        fg = "#fff"
        bg = "#448aff"
        border = "#448aff"

    if self.orientation == "vertical":
        self.vStyle = """DockLabel {
            background-color : %s;
            color : %s;
            border-top-right-radius: 0px;
            border-top-left-radius: %s;
            border-bottom-right-radius: 0px;
            border-bottom-left-radius: %s;
            border-width: 0px;
            border-right: 2px solid %s;
            padding-top: 3px;
            padding-bottom: 3px;
            font-size: 13px;
            text-transform: uppercase;
            font-weight: bold;
        }""" % (
            bg,
            fg,
            r,
            r,
            border,
        )
        self.setStyleSheet(self.vStyle)
    else:
        self.hStyle = """DockLabel {
            background-color : %s;
            color : %s;
            border-top-right-radius: %s;
            border-top-left-radius: %s;
            border-bottom-right-radius: 0px;
            border-bottom-left-radius: 0px;
            border-width: 0px;
            border-bottom: 2px solid %s;
            padding-left: 13px;
            padding-right: 13px;
            font-size: 13px;
            text-transform: uppercase;
            font-weight: bold;
        }""" % (
            bg,
            fg,
            r,
            r,
            border,
        )
        self.setStyleSheet(self.hStyle)

Dock.updateStyle = updateDockStylePatched
DockLabel.updateStyle = updateDockLabelStylePatched
