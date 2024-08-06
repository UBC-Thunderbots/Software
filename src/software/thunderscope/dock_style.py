from pyqtgraph.dockarea.Dock import Dock, DockLabel


def updateDockStylePatched(self):
    """Overrides the style of the dock."""
    border = "#3f4042"
    border_width = "1px"
    border_radius = "5px"

    if self.labelHidden:
        self.nStyle = f"""
        Dock > QWidget {{
            border: {border_width} solid {border};
            border-radius: {border_radius};
        }}"""
        self.widgetArea.setStyleSheet(self.nStyle)

    elif self.orientation == "vertical":
        self.label.setOrientation("vertical")
        if self.moveLabel:
            self.topLayout.addWidget(self.label, 1, 0)
        self.vStyle = f"""
        Dock > QWidget {{
            border: {border_width} solid {border};
            border-radius: {border_radius};
            border-top-left-radius: 0px;
            border-bottom-left-radius: 0px;
        }}"""
        self.widgetArea.setStyleSheet(self.vStyle)

    else:
        self.label.setOrientation("horizontal")
        if self.moveLabel:
            self.topLayout.addWidget(self.label, 0, 1)
        self.hStyle = f"""
        Dock > QWidget {{
            border: {border_width} solid {border};
            border-radius: {border_radius};
            border-top-left-radius: 0px;
            border-top-right-radius: 0px;
        }}"""
        self.widgetArea.setStyleSheet(self.hStyle)


def updateDockLabelStylePatched(self):
    """Overrides the style of the dock label."""
    r = "5px"
    if self.dim:
        fg = "#aaa"
        bg = "#2f5082"
    else:
        fg = "#fff"
        bg = "#4272b8"

    if self.orientation == "vertical":
        self.vStyle = f"""DockLabel {{
            background-color : {bg};
            color : {fg};
            border-top-right-radius: 0px;
            border-top-left-radius: {r};
            border-bottom-right-radius: 0px;
            border-bottom-left-radius: {r};
            border-width: 0px;
            padding-top: 3px;
            padding-bottom: 3px;
            font-size: 15px;
        }}"""
        self.setStyleSheet(self.vStyle)
    else:
        self.hStyle = f"""DockLabel {{
            background-color : {bg};
            color : {fg};
            border-top-right-radius: {r};
            border-top-left-radius: {r};
            border-bottom-right-radius: 0px;
            border-bottom-left-radius: 0px;
            border-width: 0px;
            padding-left: 13px;
            padding-right: 13px;
            font-size: 15px;
        }}"""
        self.setStyleSheet(self.hStyle)


Dock.updateStyle = updateDockStylePatched
DockLabel.updateStyle = updateDockLabelStylePatched
