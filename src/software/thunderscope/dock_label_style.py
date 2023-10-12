from pyqtgraph.dockarea.Dock import DockLabel


def updateStylePatched(self) -> None:
    """
    Overrides the style of the label.
    """
    r = "2px"
    if self.dim:
        fg = "#a0a0a0"
        bg = "#313742"
        border = "#313742"
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
            font-size: 14px;
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
            font-size: 14px
        }""" % (
            bg,
            fg,
            r,
            r,
            border,
        )
        self.setStyleSheet(self.hStyle)


DockLabel.updateStyle = updateStylePatched
