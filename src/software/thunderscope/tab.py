from typing import Callable, Optional, Sequence, Any, Dict
from software.thunderscope.constants import TabNames

import PyQt6
from PyQt6.QtWebEngineWidgets import QWebEngineView
from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.dockarea import *


class WidgetStretchData:
    """
    Data that describes how a widget should be sized
    """

    x: Optional[int]  # stretch in x direction
    y: Optional[int]  # stretch in y direction

    def __init__(self, x: int = None, y: int = None):
        self.x = x
        self.y = y


class TScopeWidget:
    """
    Data that describes a widget in Thunderscope
    """

    name: str  # name of widget (must be unique)
    widget: Any  # the widget object
    anchor: Optional[str]  # name of widget to position this relative to
    position: Optional[str]  # position relative to anchor to position this to
    stretch: Optional[WidgetStretchData]  # how widget should be sized
    no_refresh: Optional[bool]  # if this widget has a refresh function or not
    in_window: Optional[bool]  # if this widget should be added in window or not

    def __init__(
            self,
            name: str,
            widget: Any,
            anchor: Optional[str] = None,
            position: Optional[str] = None,
            in_window: Optional[bool] = False,
            stretch: Optional[object] = None,
            no_refresh: Optional[bool] = False,
    ):
        self.name = name
        self.widget = widget
        self.anchor = anchor
        self.position = position
        self.stretch = stretch
        self.no_refresh = no_refresh
        self.in_window = in_window


class TScopeTab:
    """
    Data that describes a tab in Thunderscope
    """

    name: str               # name of tab
    key: TabNames           # key to identify this tab
    dock_area: QWidget      # Dock Area for this tab

    def __init__(self, name: str, key: TabNames):
        self.name = name
        self.key = key

    def refresh(self):
        pass


class TScopeQTTab(TScopeTab):
    """
    Data that describes a tab with Qt Widgets in Thunderscope
    """

    widgets: Sequence[TScopeWidget]                   # list of widget data for this tab
    widgets_map: Dict[str, TScopeWidget]              # Mapping of widget names to widget objects
    dock_map: Dict[str, DockArea]                     # Mapping of widget names to dock areas
    refresh_functions: Dict[str, Callable[[], None]]  # Mapping of widget names to refresh functions

    def __init__(self, name: str, key: TabNames, widgets: Sequence[TScopeWidget]):
        super().__init__(name, key)
        self.widgets = widgets
        self.refresh_functions = {}

        self.widgets_map = {}
        self.dock_map = {}

        # make dock area
        self.dock_area = DockArea()

        # first widget is initial anchor widget
        # all other widgets will be positioned relative to this one
        for widget in self.widgets:
            self.add_one_widget(widget)

    def add_one_widget(self, data: TScopeWidget):
        """
        Constructs a widget and dock with the given data and adds it to the given dock area
        As well as to the given map of widgets and docks
        :param data: the data describing the widget of type TScopeWidget
        """

        widget_name = data.name
        new_widget = data.widget

        self.widgets_map[widget_name] = new_widget
        new_dock = Dock(widget_name)
        new_dock.addWidget(new_widget.win if data.in_window else new_widget)
        self.dock_map[widget_name] = new_dock

        if data.stretch:
            stretch_data = data.stretch
            if stretch_data.y:
                new_dock.setStretch(y=stretch_data.y)
            if stretch_data.x:
                new_dock.setStretch(x=stretch_data.x)

        if data.anchor and data.position:
            self.dock_area.addDock(new_dock, data.position, self.dock_map[data.anchor])
        else:
            self.dock_area.addDock(new_dock)

        if not data.no_refresh:
            self.refresh_functions[widget_name] = new_widget.refresh

    def refresh(self):
        for refresh_func in self.refresh_functions.values():
            refresh_func()

    def find_widget(self, widget_name):
        if widget_name in self.widgets_map:
            return self.widgets_map[widget_name]
        else:
            return None


class TScopeWebTab(TScopeTab):
    """
    Data that describes a tab that shows a webpage in Thunderscope
    """

    url: str  # url of webpage displayed by this tab

    def __init__(self, name: str, key: TabNames, url: str):
        super().__init__(name, key)
        self.url = url

        web_view = QWebEngineView()
        web_view.load(QtCore.QUrl(url))

        self.dock_area = web_view

