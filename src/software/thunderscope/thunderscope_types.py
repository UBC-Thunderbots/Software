from typing import Callable, Optional, Sequence, Any, Dict
from software.thunderscope.common.frametime_counter import FrameTimeCounter
from software.thunderscope.constants import TabNames

from pyqtgraph.Qt import QtCore
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.dockarea import *


class WidgetStretchData:
    """
    Data that describes how a widget should be sized
    """

    x: Optional[int]  # stretch in x direction
    y: Optional[int]  # stretch in y direction

    def __init__(self, x: int = None, y: int = None) -> None:
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
    has_refresh_func: Optional[bool]  # if this widget has a refresh function or not
    in_window: Optional[bool]  # if this widget should be added in window or not

    def __init__(
        self,
        name: str,
        widget: Any,
        anchor: Optional[str] = None,
        position: Optional[str] = None,
        in_window: Optional[bool] = False,
        stretch: Optional[object] = None,
        has_refresh_func: Optional[bool] = True,
    ) -> None:
        self.name = name
        self.widget = widget
        self.anchor = anchor
        self.position = position
        self.stretch = stretch
        self.has_refresh_func = has_refresh_func
        self.in_window = in_window


class TScopeTab:
    """
    Data that describes a tab in Thunderscope
    """

    name: str  # name of tab
    key: TabNames  # key to identify this tab
    dock_area: QWidget  # Dock Area for this tab

    def __init__(self, name: str, key: TabNames) -> None:
        self.name = name
        self.key = key

    def refresh(self) -> None:
        pass


class TScopeQTTab(TScopeTab):
    """
    Data that describes a tab with Qt Widgets in Thunderscope
    """

    widgets: Sequence[TScopeWidget]  # list of widget data for this tab
    widgets_map: Dict[str, TScopeWidget]  # Mapping of widget names to widget objects
    dock_map: Dict[str, DockArea]  # Mapping of widget names to dock areas
    refresh_functions: Dict[
        str, Callable[[], None]
    ]  # Mapping of widget names to refresh functions

    def __init__(
        self,
        name: str,
        key: TabNames,
        widgets: Sequence[TScopeWidget],
        refresh_func_counter: FrameTimeCounter = None,
    ) -> None:
        """
        name: the name of this tab
        key: the key to identify this tab
        widgets: a list of widgets that is going to be displayed in the tab
        refresh_func_counter: a counter that tracks the runtime of the refresh function
        :Return: None
        """
        super().__init__(name, key)
        self.widgets = widgets
        self.refresh_functions = {}

        self.widgets_map = {}
        self.dock_map = {}

        # make dock area
        self.dock_area = DockArea()
        self.dock_area.layout.setContentsMargins(12, 12, 12, 12)

        # first widget is initial anchor widget
        # all other widgets will be positioned relative to this one
        for widget in self.widgets:
            self.add_one_widget(widget)

        # initialized a frametime counter if none was passed in
        self.refresh_func_counter = refresh_func_counter
        if refresh_func_counter == None:
            self.refresh_func_counter = FrameTimeCounter()

    def add_one_widget(self, data: TScopeWidget) -> None:
        """
        Gets the widget name and object from the given data
        Add widget to a dock and adds dock to this tab's dock area
        And to this tab's map of widgets and docks
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

        if data.has_refresh_func:
            self.refresh_functions[widget_name] = new_widget.refresh

    def refresh(self) -> None:
        """
        Refreshes all the widgets belonging to this tab, and not refresh widget that are not visible.
        """
        if not self.dock_area.isVisible():
            return

        self.refresh_func_counter.add_one_datapoint()

        for widget_name in self.refresh_functions:
            # only refresh widget inside the dock that are visible
            widget = self.widgets_map[widget_name]
            if not widget.isVisible():
                continue

            refresh_func = self.refresh_functions[widget_name]
            refresh_func()

    def find_widget(self, widget_name: str) -> Optional[TScopeWidget]:
        """
        Finds and returns the widget object corresponding to the given name, if exists
        If not, returns None
        :param widget_name: the name of the widget
        """
        if widget_name in self.widgets_map:
            return self.widgets_map[widget_name]
        else:
            return None
