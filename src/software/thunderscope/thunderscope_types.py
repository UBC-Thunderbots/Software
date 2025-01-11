from typing import Optional, Sequence, Any
from software.thunderscope.common.frametime_counter import FrameTimeCounter

from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.dockarea import *

from dataclasses import dataclass
from enum import Enum


class WidgetPosition(Enum):
    """Enum that describes how a widget should be placed relative its anchor"""

    BOTTOM = "bottom"
    TOP = "top"
    LEFT = "left"
    RIGHT = "right"
    ABOVE = "above"
    BELOW = "below"


@dataclass
class WidgetStretchData:
    """Data that describes how a widget should be sized"""

    x: Optional[int] = None
    """Stretch in x direction"""

    y: Optional[int] = None
    """Stretch in y direction"""


@dataclass
class TScopeWidget:
    """Data that describes a widget in Thunderscope"""

    name: str
    """Name of widget (must be unique)"""

    widget: Any
    """The widget object"""

    anchor: Optional[str] = None
    """Name of widget to position this relative to"""

    position: Optional[WidgetPosition] = None
    """Position relative to anchor to position this to"""

    stretch: Optional[WidgetStretchData] = None
    """How widget should be sized"""

    has_refresh_func: Optional[bool] = True
    """If this widget has a refresh function or not"""

    in_window: Optional[bool] = False
    """If this widget should be added in window or not"""


class TScopeTab:
    """Data that describes a tab with Qt Widgets in Thunderscope"""

    def __init__(
        self,
        name: str,
        widgets: Sequence[TScopeWidget],
        refresh_counter: Optional[FrameTimeCounter] = None,
    ) -> None:
        """Constructor

        :param name: the name of this tab
        :param widgets: a list of widgets that is going to be displayed in the tab
        :param refresh_counter: a FrameTimeCounter to track the time between calls
                                to the refresh function
        """
        self.name = name
        self.widgets = widgets

        # Mapping of widget names to widget objects
        self.widgets_map: dict[str, TScopeWidget] = {}

        # Mapping of widget names to dock areas
        self.dock_map: dict[str, DockArea] = {}

        # make dock area
        self.dock_area = DockArea()
        self.dock_area.layout.setContentsMargins(12, 12, 12, 12)

        # first widget is initial anchor widget
        # all other widgets will be positioned relative to this one
        for widget in self.widgets:
            self.add_one_widget(widget)

        self.refresh_counter = (
            refresh_counter if refresh_counter else FrameTimeCounter()
        )

    def add_one_widget(self, data: TScopeWidget) -> None:
        """Gets the widget name and object from the given data
        Add widget to a dock and adds dock to this tab's dock area
        And to this tab's map of widgets and docks

        :param data: the data describing the widget of type TScopeWidget
        """
        self.widgets_map[data.name] = data
        new_dock = Dock(data.name)
        new_dock.addWidget(data.widget.win if data.in_window else data.widget)
        self.dock_map[data.name] = new_dock

        if data.stretch:
            stretch_data = data.stretch
            if stretch_data.y:
                new_dock.setStretch(y=stretch_data.y)
            if stretch_data.x:
                new_dock.setStretch(x=stretch_data.x)

        if data.anchor and data.position:
            self.dock_area.addDock(
                new_dock, data.position.value, self.dock_map[data.anchor]
            )
        else:
            self.dock_area.addDock(new_dock)

    def refresh(self) -> None:
        """Refreshes all the widgets belonging to this tab, and not refresh widget that are not visible."""
        if not self.dock_area.isVisible():
            return

        self.refresh_counter.add_one_datapoint()

        for widget_data in self.widgets_map.values():
            # only refresh widget inside the dock that are visible
            if widget_data.has_refresh_func and widget_data.widget.isVisible():
                widget_data.widget.refresh()

    def find_widget(self, widget_name: str) -> Optional[TScopeWidget]:
        """Finds and returns the widget object corresponding to the given name, if exists
        If not, returns None

        :param widget_name: the name of the widget
        """
        widget_data = self.widgets_map.get(widget_name, None)
        return widget_data.widget if widget_data else None
