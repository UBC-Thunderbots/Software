from typing import Callable, Optional, Sequence, Any, Dict
from software.thunderscope.constants import ParamTypes, ProtoUnixIOTypes


class TScopeWidgetDep:
    """
    Dependency for a widget that it needs for setup
    """

    name: str  # name of param
    type: ParamTypes  # type of param
    value: Any  # value of param

    def __init__(self, name: str, dep_type: ParamTypes, value: object):
        self.name = name
        self.type = dep_type
        self.value = value


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
    setup: Callable[[object], Any]  # setup function to construct widget
    anchor: Optional[str]  # name of widget to position this relative to
    position: Optional[str]  # position relative to anchor to position this to
    deps: Sequence[TScopeWidgetDep]  # params taken by setup function and values
    stretch: Optional[WidgetStretchData]  # how widget should be sized
    no_refresh: Optional[bool]  # if this widget has a refresh function or not
    in_window: Optional[bool]  # if this widget should be added in window or not

    def __init__(
        self,
        name: str,
        setup: Callable[[object], None],
        anchor: Optional[str] = None,
        position: Optional[str] = None,
        deps: Sequence[TScopeWidgetDep] = [],
        in_window: Optional[bool] = False,
        stretch: Optional[object] = None,
        no_refresh: Optional[bool] = False,
    ):
        self.name = name
        self.setup = setup
        self.anchor = anchor
        self.position = position
        self.deps = deps
        self.stretch = stretch
        self.no_refresh = no_refresh
        self.in_window = in_window


class TScopeTab:
    """
    Data that describes a tab in Thunderscope
    """

    name: str  # name of tab

    def __init__(self, name: str):
        self.name = name


class TScopeQTTab(TScopeTab):
    """
    Data that describes a tab with Qt Widgets in Thunderscope
    """

    widgets: Sequence[TScopeWidget]  # list of widgets for this tab

    def __init__(self, name: str, widgets: Sequence[TScopeWidget]):
        super().__init__(name)
        self.widgets = widgets


class TScopeWebTab(TScopeTab):
    """
    Data that describes a tab that shows a webpage in Thunderscope
    """

    url: str  # url of webpage displayed by this tab

    def __init__(self, name: str, url: str):
        super().__init__(name)
        self.url = url


class TScopeConfig:
    """
    Data that described a whole Thunderscope view
    """

    protos: Dict[
        ProtoUnixIOTypes, Optional[str]
    ]  # mapping of protos needed for this view
    tabs: Sequence[TScopeTab]  # list of tabs for this view

    def __init__(
        self, protos: Dict[ProtoUnixIOTypes, Optional[str]], tabs: Sequence[TScopeTab]
    ):
        self.protos = protos
        self.tabs = tabs
