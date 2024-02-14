from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar
from software.thunderscope.proto_unix_io import ProtoUnixIO
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph.Qt import QtGui
from proto.import_all_protos import *
from proto.ssl_gc_common_pb2 import Team as SslTeam
from typing import Callable
import software.thunderscope.gl.widgets.toolbar_icons.gamecontroller.icon_loader as icons


class GamecontrollerPlays:
    """
    The different plays that can be set for each team
    """

    NONE = "None"
    DIRECT = "Direct"
    INDIRECT = "Indirect"
    KICKOFF = "Kickoff"
    PENALTY = "Penalty"


class GLGamecontrollerToolbar(GLToolbar):
    """
    A toolbar with controls to send GameController commands from Thunderscope
    """

    def __init__(self, proto_unix_io: ProtoUnixIO, friendly_color_yellow: bool):
        """
        Initializes the toolbar and constructs its layout

        :param proto_unix_io the ProtoUnixIO object to send the manual gamecontroller commands to
        :param friendly_color_yellow True if yellow is friendly team, False if not
        """
        super(GLGamecontrollerToolbar, self).__init__()

        self.proto_unix_io = proto_unix_io
        self.friendly_color_yellow = friendly_color_yellow

        # the currently selected play (starts as None)
        # self.current_selected_play = GamecontrollerPlays.NONE

        # TODO: change icon to pause
        # Setup Stop button for sending the STOP gamecontroller command
        self.stop_button = self.__setup_icon_button(
            icons.get_stop_icon(self.BUTTON_ICON_COLOR),
            "Stops gameplay, robots form circle around ball",
            self.__send_stop_command,
        )

        # Setup Force Start button for sending the FORCE_START gamecontroller command
        self.force_start_button = self.__setup_icon_button(
            icons.get_force_start_icon(self.BUTTON_ICON_COLOR),
            "Force Start, restarts the game",
            self.__send_force_start_command,
        )

        # TODO: change icon to cross
        # Setup Halt button for sending the HALT gamecontroller command
        self.halt_button = self.__setup_icon_button(
            icons.get_halt_icon(self.BUTTON_ICON_COLOR),
            "Halt, stops all robots immediately",
            self.__send_halt_command,
        )

        # Setup Normal Start button for sending the NORMAL_START gamecontroller command
        self.normal_start_button = self.__setup_icon_button(
            icons.get_normal_start_icon(self.BUTTON_ICON_COLOR),
            "Normal Start, resumes game from a set play (disabled when no play selected)",
            self.__send_normal_start_command,
        )

        # disable the normal start button when no play is selected
        self.normal_start_enabled = True
        self.__toggle_normal_start_button()

        # set up the menu for selecting plays
        self.plays_menu = QMenu()

        self.plays_menu_button = QPushButton()
        self.plays_menu_button.setText(GamecontrollerPlays.NONE)
        self.plays_menu_button.setStyleSheet(self.get_button_style())
        self.plays_menu_button.setMenu(self.plays_menu)

        # add play items for each team color
        self.__add_plays_menu_items(is_blue=True)
        self.plays_menu.addSeparator()
        self.__add_plays_menu_items(is_blue=False)

        self.layout().addWidget(self.stop_button)
        self.layout().addWidget(self.halt_button)
        self.layout().addWidget(self.normal_start_button)
        self.layout().addWidget(self.force_start_button)
        self.layout().addSpacing(20)
        self.layout().addWidget(self.plays_menu_button)
        self.layout().addStretch()

    def __add_plays_menu_items(self, is_blue: bool) -> None:
        self.__add_plays_menu_item(GamecontrollerPlays.DIRECT, is_blue)
        self.__add_plays_menu_item(GamecontrollerPlays.INDIRECT, is_blue)
        self.__add_plays_menu_item(GamecontrollerPlays.KICKOFF, is_blue)
        self.__add_plays_menu_item(GamecontrollerPlays.PENALTY, is_blue)

    def __add_plays_menu_item(self, play: GamecontrollerPlays, is_blue: bool) -> None:
        icon = icons.get_blue_icon() if is_blue else icons.get_yellow_icon()
        self.plays_menu.addAction(
            icon, play, lambda: self.__plays_menu_handler(play, icon, is_blue)
        )

    def __plays_menu_handler(
        self, play: GamecontrollerPlays, icon: QtGui.QIcon, is_blue: bool
    ):
        self.plays_menu_button.setIcon(icon)
        self.plays_menu_button.setText(play)

        command_type = None
        if play == GamecontrollerPlays.DIRECT:
            command_type = Command.Type.DIRECT
        elif play == GamecontrollerPlays.INDIRECT:
            command_type = Command.Type.INDIRECT
        elif play == GamecontrollerPlays.KICKOFF:
            command_type = Command.Type.KICKOFF
        else:
            command_type = Command.Type.PENALTY

        self.__send_gc_command(
            command_type, (SslTeam.YELLOW if is_blue else SslTeam.BLUE),
        )

        if not self.normal_start_enabled:
            self.__toggle_normal_start_button()

    def __toggle_normal_start_button(self) -> None:
        """ 
        Toggles the enabled / disabled state of the Normal Start button
        """
        self.normal_start_enabled = not self.normal_start_enabled
        self.normal_start_button.setStyleSheet(
            self.get_button_style(self.normal_start_enabled)
        )
        self.normal_start_button.setIcon(
            icons.get_normal_start_icon(
                self.BUTTON_ICON_COLOR
                if self.normal_start_enabled
                else self.DISABLED_BUTTON_ICON_COLOR
            )
        )

    def __setup_icon_button(
        self, icon: QtGui.QPixmap, tooltip: str, callback: Callable[[], None]
    ) -> QPushButton:
        """
        Sets up a button with the given name and callback

        :param icon: the icon displayed on the button
        :param tooltip: the tooltip displayed when hovering over the button
        :param callback: the callback for the button click
        :return: the button
        """
        button = QPushButton()
        button.setIcon(icon)
        button.setToolTip(tooltip)
        button.setStyleSheet(self.get_button_style())
        button.clicked.connect(callback)
        return button

    def __send_stop_command(self) -> None:
        """
        Sends a STOP command to the gamecontroller
        """
        self.__send_gc_command(Command.Type.STOP, SslTeam.UNKNOWN)

    def __send_force_start_command(self) -> None:
        """
        Sends a FORCE_START command for the current friendly team to the gamecontroller
        """
        self.__send_gc_command(Command.Type.FORCE_START, SslTeam.UNKNOWN)

    def __send_halt_command(self) -> None:
        """
        Sends a HALT command for the current friendly team to the gamecontroller
        """
        self.__send_gc_command(Command.Type.HALT, SslTeam.UNKNOWN)

    def __send_normal_start_command(self) -> None:
        """
        Sends a NORMAL START command for the current friendly team to the gamecontroller
        """
        if self.normal_start_enabled:
            self.__send_gc_command(Command.Type.NORMAL_START, SslTeam.UNKNOWN)

            self.__toggle_normal_start_button()

    def __send_gc_command(
        self, command: Command.Type, team: Team, ball_pos: Vector2 = None
    ) -> None:
        """
        Sends the given command to the gamecontroller for the given Team
        If ball_pos is defined, sets the ball position

        :param command the command to send
        :param team the team the command should be sent for (BLUE, YELLOW, or UNKNOWN)
        :param ball_pos if defined, sets the position of the ball on field
        """
        command = ManualGCCommand(manual_command=Command(type=command, for_team=team))

        # only set it if defined
        if ball_pos:
            command.final_ball_placement_point = ball_pos

        self.proto_unix_io.send_proto(ManualGCCommand, command)
