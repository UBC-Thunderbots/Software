from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar
from software.thunderscope.proto_unix_io import ProtoUnixIO
from pyqtgraph.Qt.QtWidgets import QMenu, QLineEdit
from software.thunderscope.constants import SandboxModeConstants
from pyqtgraph.Qt import QtGui
from proto.import_all_protos import *
import software.python_bindings as tbots_cpp
import os
import qtawesome as qta

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


class GLSandboxStateToolbar:
    def __init__(
        self, toolbar: GLToolbar, proto_unix_io: ProtoUnixIO, buffer_size: int = 5
    ):
        self.__toolbar = toolbar
        self.proto_unix_io = proto_unix_io
        self.cached_world = None

        self.world_buffer = ThreadSafeBuffer(buffer_size, World)

        self.state_name_textbox = QLineEdit()
        self.state_name_textbox.setPlaceholderText("Enter State Name...")

        self.save_state_button = self.__toolbar.setup_icon_button(
            qta.icon("mdi6.download"),
            "Saves the current state of the field",
            self.__save_field_state,
            display_text="Save Field",
        )

        self.select_state_menu_button = self.__toolbar.setup_icon_button(
            qta.icon("mdi6.server"),
            "Select a field state to load",
            display_text="Load State",
        )

        self.state_menu = QMenu()
        self.__refresh_state_options()

        toolbar.layout().addWidget(self.state_name_textbox)
        toolbar.add_button(self.save_state_button)
        toolbar.add_button(self.select_state_menu_button)

    def refresh(self):
        try:
            world_msg = self.world_buffer.get(block=False, return_cached=True)
            self.cached_world = tbots_cpp.World(world_msg)
        except Exception:
            pass

        if not self.cached_world or not self.state_name_textbox.text():
            self.save_state_button.setEnabled(False)

    def __save_field_state(self):
        saved_states_dir = SandboxModeConstants.SAVED_STATES_PATH

        os.makedirs(saved_states_dir, exist_ok=True)

        friendly_team = self.cached_world.friendlyTeam()
        enemy_team = self.cached_world.enemyTeam()

        state_to_save = (
            f"{self.__team_to_toml(friendly_team, SandboxModeConstants.SAVED_STATE_BLUE_TEAM)}\n"
            f"{self.__team_to_toml(enemy_team, SandboxModeConstants.SAVED_STATE_YELLOW_TEAM)}\n"
        )

        state_name = self.state_name_textbox.text()

        saved_state_path = os.path.join(saved_states_dir, f"{state_name}.toml")

        with open(saved_state_path, "w") as state:
            state.write(state_to_save)

    def __team_to_toml(self, team: tbots_cpp.Team, key: str) -> str:
        return f"{key} = {[
            self.__robot_to_toml(robot) for robot in team.getAllRobots()
        ]}"

    def __robot_to_toml(self, robot: tbots_cpp.Robot) -> str:
        position = robot.position()
        velocity = robot.velocity()
        orientation = robot.orientation()
        angular_velocity = robot.angular_velocity()

        return f'{{ {', '.join(
            [
                f"{SandboxModeConstants.TeamKeys.LOCATION} = {[position.x(), position.y()]}",
                f"{SandboxModeConstants.TeamKeys.VELOCITY} = {[velocity.x(), velocity.y()]}",
                f"{SandboxModeConstants.TeamKeys.ORIENTATION} = {orientation.toRadians()}",
                f"{SandboxModeConstants.TeamKeys.ANGULAR_VELOCITY} = {angular_velocity.toRadians()}",
            ]
        )} }}'

    def __load_saved_states(self):
        saved_states_path = SandboxModeConstants.SAVED_STATES_PATH

        if not os.path.isdir(saved_states_path):
            return []

        dir_entries = os.listdir(saved_states_path)
        return [
            file_name
            for file_name in dir_entries
            if os.path.isfile(os.path.join(saved_states_path, file_name))
        ]

    def __load_state_to_field(self, state_file_name):
        saved_states_dir = SandboxModeConstants.SAVED_STATES_PATH
        state_path = os.path.join(saved_states_dir, state_file_name)

        try:
            with open(state_path, "rb") as file:
                state_dict = tomllib.load(file)

                # Load Blue team

                # Load Yellow team

        except (FileNotFoundError, PermissionError, TOMLDecodeError):
            logging.warning(
                f"Failed to read TOML file at: {RuntimeManagerConstants.RUNTIME_CONFIG_PATH}"
            )

    def __build_team(self, robot_positions: list[list[int]]):
        return (
            tbots_cpp.Team(
                [
                    tbots_cpp.Robot(
                        index,
                        location,
                        tbots_cpp.Vector(0.0, 0.0),
                        tbots_cpp.Angle.fromRadians(0),
                        tbots_cpp.Angle(),
                        tbots_cpp.Timestamp(),
                    )
                    for index, location in enumerate(blue_robot_locations)
                ]
            ),
        )

    def __refresh_state_options(self):
        saved_states = self.__load_saved_states()

        self.state_menu.clear()

        for file_name in saved_states:
            action = QtGui.QAction(file_name)
            action.triggered.connect(lambda _: self.__load_state_to_field(file_name))
            self.state_menu.addAction(action)
