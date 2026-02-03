from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar
from software.thunderscope.proto_unix_io import ProtoUnixIO
from pyqtgraph.Qt.QtWidgets import QToolButton, QMenu
from software.thunderscope.constants import SandboxModeConstants
from pyqtgraph.Qt import QtGui
import os

class GLSandboxStateToolbar:

    def __init__(
        self,
        toolbar: GLToolbar,
        proto_unix_io: ProtoUnixIO,
    ):
        self.__toolbar = toolbar
        self.proto_unix_io = proto_unix_io
        self.cached_world = None
        
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
        
        toolbar.add_button(self.save_state_button)
        toolbar.add_button(self.select_state_menu_button)
        
    def refresh():
        self.cached_world = self.world_buffer.get(block=False, return_cached=True)
        
    def __save_field_state(self):   
        saved_states_dir = SandboxModeConstants.SAVED_STATES_PATH
        
        os.makedirs(saved_states_dir, exist_ok=True)
        
        
          
        pass
        
    def __load_saved_states(self):
        saved_states_path = SandboxModeConstants.SAVED_STATES_PATH
        
        if not os.path.isdir(saved_states_path):
            return []
        
        dir_entries = os.listdir(saved_states_path)
        return [file_name for file_name in dir_entries if os.path.isfile(os.path.join(directory_path, f))]
    
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
        return tbots_cpp.Team(
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
        
    def __refresh_state_options(self):
        saved_states = self.__load_saved_states()
        
        self.state_menu.clear()
        
        for file_name in saved_states:
            action = QtGui.QAction(file_name)
            action.triggered.connect(lambda _: self.__load_state_to_field(file_name))
            self.state_menu.addAction(action)
        
        