from pyqtgraph.Qt.QtCore import *
from software.thunderscope.field.world_layer import WorldLayer
from proto.import_all_protos import *
from software.py_constants import *

class SandboxWorldLayer(WorldLayer):

    def __init__(self, simulator_io, friendly_colour_yellow):
        WorldLayer.__init__(self, simulator_io, friendly_colour_yellow)

        self.robot_add_double_click = False
        self.next_id = 0

        self.update_world_state()

    def update_world_state(self, id, new_pos, friendly):
        world_state = WorldState()

        for robot_ in cached_world.friendly_team.team_robots:
            world_state.blue_robots[robot_.id].CopyFrom(
                RobotState(
                    global_position=Point(
                        x_meters=robot_.current_state.global_position.x_meters, y_meters=robot_.current_state.global_position.y_meters
                    ),
                )
            )

        for robot_ in cached_world.enemy_team.team_robots:
            world_state.yellow_robots[robot_.id].CopyFrom(
                RobotState(
                    global_position=Point(
                        x_meters=robot_.current_state.global_position.x_meters, y_meters=robot_.current_state.global_position.y_meters
                    ),
                )
            )

        new_robot_state = RobotState(
            global_position=Point(
                x_meters=new_pos[0], y_meters=new_pos[1]
            ),
        )
        
        if (friendly) {
            world_state.yellow_robots[id].CopyFrom(new_robot_state)
        } else {
            world_state.blue_robots[id].CopyFrom(new_robot_state)
        }

        self.simulator_io.send_proto(WorldState, world_state)

    def mousePressEvent(self, event):
        super().mousePressEvent(event)

        # determine whether a robot was clicked
        friendly_robot, enemy_robot = self.identify_robots(*self.mouse_click_pos)

        if not friendly_robot and not enemy_robot:
            if self.robot_add_double_click:
                self.update_world_state(self.next_id, [
                    self.mouse_click_pos[0] / MILLIMETERS_PER_METER,
                    self.mouse_click_pos[1] / MILLIMETERS_PER_METER
                ])
                self.next_id += 1
                self.robot_add_double_click = False
            else:
                self.robot_add_double_click = True
                QTimer.singleShot(500, self.toggle_robot_add_double_click)
        else:
            if friendly_robot:
                print(friendly_robot)
            else:
                print(enemy_robot)

    def toggle_robot_add_double_click(self):
        self.robot_add_double_click = not self.robot_add_double_click


