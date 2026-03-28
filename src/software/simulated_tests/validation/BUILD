package(default_visibility = ["//visibility:public"])

py_library(
    name = "validations",
    srcs = [
        "avoid_collisions.py",
        "ball_enters_region.py",
        "ball_is_off_ground.py",
        "ball_kicked_in_direction.py",
        "ball_moves_in_direction.py",
        "ball_speed_threshold.py",
        "ball_stops_in_region.py",
        "excessive_dribbling.py",
        "friendly_has_ball_possession.py",
        "friendly_receives_ball_slow.py",
        "friendly_team_scored.py",
        "or_validation.py",
        "robot_enters_placement_region.py",
        "robot_enters_region.py",
        "robot_enters_region_and_stops.py",
        "robot_speed_threshold.py",
        "speed_threshold_helpers.py",
        "validation.py",
    ],
    data = [
        "//software:py_constants.so",
        "//software:python_bindings.so",
    ],
    deps = [
        "//proto:software_py_proto",
        "//proto:tbots_py_proto",
        "//software/logger:py_logger",
    ],
)
