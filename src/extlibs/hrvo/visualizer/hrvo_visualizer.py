import argparse
import os

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle
import pandas as pd

# Constants
FRAME_RATE = 30
FRAME_LENGTH_MS = 1000.0 / FRAME_RATE
PLAY_BACK_SPEED = 1.0

DEFAULT_ROBOT_COLOR = "royalblue"
COLLIDED_ROBOT_COLOR = "red"

FILE_DIRECTORY = "/tmp"


def animate_robots(file_loc, test_name, gif_output_file=None):
    robot_pos_df = pd.read_csv(file_loc)

    num_frames = robot_pos_df["frame"].max() + 1
    num_robots = robot_pos_df["robot_id"].max() + 1
    default_line_alpha = 0.3
    is_paused = False

    # Calculate plot size
    max_robot_x_pos = robot_pos_df["x"].max()
    min_robot_x_pos = robot_pos_df["x"].min()
    max_robot_y_pos = robot_pos_df["y"].max()
    min_robot_y_pos = robot_pos_df["y"].min()
    plot_scale_constant = 0.125
    x_offset = max(plot_scale_constant * (max_robot_x_pos - min_robot_x_pos), 1.0)
    y_offset = max(plot_scale_constant * (max_robot_y_pos - min_robot_y_pos), 1.0)

    def setup_plot():
        """Initial drawing of the scatter plot."""
        for robot in robot_list:
            ax.add_patch(robot)

        for line in line_list:
            line.set_data([], [])

        return robot_list + line_list + [time_text] + [robot_id_text]

    def update(frame):
        """Update plot for new frame"""
        if frame >= num_frames:
            return robot_list + line_list + [time_text] + [robot_id_text]

        # Current frame, dataframe filter
        curr_frame_df = robot_pos_df[robot_pos_df["frame"] == frame]

        # Update robot positions
        for robot_id in range(len(robot_list)):
            robot = curr_frame_df[curr_frame_df["robot_id"] == robot_id]
            robot_list[robot_id].center = (
                float(robot["x"].head(1)),
                float(robot["y"].head(1)),
            )

            # Update robot color if it collides with another robot
            other_robot_id = int(robot["has_collided"])
            if other_robot_id != -1:
                print(f"went into for loop with other robot {other_robot_id}\n{robot}")
                robot_list[robot_id].set_facecolor(COLLIDED_ROBOT_COLOR)

                if other_robot_id > robot_id:
                    other_robot = curr_frame_df[
                        curr_frame_df["robot_id"] == other_robot_id
                    ]
                    relative_vel_x = float(other_robot["velocity_x"]) - float(
                        robot["velocity_x"]
                    )
                    relative_vel_y = float(other_robot["velocity_y"]) - float(
                        robot["velocity_y"]
                    )
                    relative_speed = pow(
                        pow(relative_vel_x, 2) + pow(relative_vel_y, 2), 0.5
                    )
                    print(
                        f"robot {robot_id} and robot {other_robot_id} have collided with a relative velocity of {relative_speed}m/s"
                    )
            else:
                robot_list[robot_id].set_facecolor(DEFAULT_ROBOT_COLOR)

        # Update lines tracking the robot movement
        robots_grouped = robot_pos_df.groupby("robot_id")
        for robot_id, line in enumerate(line_list):
            robot_df = robots_grouped.get_group(robot_id)
            line.set_data(
                robot_df[robot_df["frame"] <= frame]["x"],
                robot_df[robot_df["frame"] <= frame]["y"],
            )

        # Update displayed time
        curr_time = float(curr_frame_df["time"].head(1))
        time_text.set_text(f"Time: {curr_time:.2f}sec")

        return robot_list + line_list + [time_text] + [robot_id_text]

    def toggle_pause(event):
        nonlocal is_paused
        if is_paused:
            robot_anim.resume()
        else:
            robot_anim.pause()
        is_paused = not is_paused

    def on_plot_hover(event):
        # Iterating over each data member plotted
        for robot_id, line in enumerate(line_list):
            # Searching which data member corresponds to current mouse position
            if line.contains(event)[0]:
                line.set_alpha(1.0)
                robot_id_text.set_text(f"Robot {robot_id}")
            else:
                line.set_alpha(default_line_alpha)

    # set up plot
    fig = plt.figure()
    fig.suptitle(test_name)
    ax = fig.add_subplot(
        111,
        autoscale_on=False,
        xlim=(min_robot_x_pos - x_offset, max_robot_x_pos + x_offset),
        ylim=(min_robot_y_pos - y_offset, max_robot_y_pos + y_offset),
    )
    ax.set_aspect("equal")
    ax.grid()
    (line,) = ax.plot([], [], "--r")
    # plt.subplots_adjust(left=0.3)
    time_text = ax.text(0.5, 0.2, "", transform=plt.gcf().transFigure)
    robot_id_text = ax.text(0.5, 0.1, "", transform=plt.gcf().transFigure)

    # set-up robot circles
    initial_frame = robot_pos_df[robot_pos_df["frame"] == 0]
    robot_list = []
    line_list = []
    for robot_id in range(num_robots):
        radius = float(
            initial_frame[initial_frame["robot_id"] == robot_id]["radius"].head(1)
        )
        robot = Circle((0, 0), radius, facecolor="aqua", edgecolor="black")
        robot_list.append(robot)
        line_obj = ax.plot([], [], linestyle="--", alpha=default_line_alpha)[0]
        line_list.append(line_obj)

    # Start/Stop on click
    fig.canvas.mpl_connect("button_press_event", toggle_pause)

    # Highlight tracked line on hover
    fig.canvas.mpl_connect("motion_notify_event", on_plot_hover)

    # Animate
    print(f"Animating robots from {test_name}")
    robot_anim = FuncAnimation(
        fig,
        func=update,
        interval=FRAME_LENGTH_MS / PLAY_BACK_SPEED,
        init_func=setup_plot,
        frames=num_frames,
        blit=True,
        repeat=False,
    )
    plt.show()

    if gif_output_file is not None:
        # save gif
        print("==============================================")
        print(f"Saving gif of robots to {gif_output_file}...")
        robot_anim.save(gif_output_file, writer=PillowWriter(fps=FRAME_RATE))
        print(f"Saved gif of robots to {gif_output_file}")


def validate_file(f):
    if not os.path.exists(f"{FILE_DIRECTORY}/{f}.csv"):
        raise argparse.ArgumentTypeError(f"{FILE_DIRECTORY}/{f}.csv does not exist")
    return f
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--test_name', dest='file_name', required=True, type=validate_file, help=f'The name of the .csv file which you want to animate. File must be stored under {FILE_DIRECTORY}.')
    parser.add_argument('-s', '--save_gif', dest='save_gif', action='store_true', help=f'If arg passed, animation will be saved as a GIF at {FILE_DIRECTORY}/test_name.gif.')
    args = parser.parse_args()

    csv_file_loc = f"{FILE_DIRECTORY}/{args.file_name}.csv"
    gif_file_location = None
    if args.save_gif:
        gif_file_location = f"{FILE_DIRECTORY}/{args.file_name}.gif"

    animate_robots(csv_file_loc, args.file_name, gif_file_location)