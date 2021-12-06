import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle
import pandas as pd

# Constants
robot_radius = 0.09
frame_rate = 200
frame_length_ms = 1000.0 / frame_rate
play_back_speed = 1.0
div_a_field_width = 9.0
div_a_field_length = 12.0
default_robot_color = "royalblue"
collided_robot_color = "red"


def animate_robots(robot_pos_df, gif_output_file=None):
    num_frames = robot_pos_df["frame"].max() + 1
    num_robots = robot_pos_df["robot_id"].max() + 1
    default_line_alpha = 0.3
    is_paused = False

    # Calculate plot size
    max_robot_x_pos = robot_pos_df["x"].max()
    min_robot_x_pos = robot_pos_df["x"].min()
    max_robot_y_pos = robot_pos_df["y"].max()
    min_robot_y_pos = robot_pos_df["y"].min()
    scale_constant = 1.0 / 8
    x_offset = max(scale_constant * (max_robot_x_pos - min_robot_x_pos), 1.0)
    y_offset = max(scale_constant * (max_robot_y_pos - min_robot_y_pos), 1.0)

    def setup_plot():
        """Initial drawing of the scatter plot."""
        # TODO: Make robot and tracking line the same color
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
                robot_list[robot_id].set_facecolor(collided_robot_color)

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
                robot_list[robot_id].set_facecolor(default_robot_color)

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
    fig.suptitle(file_name)
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

    # TODO: Allow highlighting after animation is over. Can achieve this by not passing frames to FunAnimation
    # Highlight tracked line on hover
    fig.canvas.mpl_connect("motion_notify_event", on_plot_hover)

    # Animate
    print(f"Animating robots from {file_location}")
    robot_anim = FuncAnimation(
        fig,
        func=update,
        interval=frame_length_ms / play_back_speed,
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
        robot_anim.save(gif_output_file, writer=PillowWriter(fps=frame_rate))
        print(f"Saved gif of robots to {gif_output_file}")


file_name = "1_robot_moving_in_square"
file_location = (
    f"/home/nima/thunderbots/Software/src/software/hrvo/hrvo_data/{file_name}.csv"
)
df = pd.read_csv(file_location)  # , skiprows=3)
animate_robots(
    df
)  # , f'/home/nima/thunderbots/Software/src/software/hrvo/hrvo_data/{file_name}.gif')
