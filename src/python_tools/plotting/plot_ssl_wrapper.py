import numpy as np
from bokeh.plotting import figure
from bokeh.models import ColumnDataSource, LabelSet
from software.proto.messages_robocup_ssl_geometry_pb2 import (
    SSL_FieldCircularArc,
    SSL_FieldLineSegment,
)
from software.proto.messages_robocup_ssl_detection_pb2 import (
    SSL_DetectionBall,
    SSL_DetectionRobot,
)
from software.proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from typing import Iterable

ROBOT_MAX_RADIUS = 0.09
MM_PER_M = 1000
BALL_RADIUS = 0.02


def plot_circular_section(
    fig: figure,
    center_x: float,
    center_y: float,
    radius: float,
    start_angle: float,
    end_angle: float,
):
    """
    Draw a circular section on to the given figure by drawing line segments between points sampled from the circle at
    regular angle intervals between the start angle and end angle
    :param fig: a Bokeh figure
    :param center_x: x of the center of the circle
    :param center_y: y of the center of the circle
    :param radius: radius of the circle
    :param start_angle: angle of the start of the circular section
    :param end_angle: angle of the end of the circular section
    """
    # decide how many samples to take along the arc, a full circle will be a 36-gon
    num_samples = (end_angle - start_angle) * (18 / np.pi)
    sample_angles = np.linspace(
        start_angle, end_angle, np.ceil(num_samples).astype(int)
    )
    sampled_pt_xs = []
    sampled_pt_ys = []

    for i in range(len(sample_angles) - 1):
        sampled_pt_xs.append(
            [
                center_x + np.cos(sample_angles[i]) * radius,
                center_x + np.cos(sample_angles[i + 1]) * radius,
            ]
        )
        sampled_pt_ys.append(
            [
                center_y + np.sin(sample_angles[i]) * radius,
                center_y + np.sin(sample_angles[i + 1]) * radius,
            ]
        )

    fig.multi_line(sampled_pt_xs, sampled_pt_ys, line_color="black")


def plot_robots(
    fig: figure,
    ssl_detectionrobots_protos: Iterable[SSL_DetectionRobot],
    colour: str,
    label_robot_ids: bool = True,
):
    """
    Plot the given robots onto the given figure.
    :param fig: a Bokeh figure
    :param ssl_detectionrobots_protos: the SSL_DetectionRobot protos to plot
    :param colour: colour of the robots
    :param label_robot_ids: whether to label the robot IDs on top of the robots
    """
    robot_xs = np.asarray([robot.x for robot in ssl_detectionrobots_protos]) / MM_PER_M
    robot_ys = np.asarray([robot.y for robot in ssl_detectionrobots_protos]) / MM_PER_M
    robot_oris = [robot.orientation for robot in ssl_detectionrobots_protos]

    # draw a line segment from the center of the robot to the perimeter in the direction
    # of the robot's orientation
    robot_ori_line_seg_xs = np.asarray(
        [
            [x, x + np.cos(ori) * ROBOT_MAX_RADIUS]
            for ori, x in zip(robot_oris, robot_xs)
        ]
    )

    robot_ori_line_seg_ys = np.asarray(
        [
            [y, y + np.sin(ori) * ROBOT_MAX_RADIUS]
            for ori, y in zip(robot_oris, robot_ys)
        ]
    )

    fig.circle(
        robot_xs,
        robot_ys,
        radius=ROBOT_MAX_RADIUS,
        fill_color=colour,
        line_color="black",
    )
    fig.multi_line(
        robot_ori_line_seg_xs.tolist(),
        robot_ori_line_seg_ys.tolist(),
        line_color="black",
    )

    if label_robot_ids:
        # draw labels for the robot ids
        robot_id_strs = [str(robot.robot_id) for robot in ssl_detectionrobots_protos]
        label_data_source = ColumnDataSource(
            data=dict(xs=robot_xs, ys=robot_ys, ids=robot_id_strs)
        )

        labels = LabelSet(
            x="xs", y="ys", text="ids", source=label_data_source, text_font_size="12pt"
        )
        fig.add_layout(labels)


def plot_balls(fig: figure, ssl_detectionballs: Iterable[SSL_DetectionBall]):
    """
    Plot the given balls onto the given Bokeh figure.
    :param fig: a Bokeh figure
    :param ssl_detectionballs: the SSL_DetectionBall protos to plot
    """
    ball_xs = np.asarray([ball.x for ball in ssl_detectionballs]) / MM_PER_M
    ball_ys = np.asarray([ball.y for ball in ssl_detectionballs]) / MM_PER_M

    fig.circle(ball_xs, ball_ys, radius=0.02, fill_color="orange", line_color="black")


def plot_field_line_segments(
    fig: figure, field_lines_proto: Iterable[SSL_FieldLineSegment]
):
    """
    Plot the given field line segments onto the given Bokeh figure.
    :param fig: a Bokeh figure
    :param field_lines_proto: an Iterable of SSL_FieldLineSegment
    :return:
    """
    field_line_xs = (
        np.asarray([[line.p1.x, line.p2.x] for line in field_lines_proto]) / MM_PER_M
    )
    field_line_ys = (
        np.asarray([[line.p1.y, line.p2.y] for line in field_lines_proto]) / MM_PER_M
    )
    fig.multi_line(
        xs=field_line_xs.tolist(), ys=field_line_ys.tolist(), line_color="black"
    )


def plot_field_circular_arcs(
    fig, field_circular_arcs_proto: Iterable[SSL_FieldCircularArc]
):
    for arc in field_circular_arcs_proto:
        plot_circular_section(
            fig,
            arc.center.x / MM_PER_M,
            arc.center.y / MM_PER_M,
            arc.radius / MM_PER_M,
            arc.a1,
            arc.a2,
        )


def plot_ssl_wrapperpacket(
    fig: figure, ssl_wrapperpacket: SSL_WrapperPacket, label_robot_ids: bool = True
):
    """
    Plot the given SSL_WrapperPacket representing the state of the entire world onto the
    given Bokeh figure.
    :param fig: a Bokeh figure
    :param ssl_wrapperpacket: an SSL_WrapperPacket proto to plot the world state from
    :param label_robot_ids: whether to label robot IDs on top of the robots
    """
    if ssl_wrapperpacket.HasField("detection"):
        plot_robots(
            fig,
            ssl_wrapperpacket.detection.robots_yellow,
            "yellow",
            label_robot_ids=label_robot_ids,
        )
        plot_robots(
            fig,
            ssl_wrapperpacket.detection.robots_blue,
            "cyan",
            label_robot_ids=label_robot_ids,
        )
        plot_balls(fig, ssl_wrapperpacket.detection.balls)
    if ssl_wrapperpacket.HasField("geometry"):
        plot_field_line_segments(fig, ssl_wrapperpacket.geometry.field.field_lines)
        plot_field_circular_arcs(fig, ssl_wrapperpacket.geometry.field.field_arcs)
