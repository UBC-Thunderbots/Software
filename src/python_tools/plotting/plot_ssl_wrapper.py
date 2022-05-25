import numpy as np
from bokeh.plotting import figure, Figure
from bokeh.models import ColumnDataSource, LabelSet
from proto.ssl_vision_geometry_pb2 import (
    SSL_FieldCircularArc,
    SSL_FieldLineSegment,
)
from proto.ssl_vision_detection_pb2 import (
    SSL_DetectionBall,
    SSL_DetectionRobot,
)
from proto.ssl_vision_wrapper_pb2 import SSL_WrapperPacket
from typing import Iterable

ROBOT_MAX_RADIUS = 0.09
MM_PER_M = 1000
BALL_RADIUS = 0.02


def circular_arc_to_line_segs(arc: SSL_FieldCircularArc):
    """
    Sample line segments from a circular section described by a SSL_FieldCircularArc proto and return them as a tuple
    of a list of list of line segment x's, and a list of list of line segment y's

    return will look like:
    (
        [[start x, end x],
         ...
         [start x, end x]],

        [[start y, end y],
         ...
         [start y, end y]]
    )

    :param fig: a Bokeh figure
    :param center_x: x of the center of the circle
    :param center_y: y of the center of the circle
    :param radius: radius of the circle
    :param start_angle: angle of the start of the circular section
    :param end_angle: angle of the end of the circular section
    :return a tuple of list[list[x]], list[list[y]], a series of line segment x's and y's representing LINE SEGMENTS
            sampled along the circular arc
    """
    center_x = arc.center.x
    center_y = arc.center.y
    radius = arc.radius
    start_angle = arc.a1
    end_angle = arc.a2

    # decide how many samples to take along the arc, a full circle will be a 36-gon
    num_samples = (end_angle - start_angle) * (18 / np.pi)
    sample_angles = np.linspace(
        start_angle, end_angle, np.ceil(num_samples).astype(int)
    )
    sampled_pt_xs = []
    sampled_pt_ys = []

    for i in range(len(sample_angles) - 1):
        # sometimes we have to convert to numpy arrays, do the MM_PER_M division, and then convert it back because
        # bokeh really dislikes 2d numpy arrays in some circumstances
        sampled_pt_xs.append(
            (
                np.asarray(
                    [
                        center_x + np.cos(sample_angles[i]) * radius,
                        center_x + np.cos(sample_angles[i + 1]) * radius,
                    ]
                )
                / MM_PER_M
            ).tolist()
        )
        sampled_pt_ys.append(
            (
                np.asarray(
                    [
                        center_y + np.sin(sample_angles[i]) * radius,
                        center_y + np.sin(sample_angles[i + 1]) * radius,
                    ]
                )
                / MM_PER_M
            ).tolist()
        )
    return sampled_pt_xs, sampled_pt_ys


def get_robots_data(ssl_detectionrobots_protos: Iterable[SSL_DetectionRobot]):
    """
    Extract data from SSL_DetectionRobot protos into a dict with field names according to the SSLWrapperPlotter
    ColumnDataSource for robots
    :param ssl_detectionrobots_protos: the SSL_DetectionRobot protos to plot
    :return a dict that can be assigned to a ColumnDataSource for robot plotters as per SSLWrapperPlotter
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

    robot_id_strs = [str(robot.robot_id) for robot in ssl_detectionrobots_protos]

    return dict(
        robot_xs=robot_xs,
        robot_ys=robot_ys,
        robot_ori_line_seg_xs=robot_ori_line_seg_xs.tolist(),
        robot_ori_line_seg_ys=robot_ori_line_seg_ys.tolist(),
        robot_ids=robot_id_strs,
    )


def get_balls_data(ssl_detectionballs: Iterable[SSL_DetectionBall]):
    """
    Extract data from SSL_DetectionBall protos into a dict with field names according to the SSLWrapperPlotter
    ColumnDataSource for balls
    :param ssl_detectionballs: the SSL_DetectionBall protos to plot
    :return a dict that can be assigned to a ColumnDataSource for the ball plotter as per SSLWrapperPlotter
    """
    ball_xs = np.asarray([ball.x for ball in ssl_detectionballs]) / MM_PER_M
    ball_ys = np.asarray([ball.y for ball in ssl_detectionballs]) / MM_PER_M

    return dict(ball_xs=ball_xs, ball_ys=ball_ys)


def get_field_lines_data(
    field_lines_proto: Iterable[SSL_FieldLineSegment],
    field_circular_arcs_proto: Iterable[SSL_FieldCircularArc],
):
    """
    Extract data from SSL_FieldLineSegment and  SSL_FieldCircularArc protos into a dict with field names according
    to the SSLWrapperPlotter ColumnDataSource for field line segments
    :param field_lines_proto: the field line protos to plot
    :param field_circular_arcs_proto: the field circular arc protos to plot
    :return a dict that can be assigned to a ColumnDataSource for the field line segments plotter as per SSLWrapperPlotter
    """
    # sometimes we have to convert to numpy arrays, do the MM_PER_M division, and then convert it back because
    # bokeh really dislikes 2d numpy arrays in some circumstances
    field_line_xs = (
        np.asarray([[line.p1.x, line.p2.x] for line in field_lines_proto]) / MM_PER_M
    ).tolist()
    field_line_ys = (
        np.asarray([[line.p1.y, line.p2.y] for line in field_lines_proto]) / MM_PER_M
    ).tolist()

    # convert all the field circular arcs to multilines and append them
    for circular_arc in field_circular_arcs_proto:
        arc_xs, arc_ys = circular_arc_to_line_segs(circular_arc)
        field_line_xs.extend(arc_xs)
        field_line_ys.extend(arc_ys)

    return dict(field_line_xs=field_line_xs, field_line_ys=field_line_ys)


class SSLWrapperPlotter:
    def __setup_robot_plotters(self, robot_colour, robot_plot_colour):
        """
        A helper function to set circles, orientation lines, and labels for a given robot colour and a corresponding
        plot colour.
        :param robot_colour: Robot colour to set the legend label and robot_sources dict key with
        :param robot_plot_colour: Colour to actually plot the robot circles with
        """
        # blue robots
        self.robots_sources[robot_colour] = ColumnDataSource(
            dict(
                robot_ids=[],
                robot_xs=[],
                robot_ys=[],
                robot_ori_line_seg_xs=[[]],
                robot_ori_line_seg_ys=[[]],
            )
        )
        # circles representing the robots
        self.fig.circle(
            source=self.robots_sources[robot_colour],
            x="robot_xs",
            y="robot_ys",
            radius=ROBOT_MAX_RADIUS,
            fill_color=robot_plot_colour,
            line_color="black",
            legend_label=robot_colour + " robots",
        )
        # line segments representing robot orientations
        self.fig.multi_line(
            source=self.robots_sources[robot_colour],
            xs="robot_ori_line_seg_xs",
            ys="robot_ori_line_seg_ys",
            line_color="black",
            legend_label=robot_colour + " robot orientations",
        )
        # labels for the robot ids
        labels = LabelSet(
            x="robot_xs",
            y="robot_ys",
            text="robot_ids",
            source=self.robots_sources[robot_colour],
            text_font_size="12pt",
        )
        self.fig.add_layout(labels)

    def __init__(self, fig: Figure):
        """
        Create a new SSL_WrapperPacket plotter associated to the given figure. This function sets up all the
        ColumnDataSource's needed to live update the bokeh figure when we plot different SSL_WrapperPackets.
        :param fig: a bokeh Figure
        """
        self.fig = fig
        # field lines
        self.field_lines_source = ColumnDataSource(
            dict(field_line_xs=[[]], field_line_ys=[[]])
        )
        self.fig.multi_line(
            source=self.field_lines_source,
            xs="field_line_xs",
            ys="field_line_ys",
            line_color="black",
            legend_label="Field lines",
        )

        # ball
        self.ball_source = ColumnDataSource(dict(ball_xs=[], ball_ys=[]))
        self.fig.circle(
            source=self.ball_source,
            x="ball_xs",
            y="ball_ys",
            radius=0.02,
            fill_color="orange",
            line_color="black",
            legend_label="Balls",
        )

        # robots
        self.robots_sources = dict()
        self.__setup_robot_plotters("blue", "cyan")
        self.__setup_robot_plotters("yellow", "yellow")

    def plot_ssl_wrapper(self, ssl_wrapper: SSL_WrapperPacket):
        if ssl_wrapper.HasField("detection"):
            self.robots_sources["blue"].data.update(
                get_robots_data(ssl_wrapper.detection.robots_blue)
            )
            self.robots_sources["yellow"].data.update(
                get_robots_data(ssl_wrapper.detection.robots_yellow)
            )
            self.ball_source.data.update(get_balls_data(ssl_wrapper.detection.balls))
        if ssl_wrapper.HasField("geometry"):
            self.field_lines_source.data.update(
                get_field_lines_data(
                    ssl_wrapper.geometry.field.field_lines,
                    ssl_wrapper.geometry.field.field_arcs,
                )
            )
