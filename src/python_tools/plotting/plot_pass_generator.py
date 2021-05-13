from bokeh.plotting import Figure
from bokeh.models import ColumnDataSource, ColorBar, LinearColorMapper
from bokeh.models.tools import HoverTool
from bokeh.palettes import mpl
import numpy as np


class PassGeneratorPlotter:
    def plot_zones(self, zones: list):
        """
        Plots pass generator zones from the Python bindings as per
        software/python_bindings/pass_generator.cpp.
        :param zones: A list of dicts describing rectangular passing zones.
        """
        zones_dict = dict(center_xs=[], center_ys=[], widths=[], heights=[])

        for zone in zones:
            zones_dict["center_xs"].append(zone.centre().x())
            zones_dict["center_ys"].append(zone.centre().y())
            zones_dict["widths"].append(zone.xLength())
            zones_dict["heights"].append(zone.yLength())

        self.zones_source.data.update(zones_dict)

    def plot_passes(self, passes: list):
        """
        Plots dicts describing passes from Python bindings as per
        software/python_bindings/pass_utilities.cpp.
        :param passes: A list of dicts describing passes and their associated ratings.
        """
        passes_dict = dict(
            receiver_xs=[],
            receiver_ys=[],
            pass_line_xs=[],
            pass_line_ys=[],
            pass_rating=[],
            line_width=[],
        )

        for pass_and_rating_dict in passes:
            the_pass = pass_and_rating_dict["pass"]
            rating = pass_and_rating_dict["rating"]

            passer_point = the_pass["passer_point"]
            receiver_point = the_pass["receiver_point"]

            passes_dict["receiver_xs"].append(receiver_point.x())
            passes_dict["receiver_ys"].append(receiver_point.y())

            passes_dict["pass_line_xs"].append([passer_point.x(), receiver_point.x()])
            passes_dict["pass_line_ys"].append([passer_point.y(), receiver_point.y()])

            passes_dict["pass_rating"].append(rating)

            # line width is 1 + pass_rating
            passes_dict["line_width"].append(1 + rating)

        self.passes_source.data.update(passes_dict)

    def __init__(self, fig: Figure):
        """
        Constructs a new PassGeneratorPlotter associated to fig.
        :param fig: A Bokeh figure
        """
        self.fig = fig

        self.zones_source = ColumnDataSource(
            dict(center_xs=[], center_ys=[], widths=[], heights=[])
        )
        self.fig.rect(
            source=self.zones_source,
            x="center_xs",
            y="center_ys",
            width="widths",
            height="heights",
            fill_alpha=0,
            line_color="blue",
            line_width=1.2,
            legend_label="Pass Generator zones",
        )

        self.passes_source = ColumnDataSource(
            dict(
                receiver_xs=[],
                receiver_ys=[],
                pass_line_xs=[],
                pass_line_ys=[],
                pass_rating=[],
                line_width=[],
            )
        )

        self.pass_color_mapper = LinearColorMapper(
            palette=mpl["Plasma"][256], low=0, high=1
        )
        color_bar = ColorBar(
            color_mapper=self.pass_color_mapper,
            width=8,
            height=400,
            location="bottom_right",
            title="Pass rating",
        )
        self.fig.add_layout(color_bar, "right")

        self.fig.multi_line(
            source=self.passes_source,
            xs="pass_line_xs",
            ys="pass_line_ys",
            line_width="line_width",
            line_color=dict(field="pass_rating", transform=self.pass_color_mapper),
        )

        receiver_point_glyph = self.fig.circle_cross(
            source=self.passes_source,
            x="receiver_xs",
            y="receiver_ys",
            size=10,
            fill_alpha=0.0,
            legend_label="Receiver points",
        )

        pass_rating_tooltip = ("Pass rating", "@pass_rating")
        hover_tool = HoverTool(
            tooltips=[pass_rating_tooltip], renderers=[receiver_point_glyph]
        )
        fig.add_tools(hover_tool)
