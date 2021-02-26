import numpy as np
from bokeh.plotting import Figure
from bokeh.models import ColumnDataSource, LabelSet
from bokeh import palettes
from typing import Iterable, Callable


def generate_heatmap(
    x_bounds: tuple,
    y_bounds: tuple,
    grid_size: float,
    heatmap_function: Callable[[float, float], float],
) -> np.ndarray:
    grid_dims = (
        int((max(x_bounds) - min(x_bounds)) // grid_size),
        int((max(y_bounds) - min(y_bounds)) // grid_size),
    )
    heatmap_grid = np.ndarray(grid_dims)

    xcoords = np.arange(min(x_bounds), max(x_bounds), grid_size)
    ycoords = np.arange(min(y_bounds), max(y_bounds), grid_size)

    for x_idx, x in enumerate(xcoords):
        for y_idx, y in enumerate(ycoords):
            if x_idx < grid_dims[0] and y_idx < grid_dims[1]:
                heatmap_grid[x_idx, y_idx] = heatmap_function(x, y)

    return np.flip(np.rot90(heatmap_grid), axis=0)


class HeatmapPlotter:
    def __init__(
        self,
        fig: Figure,
        x_bounds: tuple,
        y_bounds: tuple,
        grid_size: float,
        legend_label: str,
    ):
        self.image_data_source = ColumnDataSource(dict(image=[]))
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.grid_size = grid_size
        fig.image(
            source=self.image_data_source,
            image="image",
            x=min(x_bounds),
            y=min(y_bounds),
            dh=max(y_bounds) - min(y_bounds),
            dw=max(x_bounds) - min(x_bounds),
            palette=palettes.viridis(100),
            level="image",
            legend_label=legend_label,
        )

    def plot_heatmap(self, heatmap_function: Callable[[float, float], float]):
        heatmap = generate_heatmap(
            self.x_bounds, self.y_bounds, self.grid_size, heatmap_function
        )
        self.image_data_source.data.update(dict(image=[heatmap]))
