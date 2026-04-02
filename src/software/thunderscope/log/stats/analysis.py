import numpy as np
import pandas as pd
import pyqtgraph as pg
from pyqtgraph.exporters import ImageExporter

from software.py_constants import DIV_B_NUM_ROBOTS
from software.thunderscope.constants import RuntimeManagerConstants

CSV_PATH = f"{RuntimeManagerConstants.RUNTIME_EVENTS_DIRECTORY_PATH}/{RuntimeManagerConstants.RUNTIME_EVENTS_FILE}"
OUTPUT_DIR = RuntimeManagerConstants.RUNTIME_EVENTS_DIRECTORY_PATH

COLUMNS = {
    "ball": ["ball_x", "ball_y", "ball_vx", "ball_vy"],
    "robot_attrs": ["x", "y", "orientation", "vx", "vy", "angular_velocity"],
    "event_type": ["event_type"],
    "timestamp": ["timestamp"],
}

NUM_ROBOTS_PER_TEAM = DIV_B_NUM_ROBOTS


def load_raw_data(path: str) -> pd.DataFrame:
    """Load raw CSV data and assign column names.

    :param path: Path to the CSV file.
    :return: DataFrame with named columns for event_type, timestamp, ball state,
             and robot state (6 robots per team, zero-indexed).
    """
    cols = (
        COLUMNS["event_type"]
        + COLUMNS["timestamp"]
        + COLUMNS["ball"]
        + [f"friendly_{i // 6}_{COLUMNS['robot_attrs'][i % 6]}" for i in range(36)]
        + [f"enemy_{i // 6}_{COLUMNS['robot_attrs'][i % 6]}" for i in range(36)]
    )
    df = pd.read_csv(path, header=None)
    df.columns = cols
    return df


def extract_events(df: pd.DataFrame) -> pd.DataFrame:
    """Extract event data with ball state.

    :param df: Raw DataFrame from load_raw_data().
    :return: DataFrame with columns: event_type, timestamp, ball_x, ball_y,
             ball_vx, ball_vy.
    """
    return df[COLUMNS["event_type"] + COLUMNS["timestamp"] + COLUMNS["ball"]].copy()


def extract_robots(df: pd.DataFrame) -> pd.DataFrame:
    """Extract robot state data into long format.

    :param df: Raw DataFrame from load_raw_data().
    :return: DataFrame with columns: timestamp, robot_index (0-5), team, x, y,
             orientation, vx, vy, angular_velocity. Each row represents one robot
             at one timestamp.
    """
    records = []
    for team in ["friendly", "enemy"]:
        for robot_idx in range(NUM_ROBOTS_PER_TEAM):
            cols = [f"{team}_{robot_idx}_{attr}" for attr in COLUMNS["robot_attrs"]]
            records.append(
                pd.DataFrame(
                    {
                        "timestamp": df["timestamp"],
                        "robot_index": robot_idx,
                        "team": team,
                        **{
                            attr: df[col]
                            for attr, col in zip(COLUMNS["robot_attrs"], cols)
                        },
                    }
                )
            )
    return pd.concat(records, ignore_index=True)


def plot_shots(events_df: pd.DataFrame, output_path: str) -> None:
    """Plot cumulative shots on goal over time.

    :param events_df: DataFrame from extract_events().
    :param output_path: Path to save the PNG plot.
    """
    shots = events_df[events_df["event_type"] == "shot_on_goal"].copy()
    shots = shots.sort_values("timestamp")
    shots["time_relative_ms"] = shots["timestamp"] - shots["timestamp"].min()
    shots["cumulative"] = range(1, len(shots) + 1)

    pg.setConfigOption("antialias", True)
    plot = pg.plot()

    plot.plot(
        shots["time_relative_ms"].values / 1000,
        shots["cumulative"].values,
        pen={"color": "g", "width": 2},
        symbol="o",
        symbolSize=8,
        symbolBrush=pg.mkBrush("g"),
    )

    plot.setLabel("bottom", "Time (seconds)")
    plot.setLabel("left", "Total Shots")
    plot.setTitle("Cumulative Shots on Goal")

    ImageExporter(plot.plotItem).export(output_path)
    print(f"Saved plot to {output_path}")


def plot_robot_heatmap(robots_df: pd.DataFrame, output_path: str) -> None:
    """Plot a heatmap of robot positions on the field.

    SSL Division B field dimensions:
    - Field: 9.0m x 6.0m
    """
    x = robots_df["x"].to_numpy()
    y = robots_df["y"].to_numpy()

    field_x_min, field_x_max = -4.5, 4.5
    field_y_min, field_y_max = -3.0, 3.0

    bins = 80

    heatmap, xedges, yedges = np.histogram2d(
        x,
        y,
        bins=bins,
        range=[[field_x_min, field_x_max], [field_y_min, field_y_max]],
    )

    pg.setConfigOption("antialias", True)

    plot_widget = pg.PlotWidget()
    plot_item = plot_widget.getPlotItem()

    img = pg.ImageItem(heatmap.T)

    img.setRect(
        field_x_min,
        field_y_min,
        field_x_max - field_x_min,
        field_y_max - field_y_min,
    )

    # Apply colormap
    cmap = pg.colormap.get("viridis")
    img.setColorMap(cmap)

    plot_item.addItem(img)

    plot_item.setLabel("bottom", "X (meters)")
    plot_item.setLabel("left", "Y (meters)")
    plot_item.setTitle("Robot Position Heatmap")

    plot_item.setXRange(field_x_min, field_x_max)
    plot_item.setYRange(field_y_min, field_y_max)

    plot_item.showGrid(x=True, y=True)

    # ---- sanity check point ----
    sanity_x = 1
    sanity_y = 0

    plot_item.plot(
        [sanity_x],
        [sanity_y],
        pen=None,
        symbol="o",
        symbolSize=10,
        symbolBrush="red",
    )

    # label it so you know what you're looking at
    text = pg.TextItem("(1,0)", anchor=(0, 1))
    text.setPos(sanity_x, sanity_y)
    plot_item.addItem(text)
    # ----------------------------

    exporter = ImageExporter(plot_item)
    exporter.export(output_path)

    print(f"Saved heatmap to {output_path}")


def main() -> None:
    """Run exploratory analysis on game events data."""
    df = load_raw_data(CSV_PATH)
    events_df = extract_events(df)
    robots_df = extract_robots(df)

    print()
    print("Events summary:")
    print()
    print(events_df["event_type"].value_counts().to_string())
    print()

    plot_shots(events_df, f"{OUTPUT_DIR}/shots_over_time.png")
    plot_robot_heatmap(robots_df, f"{OUTPUT_DIR}/robot_heatmap.png")


if __name__ == "__main__":
    main()
