import os
import tomllib


class EmbeddedData:
    """Model class responsible for interfacing with onboard disk data on the robot.
    This class manages static data on the robot as well as the operations necessary with mutating data for use
    """

    def __init__(self) -> None:
        self.config_file_path = "/opt/tbotspython/robot_config.toml"
        self.config = self._load_config()
        self.epoch_timestamp_seconds = 0
        self.battery_voltage = 0
        self.primitive_packet_loss_percentage = 0
        self.primitive_executor_step_time_ms = 0

    def _load_config(self) -> dict:
        """Load TOML configuration file."""
        if not os.path.exists(self.config_file_path):
            return {}
        try:
            with open(self.config_file_path, "rb") as f:
                return tomllib.load(f)
        except Exception as e:
            print(
                f"Warning: Failed to load TOML config from {self.config_file_path}: {e}"
            )
            return {}

    def _get_value(self, key: str):
        """Get a value from the TOML file; TOML keys do not start with '/'."""
        normalized_key = key.lstrip("/")
        return self.config.get(normalized_key, "")

    def get_robot_id(self) -> str:
        return self._get_value(ROBOT_ID_CONFIG_KEY)

    def get_network_interface(self) -> str:
        return self._get_value(ROBOT_NETWORK_INTERFACE_CONFIG_KEY)

    def get_channel_id(self) -> str:
        return self._get_value(ROBOT_MULTICAST_CHANNEL_CONFIG_KEY)

    def get_kick_constant(self) -> str:
        return self._get_value(ROBOT_KICK_CONSTANT_CONFIG_KEY)

    def get_kick_coeff(self) -> str:
        return self._get_value(ROBOT_KICK_EXP_COEFF_CONFIG_KEY)

    def get_chip_pulse_width(self) -> str:
        return self._get_value(ROBOT_CHIP_PULSE_WIDTH_CONFIG_KEY)

    def get_current_draw(self) -> str:
        return self._get_value(ROBOT_CURRENT_DRAW_CONFIG_KEY)

    def get_battery_volt(self) -> str:
        return self._get_value(ROBOT_BATTERY_VOLTAGE_CONFIG_KEY)

    def get_cap_volt(self) -> str:
        return self._get_value(ROBOT_CAPACITOR_VOLTAGE_CONFIG_KEY)
