from tomllib import TOMLDecodeError
from software.thunderscope.constants import RuntimeManagerConstants
from dataclasses import dataclass
import os
import tomllib
import logging


@dataclass
class RuntimeConfig:
    """Data class to store the paths of the two binaries"""

    chosen_blue_path: str = RuntimeManagerConstants.DEFAULT_BINARY_PATH
    """Blue runtime path"""

    chosen_yellow_path: str = RuntimeManagerConstants.DEFAULT_BINARY_PATH
    """Yellow runtime path"""


class RuntimeLoader:
    """Delegate class for handling local runtimes and managing runtime selection"""

    def fetch_installed_runtimes(self) -> list[str]:
        """Fetches the list of available runtimes, including our FullSystem, from the local disk. Makes the folder
        in our local disk if it does not exist yet.
        :return: A list of names for available runtimes, or just a list with our FullSystem if no available runtimes
        could be found
        """
        runtime_list = [RuntimeManagerConstants.DEFAULT_BINARY_NAME]

        if not os.path.isdir(RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH):
            os.mkdir(RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH)
        # Check for all executable files in the folder
        for file_name in os.listdir(RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH):
            file_path = os.path.join(
                RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH, file_name
            )
            if os.access(file_path, os.X_OK):
                runtime_list.append(file_name)

        # Cache external runtimes
        return runtime_list

    def load_existing_runtimes(self, yellow_runtime: str, blue_runtime: str) -> None:
        """Loads the yellow and blue runtimes specified by saving them in the local disk.
        :param blue_runtime: Unique name of the blue runtime to set
        :param yellow_runtime: Unique name of the yellow runtime to set
        """
        config = RuntimeConfig(
            self._return_runtime_path(blue_runtime),
            self._return_runtime_path(yellow_runtime),
        )
        self._set_runtime_config(config)

    def fetch_runtime_config(self) -> RuntimeConfig:
        """Fetches the runtime configuration from the local disk. If the blue/yellow configuration is invalid,
        returns the default runtime configuration for blue/yellow
        :return: Returns the runtime configuration as a RuntimeConfig
        """
        # Create default FullSystem pair with our FullSystem binaries
        config = RuntimeConfig()

        try:
            with open(RuntimeManagerConstants.RUNTIME_CONFIG_PATH, "rb") as file:
                selected_runtime_dict = tomllib.load(file)
                # If a different blue FullSystem is persisted, replace the default arrangement
                toml_blue_path = selected_runtime_dict.get(
                    RuntimeManagerConstants.RUNTIME_CONFIG_BLUE_KEY,
                    RuntimeManagerConstants.DEFAULT_BINARY_PATH,
                )
                if self._is_valid_runtime(toml_blue_path):
                    config.chosen_blue_path = toml_blue_path
                # If a different yellow FullSystem is persisted, replace the default arrangement
                toml_yellow_path = selected_runtime_dict.get(
                    RuntimeManagerConstants.RUNTIME_CONFIG_YELLOW_KEY,
                    RuntimeManagerConstants.DEFAULT_BINARY_PATH,
                )
                if self._is_valid_runtime(toml_yellow_path):
                    config.chosen_yellow_path = toml_yellow_path
        except (FileNotFoundError, PermissionError, TOMLDecodeError):
            logging.warning(
                f"Failed to read TOML file at: {RuntimeManagerConstants.RUNTIME_CONFIG_PATH}"
            )

        return config

    def _set_runtime_config(self, config: RuntimeConfig) -> None:
        """Sets/persists the runtime configuration file on disk and creates the configuration
        file if it doesn't exist.
        :param config: The runtime configuration containing
         - color_runtime : absolute path of external runtime, or
         - color_runtime : relative path of DEFAULT_BINARY_PATH
        """
        blue_path = config.chosen_blue_path
        yellow_path = config.chosen_yellow_path

        """Format in TOML as:
        blue_path_to_binary: '<runtime path>'
        yellow_path_to_binary: '<runtime path>'"""

        selected_runtimes = (
            f'{RuntimeManagerConstants.RUNTIME_CONFIG_BLUE_KEY} = "{blue_path}"\n'
            f'{RuntimeManagerConstants.RUNTIME_CONFIG_YELLOW_KEY} = "{yellow_path}"'
        )

        # create a new config file if it doesn't exist, and write in the format above to it
        with open(RuntimeManagerConstants.RUNTIME_CONFIG_PATH, "w") as file:
            file.write(selected_runtimes)

    def _return_runtime_path(self, selected_runtime: str) -> str:
        """Returns the absolute path of a binary given its name, or the path of our default FullSystem
        if the binary is not valid.
        :param selected_runtime: the name of the selected runtime binary
        :return: the absolute path of the binary as a string, or the relative path of our FullSystem
        """
        file_path = os.path.join(
            RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH, selected_runtime
        )
        # Default to our full system if it is selected or the selected binary isn't a valid runtime
        if (
            selected_runtime == RuntimeManagerConstants.DEFAULT_BINARY_NAME
            or not self._is_valid_runtime(file_path)
        ):
            return RuntimeManagerConstants.DEFAULT_BINARY_PATH
        # Remove leading and trailing white space and return
        return file_path.strip()

    def _is_valid_runtime(self, runtime_path: str) -> bool:
        """Returns if the path exists and if it is an executable. Logs a warning if it is not valid.
        :param runtime_path the path to check
        :return: whether it is a valid runtime or not
        """
        if os.path.isfile(runtime_path) and os.access(runtime_path, os.X_OK):
            return True
        logging.warning(
            f"The runtime retrieved at {runtime_path} is not a valid runtime."
        )
        return False
