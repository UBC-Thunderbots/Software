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

    def __init__(self):
        # Dictionary of runtime name to runtime path. When fetching installed runtimes,
        # scans the disk if the cache is empty or None.
        self.cached_runtimes = None

    def fetch_installed_runtimes(self) -> list[str]:
        """Fetches the list of available runtimes, including our FullSystem, from the local disk. Caches a dictionary
        of the runtime name mapped to runtime path.
        :return: A list of names for available runtimes, or just a list with our FullSystem if no available runtimes
        could be found
        """
        if self.cached_runtimes:
            return list(self.cached_runtimes.keys())
        runtime_dict = {}
        try:
            # Check for all executable files in the folder
            for file_name in os.listdir(RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH):
                file_path = os.path.join(
                    RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH, file_name
                )
                if self._is_valid_runtime(file_path):
                    runtime_dict[os.path.basename(file_name)] = file_path

        except (FileNotFoundError, PermissionError, NotADirectoryError):
            logging.warning(f"Folder for external runtimes {RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH} could not be accessed.")

        finally:
            # Add an option for our FullSystem
            runtime_dict[RuntimeManagerConstants.DEFAULT_BINARY_NAME] = RuntimeManagerConstants.DEFAULT_BINARY_PATH

            # Cache external runtimes
            self.cached_runtimes = runtime_dict
            return list(runtime_dict.keys())

    def load_existing_runtimes(self, yellow_runtime: str, blue_runtime: str) -> None:
        """Loads the yellow and blue runtimes specified by saving them in the local disk.
        :param blue_runtime: Unique name of the blue runtime to set
        :param yellow_runtime: Unique name of the yellow runtime to set
        """
        config = RuntimeConfig(self._return_runtime_path(blue_runtime), self._return_runtime_path(yellow_runtime))
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
                if (
                        RuntimeManagerConstants.RUNTIME_CONFIG_BLUE_KEY
                        in selected_runtime_dict.keys()
                ):
                    toml_blue_path = selected_runtime_dict[RuntimeManagerConstants.RUNTIME_CONFIG_BLUE_KEY]
                    if self._is_valid_runtime(toml_blue_path):
                        config.chosen_blue_path = toml_blue_path
                else:
                    logging.warning(f"Failed to fetch runtime configuration blue field from {RuntimeManagerConstants.RUNTIME_CONFIG_PATH}")
                # If a different yellow FullSystem is persisted, replace the default arrangement
                if (
                        RuntimeManagerConstants.RUNTIME_CONFIG_YELLOW_KEY
                        in selected_runtime_dict.keys()
                ):
                    toml_yellow_path = selected_runtime_dict[RuntimeManagerConstants.RUNTIME_CONFIG_YELLOW_KEY]
                    if self._is_valid_runtime(toml_yellow_path):
                        config.chosen_yellow_path = toml_yellow_path
                else:
                    logging.warning(f"Failed to fetch runtime configuration yellow field from {RuntimeManagerConstants.RUNTIME_CONFIG_PATH}")
        except (FileNotFoundError, PermissionError, TOMLDecodeError):
            logging.warning(f"Failed to read TOML file at: {RuntimeManagerConstants.RUNTIME_CONFIG_PATH}")

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
            f'{RuntimeManagerConstants.RUNTIME_CONFIG_YELLOW_KEY} = "{yellow_path}"\n'
        )

        # create a new config file if it doesn't exist, and write in the format above to it
        try:
            with open(RuntimeManagerConstants.RUNTIME_CONFIG_PATH, "w") as file:
                file.write(selected_runtimes)
        except (PermissionError, NotADirectoryError):
            logging.error("Could not access the configuration file.")

    def _return_runtime_path(self, selected_runtime: str) -> str:
        """Returns the absolute path of a binary given its name, or the path of our default FullSystem
        if the binary is not valid.
        :param selected_runtime: the name of the selected runtime binary

        :return: the absolute path of the binary as a string
        """
        # Check cache
        if self.cached_runtimes and selected_runtime in self.cached_runtimes:
            return self.cached_runtimes[selected_runtime]
        file_path = os.path.join(
            RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH, selected_runtime
        )
        # Default to our full system if it is selected or the selected binary isn't a valid runtime
        if (
                selected_runtime == RuntimeManagerConstants.DEFAULT_BINARY_NAME
                or not self._is_valid_runtime(file_path)
        ):
            return RuntimeManagerConstants.DEFAULT_BINARY_PATH
        else:
            # Remove leading and trailing white space and return
            return file_path.strip()

    def _is_valid_runtime(self, runtime_path: str) -> bool:
        """Returns if the path exists and if it is an executable. Logs a warning if it is not valid.
        :param runtime_path the path to check
        :return: whether it is a valid runtime or not
        """
        if os.path.isfile(runtime_path) and os.access(runtime_path, os.X_OK):
            return True
        logging.warning(f"The runtime retrieved at {runtime_path} is not a valid runtime.")
        return False
