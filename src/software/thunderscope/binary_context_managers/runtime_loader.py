from tomllib import TOMLDecodeError

from software.thunderscope.constants import RuntimeManagerConstants
import os
import tomllib
import logging


class RuntimeConfig:
    """Data class to store the paths of the two binaries"""

    def __init__(
        self,
        chosen_blue_path: str = RuntimeManagerConstants.DEFAULT_BINARY_PATH,
        chosen_yellow_path: str = RuntimeManagerConstants.DEFAULT_BINARY_PATH,
    ) -> None:
        """Stores the paths of the binaries in this
        :param chosen_blue_path the name for the blue FullSystem
        :param chosen_yellow_path the name for the yellow FullSystem
        """
        self.chosen_blue_path = chosen_blue_path
        self.chosen_yellow_path = chosen_yellow_path


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
                if os.path.isfile(file_path) and os.access(file_path, os.X_OK):
                    runtime_dict[os.path.basename(file_name)] = file_path

        except (FileNotFoundError, PermissionError, NotADirectoryError):
            logging.warning(
                f"Folder for external runtimes {RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH} could not be accessed."
            )

        finally:
            # Add an option for our FullSystem
            runtime_dict[RuntimeManagerConstants.DEFAULT_BINARY_NAME] = (
                RuntimeManagerConstants.DEFAULT_BINARY_PATH
            )

            # Cache external runtimes
            self.cached_runtimes = runtime_dict
            return list(runtime_dict.keys())

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
        """Fetches the runtime configuration from the local disk
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
                    config.chosen_blue_path = selected_runtime_dict[
                        RuntimeManagerConstants.RUNTIME_CONFIG_BLUE_KEY
                    ]
                # If a different yellow FullSystem is persisted, replace the default arrangement
                if (
                    RuntimeManagerConstants.RUNTIME_CONFIG_YELLOW_KEY
                    in selected_runtime_dict.keys()
                ):
                    config.chosen_yellow_path = selected_runtime_dict[
                        RuntimeManagerConstants.RUNTIME_CONFIG_YELLOW_KEY
                    ]
        except (FileNotFoundError, PermissionError, TOMLDecodeError):
            logging.warning(
                f"Failed to fetch runtime configuration from {RuntimeManagerConstants.RUNTIME_CONFIG_PATH}"
            )

        # Display logging message when using default FullSystem
        if config.chosen_blue_path == RuntimeManagerConstants.DEFAULT_BINARY_PATH:
            logging.info("TBots FullSystem selected for blue side.")
        if config.chosen_yellow_path == RuntimeManagerConstants.DEFAULT_BINARY_PATH:
            logging.info("TBots FullSystem selected for yellow side.")

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
        # Default to our full system if it is selected or the selected binary isn't an existing executable
        if selected_runtime == RuntimeManagerConstants.DEFAULT_BINARY_NAME or not (
            os.path.isfile(file_path) and os.access(file_path, os.X_OK)
        ):
            return RuntimeManagerConstants.DEFAULT_BINARY_PATH
        else:
            # Remove leading and trailing white space and return
            return file_path.strip()
