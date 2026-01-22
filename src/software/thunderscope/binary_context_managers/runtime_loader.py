from software.thunderscope.constants import RuntimeManagerConstants
import os
import tomllib
import logging


class AIConfig:
    def __init__(
        self,
        chosen_blue_name: str = RuntimeManagerConstants.DEFAULT_BINARY_PATH,
        chosen_yellow_name: str = RuntimeManagerConstants.DEFAULT_BINARY_PATH,
    ) -> None:
        """Data class to store the names of the binaries
        :param chosen_blue_name the name for the blue FullSystem
        :param chosen_yellow_name the name for the yellow FullSystem
        """
        self.chosen_blue_name = chosen_blue_name
        self.chosen_yellow_name = chosen_yellow_name


class RuntimeLoader:
    """Delegate class for handling local runtimes and managing runtime selection"""

    def __init__(self):
        self.cached_runtimes = None

    def fetch_installed_runtimes(self) -> list[str]:
        """Fetches the list of available runtimes, including our FullSystem, from the local disk
        :return: A list of names for available runtimes, or just a list with our FullSystem if no available runtimes
        could be found
        """
        if self.cached_runtimes:
            return self.cached_runtimes.keys()
        list_of_ai = []
        try:
            # Check for all executable files in the folder
            for file_name in os.listdir(RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH):
                file_path = os.path.join(
                    RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH, file_name
                )
                if os.path.isfile(file_path) and os.access(file_path, os.X_OK):
                    list_of_ai.append(os.path.basename(file_name))

        except (FileNotFoundError, PermissionError, NotADirectoryError):
            logging.info("Folder for external AI could not be accessed.")

        finally:
            # Add an option for our FullSystem
            list_of_ai.append(RuntimeManagerConstants.DEFAULT_BINARY_NAME)
            return list_of_ai

    def load_existing_runtimes(self, yellow_runtime: str, blue_runtime: str) -> None:
        """Loads the yellow and blue runtimes specified by saving them in the local disk.
        :param blue_runtime: Unique name of the blue runtime to set
        :param yellow_runtime: Unique name of the yellow runtime to set
        """
        config = AIConfig(blue_runtime, yellow_runtime)
        self._set_runtime_config(config)
        pass

    def fetch_runtime_config(self) -> AIConfig:
        """Fetches the runtime configuration from the local disk, and caches it in this.
        :return: Returns the runtime configuration as a AIConfig
        """
        # Create default FullSystem pair with our FullSystem binaries
        config = AIConfig()

        if os.path.isfile(RuntimeManagerConstants.RUNTIME_CONFIG_PATH):
            with open(RuntimeManagerConstants.RUNTIME_CONFIG_PATH, "rb") as file:
                selected_ai_dict = tomllib.load(file)
                # If a different blue FullSystem is persisted, replace the default arrangement
                if (
                    RuntimeManagerConstants.RUNTIME_CONFIG_BLUE_KEY
                    in selected_ai_dict.keys()
                ):
                    config.chosen_blue_name = selected_ai_dict.get(
                        RuntimeManagerConstants.RUNTIME_CONFIG_BLUE_KEY
                    )
                # If a different yellow FullSystem is persisted, replace the default arrangement
                if (
                    RuntimeManagerConstants.RUNTIME_CONFIG_YELLOW_KEY
                    in selected_ai_dict.keys()
                ):
                    config.chosen_blue_name = selected_ai_dict.get(
                        RuntimeManagerConstants.RUNTIME_CONFIG_YELLOW_KEY
                    )

        # Display logging message when using default FullSystem
        if config.chosen_blue_name == RuntimeManagerConstants.RUNTIME_CONFIG_PATH:
            logging.info("TBots FullSystem selected for blue side.")
        if config.chosen_yellow_name == RuntimeManagerConstants.RUNTIME_CONFIG_PATH:
            logging.info("TBots FullSystem selected for yellow side.")

        # Cache runtimes
        self.cached_runtimes = config

        return config

    def _create_runtime_config(self) -> None:
        """Creates the runtime configuration file on disk and throws an error upon failure."""
        # TODO Not sure if this function really has an use, since python open() creates a new file by default if it
        #  doesn't exist
        pass

    def _set_runtime_config(self, config: AIConfig) -> None:
        """Sets/persists the runtime configuration file on disk and creates the configuration
        file if it doesn't exist.
        :param config: The runtime configuration containing
         - color_runtime : absolute path of external runtime, or
         - color_runtime : relative path of DEFAULT_BINARY_PATH
        """
        blue_path = self._return_ai_path(config.chosen_blue_name)
        yellow_path = self._return_ai_path(config.chosen_yellow_name)

        """Format in TOML as:
        blue_path_to_binary: '<absolute path>'
        yellow_path_to_binary: '<absolute path>'"""

        selected_ais = (
            f'{RuntimeManagerConstants.RUNTIME_CONFIG_BLUE_KEY} = "{blue_path}"\n'
            f'{RuntimeManagerConstants.RUNTIME_CONFIG_YELLOW_KEY} = "{yellow_path}"\n'
        )

        # create a new config file if it doesn't exist, and write in the format above to it
        try:
            with open(RuntimeManagerConstants.RUNTIME_CONFIG_PATH, "w") as file:
                file.write(selected_ais)
        except (PermissionError, NotADirectoryError):
            logging.error("Could not access the configuration file.")
        pass

    def _return_ai_path(self, selected_ai: str) -> str:
        """Returns the absolute path of a binary given its name, or the path of our default FullSystem
        if the binary is not valid.
        :param selected_ai: the name of the selected AI binary

        :return: the absolute path of the binary as a string
        """
        file_path = os.path.join(
            RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH, selected_ai
        )
        # Default to our full system if it is selected or the selected binary doesn't exist
        if (
            selected_ai == RuntimeManagerConstants.DEFAULT_BINARY_NAME
            or not os.path.isfile(file_path)
        ):
            return RuntimeManagerConstants.DEFAULT_BINARY_PATH
        else:
            # Remove leading and trailing white space and return
            return file_path.strip()
