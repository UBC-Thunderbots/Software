from software.thunderscope.constants import RuntimeManagerConstants
import os
from pathlib import Path
import tomllib
import logging
from software.thunderscope.constants import AISelectorConstants

# TODO: #3557
class AIConfig:

    def __init__(
            self,
            chosen_blue_name: str,
            chosen_yellow_name: str,
    ) -> None:
        """Data class to store the names of the binaries
        :param chosen_blue_name the name for the blue fullsystem
        :param chosen_yellow_name the name for the yellow fullsystem
        """
        self.chosen_blue_name = chosen_blue_name
        self.chosen_yellow_name = chosen_yellow_name

class RuntimeLoader:
    """Delegate class for handling local runtimes and managing runtime selection"""

    def __init__(self):
        self.cached_runtimes = None

    def fetch_installed_runtimes(self) -> list[str]:
        """Fetches the list of available runtimes, including our fullsystem, from the local disk
        :return: A list of names for available runtimes, or just a list with our fullsystem if no available runtimes
        could be found
        """
        if self.cached_runtimes:
            return self.cached_runtimes.keys()
        list_of_ai = []
        try:
            # Check for all executable files in the folder
            for file_name in os.listdir(AISelectorConstants.AI_FOLDER):
                file_path = os.path.join(AISelectorConstants.AI_FOLDER, file_name)
                if os.path.isfile(file_path) and os.access(file_path, os.X_OK):
                    list_of_ai.append(os.path.basename(file_name))

        except (FileNotFoundError, PermissionError, NotADirectoryError):
            logging.info("No external AI")

        finally:
            # Add an option for our fullsystem
            list_of_ai.append(AISelectorConstants.FULLSYSTEM_DISPLAY_NAME)
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
        """Fetches the runtime configuration from the local disk.
        :return: Returns the runtime configuration as a AIConfig
        """
        # Create default fullsystem pair with our fullsystem binaries
        fullsystem_pair = AIConfig(AISelectorConstants.FULLSYSTEM_PATH, AISelectorConstants.FULLSYSTEM_PATH)

        if os.path.isfile(AISelectorConstants.CONFIG_FILE_PATH):
            with open(AISelectorConstants.CONFIG_FILE_PATH, "rb") as file:
                selected_ai_dict = tomllib.load(file)
                # If a different blue fullsystem is persisted, replace the default arrangement
                if AISelectorConstants.BLUE_TOML_FIELD in selected_ai_dict.keys():
                    fullsystem_pair.chosen_blue_name = selected_ai_dict.get(AISelectorConstants.BLUE_TOML_FIELD)
                # If a different yellow fullsystem is persisted, replace the default arrangemnet
                if AISelectorConstants.YELLOW_TOML_FIELD in selected_ai_dict.keys():
                    fullsystem_pair.chosen_blue_name = selected_ai_dict.get(AISelectorConstants.YELLOW_TOML_FIELD)
                file.close()

        # Display logging message when using default fullsystem
        if fullsystem_pair.chosen_blue_name == AISelectorConstants.BLUE_TOML_FIELD:
            logging.info("TBots Fullsystem selected for blue side.")
        if fullsystem_pair.chosen_yellow_name == AISelectorConstants.YELLOW_TOML_FIELD:
            logging.info("TBots Fullsystem selected for yellow side.")

        return fullsystem_pair

    def _create_runtime_config(self) -> None:
        """Creates the runtime configuration file on disk and throws an error upon failure."""
        # TODO ask about creating config file
        pass

    def _set_runtime_config(self, fullsystem_pair: AIConfig) -> None:
        """Sets/persists the runtime configuration file on disk and creates the configuration
        file if it doesn't exist.
        :param config: The runtime configuration containing
         - color_runtime : absolute path of external runtime, or
         - color_runtime : relative path of DEFAULT_BINARY_PATH
        """

        blue_path = self._return_ai_path(fullsystem_pair.chosen_blue_name)
        yellow_path = self._return_ai_path(fullsystem_pair.chosen_yellow_name)

        """Format in TOML as:
        blue_path_to_binary: '<absolute path>'
        yellow_path_to_binary: '<absolute path>'"""

        selected_ais = (
            f'{AISelectorConstants.BLUE_TOML_FIELD} = "{blue_path}"\n'
            f'{AISelectorConstants.YELLOW_TOML_FIELD} = "{yellow_path}"\n'
        )

        # create a new config file if it doesn't exist, and write in the format above to it

        with open(AISelectorConstants.CONFIG_FILE_PATH, "w") as file:
            Path(AISelectorConstants.CONFIG_FILE_PATH).write_text(selected_ais, encoding="utf-8")
            file.close()
        pass

    def _return_ai_path(self, selected_ai: str) -> str:
        """Returns the absolute path of a binary given its name, or the path of our default fullsystem if the binary is not
        valid.
        :param selected_ai: the name of the selected AI binary

        :return: the absolute path of the binary as a string
        """

        file_path = os.path.join(AISelectorConstants.AI_FOLDER, selected_ai)
        # Default to our full system if it is selected or the selected binary doesn't exist
        if selected_ai == AISelectorConstants.FULLSYSTEM_DISPLAY_NAME or not os.path.isfile(file_path):
            return AISelectorConstants.FULLSYSTEM_PATH
        else:
            # Remove leading and trailing white space and return
            return file_path.strip()
