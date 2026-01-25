from tomllib import TOMLDecodeError
from software.thunderscope.constants import RuntimeManagerConstants
from dataclasses import dataclass
import os
import tomllib
import logging


@dataclass
class RuntimeConfig:
    """Data class to store the paths of the two binaries"""

    blue_runtime_path: str = RuntimeManagerConstants.DEFAULT_BINARY_PATH
    yellow_runtime_path: str = RuntimeManagerConstants.DEFAULT_BINARY_PATH


class RuntimeLoader:
    """Delegate class for handling local runtimes and managing runtime selection"""

    def fetch_installed_runtimes(self) -> list[str]:
        """Fetches the list of installed runtimes from the local disk.
        Creates the external runtimes directory in our local disk if it does not exist yet.
        :return: A list of installed runtime names
        """
        runtime_list = [RuntimeManagerConstants.DEFAULT_BINARY_NAME]

        if not os.path.isdir(RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH):
            os.mkdir(RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH)

        # Check for all executable files in the directory, and add its name to the list
        for file_name in os.listdir(RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH):
            file_path = os.path.join(
                RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH, file_name
            )
            if os.access(file_path, os.X_OK):
                runtime_list.append(file_name)

        return runtime_list

    def load_selected_runtimes(self, yellow_runtime: str, blue_runtime: str) -> None:
        """Loads the yellow and blue runtimes specified by saving them in the local disk.
        :param blue_runtime: name of the blue runtime to set
        :param yellow_runtime: name of the yellow runtime to set
        """
        config = RuntimeConfig(
            self._return_runtime_path(blue_runtime),
            self._return_runtime_path(yellow_runtime),
        )
        self._set_runtime_config(config)

    def fetch_runtime_config(self) -> RuntimeConfig:
        """Fetches the runtime configuration from the local disk.
        If the saved blue/yellow runtime is invalid, returns the default runtimes for blue/yellow
        :return: Returns the runtime configuration as a RuntimeConfig
        """
        config = RuntimeConfig()

        try:
            with open(RuntimeManagerConstants.RUNTIME_CONFIG_PATH, "rb") as file:
                selected_runtime_dict = tomllib.load(file)

                # Get the persisted blue path, or replace with the default path if it doesn't exist
                toml_blue_path = selected_runtime_dict.get(
                    RuntimeManagerConstants.RUNTIME_CONFIG_BLUE_KEY,
                    RuntimeManagerConstants.DEFAULT_BINARY_PATH,
                )
                if self._is_valid_runtime(toml_blue_path):
                    config.blue_runtime_path = toml_blue_path

                # Get the persisted yellow path, or replace with the default path if it doesn't exist
                toml_yellow_path = selected_runtime_dict.get(
                    RuntimeManagerConstants.RUNTIME_CONFIG_YELLOW_KEY,
                    RuntimeManagerConstants.DEFAULT_BINARY_PATH,
                )
                if self._is_valid_runtime(toml_yellow_path):
                    config.yellow_runtime_path = toml_yellow_path
        except (FileNotFoundError, PermissionError, TOMLDecodeError):
            logging.warning(
                f"Failed to read TOML file at: {RuntimeManagerConstants.RUNTIME_CONFIG_PATH}"
            )

        return config

    def _set_runtime_config(self, config: RuntimeConfig) -> None:
        """Sets the runtime configuration file on disk
        :param config: The runtime configuration being saved
        """
        blue_path = config.blue_runtime_path
        yellow_path = config.yellow_runtime_path

        """
        Format in TOML as:
        blue_path_to_binary: "<runtime path>"
        yellow_path_to_binary: "<runtime path>"
        """

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
        # Default to local FullSystem if it is selected or the selected binary isn't a valid runtime
        if (
            selected_runtime == RuntimeManagerConstants.DEFAULT_BINARY_NAME
            or not self._is_valid_runtime(file_path)
        ):
            return RuntimeManagerConstants.DEFAULT_BINARY_PATH
        # Remove leading and trailing white space and return
        return file_path.strip()

    def _is_valid_runtime(self, runtime_path: str) -> bool:
        """Returns if the binary exists and if it is an executable.
        :param runtime_path: the path to check
        :return: True if it is a valid runtime
        """
        return os.path.isfile(runtime_path) and os.access(runtime_path, os.X_OK)
