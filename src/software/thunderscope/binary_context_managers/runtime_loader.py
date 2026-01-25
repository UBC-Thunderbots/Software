from tomllib import TOMLDecodeError
from software.thunderscope.constants import RuntimeManagerConstants
import os
import tomllib
import logging


class RuntimeConfig:
    """Class to store the names and get paths of the two binaries"""

    def __init__(self) -> None:
        self.blue_runtime = RuntimeManagerConstants.DEFAULT_BINARY_NAME
        self.yellow_runtime = RuntimeManagerConstants.DEFAULT_BINARY_NAME

    def get_blue_runtime_path(self) -> str:
        """Returns the path of the stored yellow runtime
        :return: the absolute path of the binary as a string, or the relative path of our FullSystem
        """
        return self._get_runtime_path(self.blue_runtime)

    def get_yellow_runtime_path(self) -> str:
        """Returns the path of the stored yellow runtime
        :return: the absolute path of the binary as a string, or the relative path of our FullSystem
        """
        return self._get_runtime_path(self.yellow_runtime)

    def _get_runtime_path(self, selected_runtime: str) -> str:
        """Gets the absolute path of a binary given its name, or the path of our default FullSystem
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
        # Format as TOML
        selected_runtimes = (
            f'{RuntimeManagerConstants.RUNTIME_CONFIG_BLUE_KEY} = "{blue_runtime}"\n'
            f'{RuntimeManagerConstants.RUNTIME_CONFIG_YELLOW_KEY} = "{yellow_runtime}"'
        )

        # create a new config file if it doesn't exist, and write in the format above to it
        with open(RuntimeManagerConstants.RUNTIME_CONFIG_PATH, "w") as file:
            file.write(selected_runtimes)

    def fetch_runtime_config(self) -> RuntimeConfig:
        """Fetches the runtime configuration from the local disk, creating it if it doesn't exist.
        :return: Returns the runtime configuration as a RuntimeConfig
        """
        config = RuntimeConfig()

        # Create empty config file if doesn't exist yet
        os.makedirs(
            os.path.dirname(RuntimeManagerConstants.RUNTIME_CONFIG_PATH), exist_ok=True
        )
        open(RuntimeManagerConstants.RUNTIME_CONFIG_PATH, "a").close()

        try:
            with open(RuntimeManagerConstants.RUNTIME_CONFIG_PATH, "rb") as file:
                selected_runtime_dict = tomllib.load(file)

                # Get the persisted runtimes, or replace with the default runtime if it doesn't exist
                config.blue_runtime = selected_runtime_dict.get(
                    RuntimeManagerConstants.RUNTIME_CONFIG_BLUE_KEY,
                    RuntimeManagerConstants.DEFAULT_BINARY_NAME,
                )
                config.yellow_runtime = selected_runtime_dict.get(
                    RuntimeManagerConstants.RUNTIME_CONFIG_YELLOW_KEY,
                    RuntimeManagerConstants.DEFAULT_BINARY_NAME,
                )
        except TOMLDecodeError:
            logging.warning(
                f"Failed to read TOML file at: {RuntimeManagerConstants.RUNTIME_CONFIG_PATH}"
            )

        return config
