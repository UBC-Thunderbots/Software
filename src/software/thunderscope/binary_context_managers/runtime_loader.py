from software.thunderscope.constants import RuntimeManagerConstants


class RunTimeLoader:
    """Delegate class for handling local runtimes and managing runtime selection"""

    def __init__(self):
        self.cached_runtimes = None

    def fetch_installed_runtimes(self) -> list[str]:
        """Fetches the list of available runtimes from the local disk
        :return: A list of names for available runtimes
        """
        if self.cached_runtimes:
            return self.cached_runtimes.keys()
        else:
            return []

    def load_existing_runtimes(self, yellow_runtime: str, blue_runtime: str) -> None:
        """Loads the yellow and blue runtimes specified by saving them in the local disk.
        :param blue_runtime: Unique name of the blue runtime to set
        :param yellow_runtime: Unique name of the yellow runtime to set
        """
        config = {}
        self._set_runtime_config(config)
        pass

    def fetch_runtime_config(self) -> dict[str, str]:
        """Fetches the runtime configuration from the local disk.
        :return: Returns the runtime configuration as a map
        """
        return {
            RuntimeManagerConstants.RUNTIME_CONFIG_BLUE_KEY: RuntimeManagerConstants.DEFAULT_BINARY_PATH,
            RuntimeManagerConstants.RUNTIME_CONFIG_YELLOW_KEY: RuntimeManagerConstants.DEFAULT_BINARY_PATH,
        }

    def _create_runtime_config(self) -> None:
        """Creates the runtime configuration file on disk and throws an error upon failure."""
        pass

    def _set_runtime_config(self, config: dict[str, str]) -> None:
        """Sets/persists the runtime configuration file on disk and creates the configuration
        file if it doesn't exist.
        :param config: The runtime configuration containing
         - color_runtime : absolute path of external runtime, or
         - color_runtime : relative path of DEFAULT_BINARY_PATH
        """
        pass
