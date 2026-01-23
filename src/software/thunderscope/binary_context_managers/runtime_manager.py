from software.thunderscope.binary_context_managers.runtime_installer import (
    RuntimeInstaller,
)
from software.thunderscope.binary_context_managers.runtime_loader import (
    RuntimeLoader,
    RuntimeConfig,
)


class RuntimeManager:
    """Class for interfacing with AI runtimes/backends (full system or external) on the disk"""

    def __init__(self):
        self.runtime_installer = RuntimeInstaller()
        self.runtime_loader = RuntimeLoader()

    def fetch_remote_runtimes(self) -> list[str]:
        """Requests a list of available runtimes from the remote. Includes DEFAULT_BINARY_NAME by default
        :return: A unique list of names for available runtimes
        """
        return self.runtime_installer.fetch_remote_runtimes()

    def install_runtime(self, version: str) -> None:
        """Installs the runtime of the specified version or throws an error upon failure.
        :param version: Version of the runtime hosted on the remote to install
        """
        self.runtime_installer.install_runtime(version)

    def fetch_installed_runtimes(self) -> list[str]:
        """Fetches the list of available runtimes from the local disk
        :return: A list of names for available runtimes
        """
        return self.runtime_loader.fetch_installed_runtimes()

    def load_existing_runtimes(self, yellow_runtime: str, blue_runtime: str) -> None:
        """Loads the runtimes of the specified name or throws an error upon failure.
        :param blue_runtime: name of the blue runtime to load
        :param yellow_runtime: name of the yellow runtime to load
        """
        self.runtime_loader.load_existing_runtimes(yellow_runtime, blue_runtime)

    def fetch_runtime_config(self) -> RuntimeConfig:
        """Fetches the runtime configuration from the local disk
        :return: Returns the runtime configuration as a RuntimeConfig
        """
        return self.runtime_loader.fetch_runtime_config()


runtime_manager_instance = RuntimeManager()
