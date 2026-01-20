class RuntimeInstaller:
    def __init__(self):
        pass

    def fetch_remote_runtimes(self) -> list[str]:
        """Requests a list of available runtimes from the remote. This is an expensive operation
        and should only be called when necessary.
        :return: A unique list of names for available runtimes
        """
        return []

    def install_runtime(self, version: str) -> None:
        """Installs the runtime of the specified version or throws an error upon failure.
        Ensures that the runtime is compatible with the current platform
        :param version: Version of the runtime hosted on the remote to install
        """
        pass
