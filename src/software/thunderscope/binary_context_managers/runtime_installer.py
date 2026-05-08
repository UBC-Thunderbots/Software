import requests
from pathlib import Path
import tarfile
import shutil
import platform
from software.thunderscope.constants import RuntimeManagerConstants


class RuntimeInstaller:
    """Delegate class for handling runtime installation and remote interfacing"""

    def __init__(self):
        self.runtime_install_targets = {}

    def fetch_remote_runtimes(self) -> list[str]:
        """Requests a list of available runtimes from the remote. This is an expensive operation
        and should only be called when necessary.
        :return: A unique list of names for available runtimes
        """
        releases = requests.get(
            RuntimeManagerConstants.RELEASES_URL,
            headers={"Accept": "application/vnd.github+json"},
        ).json()

        version_names = []

        # Currently the only targets that are supported for each os
        os_to_target = {"Darwin": "mac-arm64", "Linux": "ubuntu-x86"}
        target = os_to_target[platform.system()]

        for release in releases:
            version = release["tag_name"]
            for asset in release.get("assets", []):
                url = asset["browser_download_url"]

                if "unix_full_system" in url and target in url:
                    version_names.append(version)
                    self.runtime_install_targets[version] = url

        return version_names[: RuntimeManagerConstants.MAX_RELEASES_FETCHED]

    def install_runtime(self, version: str) -> None:
        """Installs the runtime of the specified version or throws an error upon failure.
        Ensures that the runtime is compatible with the current platform
        :param version: Version of the runtime hosted on the remote to install
        """
        url = self.runtime_install_targets[version]

        filename = Path(url).name
        target_dir = Path(RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH)
        tmp_dir = Path("/tmp")
        tmp_path = tmp_dir / filename
        extracted_binary_name = "unix_full_system"

        with requests.get(url, stream=True) as r:
            r.raise_for_status()
            with open(tmp_path, "wb") as f:
                for chunk in r.iter_content(chunk_size=8192):
                    if chunk:
                        f.write(chunk)

        dest = target_dir / f"{extracted_binary_name}_{version}"

        # Our release assets for FullSystem are always tar.gz files
        with tarfile.open(tmp_path, "r:*") as tar:
            tar.extractall(tmp_dir)
            shutil.move(tmp_dir / extracted_binary_name, dest)
