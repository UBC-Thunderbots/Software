import requests
from pathlib import Path
import zipfile
import tarfile
import shutil
import logging
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
            RuntimeManagerConstants.INSTALL_URL,
            headers={"Accept": "application/vnd.github+json"},
        ).json()

        download_names = []

        for release in releases:
            for asset in release.get("assets", []):
                url = asset["browser_download_url"]
                if "unix_full_system" not in url:
                    continue

                version = url.removeprefix(RuntimeManagerConstants.DOWNLOAD_PREFIX)
                download_names.append(version)
                self.runtime_install_targets[version] = url

        return download_names[:5]

    def install_runtime(self, version: str) -> None:
        """Installs the runtime of the specified version or throws an error upon failure.
        Ensures that the runtime is compatible with the current platform
        :param version: Version of the runtime hosted on the remote to install
        """
        url = self.runtime_install_targets[version]

        filename = Path(url).name
        target_dir = Path(RuntimeManagerConstants.EXTERNAL_RUNTIMES_PATH)
        tmp_path = Path("/tmp") / filename

        with requests.get(url, stream=True) as r:
            r.raise_for_status()
            with open(tmp_path, "wb") as f:
                for chunk in r.iter_content(chunk_size=8192):
                    if chunk:
                        f.write(chunk)

        if filename.endswith((".tar.gz", ".tgz")):
            with tarfile.open(tmp_path, "r:*") as tar:
                tar.extractall(path=target_dir)
        elif filename.endswith(".zip"):
            with zipfile.ZipFile(tmp_path, "r") as zipf:
                zipf.extractall(path=target_dir)
        else:
            dest = target_dir / filename
            shutil.copy2(tmp_path, dest)
            dest.chmod(0o755)  # make executable (common for runtimes)
