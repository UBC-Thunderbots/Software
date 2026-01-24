# TODO: #3559
import requests
import subprocess
from pathlib import Path
from software.logger.logger import create_logger
import zipfile
import tarfile
import shutil

logger = create_logger(__name__)
class RuntimeInstaller:
    """Delegate class for handling runtime installation and remote interfacing"""

    def __init__(self):
        download_urls = []

        pass

    def fetch_remote_runtimes(self) -> list[str]:
        """Requests a list of available runtimes from the remote. This is an expensive operation
        and should only be called when necessary.
        :return: A unique list of names for available runtimes
        """
        url = "https://api.github.com/repos/UBC-Thunderbots/Software/releases"
        headers = {"Accept": "application/vnd.github+json"}

        response = requests.get(url, headers=headers)
        logger.warning(response)

        releases = response.json()

        binaries = []

        for release in releases:
            for asset in release.get("assets", []):
                binaries.append(
                    {
                        "name": asset["name"],
                        "download_url": asset["browser_download_url"],
                        "size_bytes": asset["size"],
                        "created_at": asset["created_at"],
                        "release_tag": release["tag_name"],
                    }
                )
                if len(binaries) == 5:
                    break
            if len(binaries) == 5:
                break

        PREFIX = "https://github.com/UBC-Thunderbots/Software/releases/download/"

        #I'm going to assume you are trying to reload the assets so reset the download_urls
        if len(download_urls) != 0:
            download_urls = []
        download_names = []
        for release in releases:
            for asset in release.get("assets", []):
                url = asset["browser_download_url"]
                download_urls.append(url)
                trimmed = url.removeprefix(PREFIX)
                download_names.append(trimmed)
        return download_names

    def install_runtime(self, version: str) -> None:
        """Installs the runtime of the specified version or throws an error upon failure.
        Ensures that the runtime is compatible with the current platform
        :param version: Version of the runtime hosted on the remote to install
        """
        url = "https://api.github.com/repos/UBC-Thunderbots/Software/releases"
        headers = {"Accept": "application/vnd.github+json"}

        response = requests.get(url, headers=headers)
        logger.warning(response)

        releases = response.json()

        TARGET_SUFFIX = version

        selected_asset = None

        for i in range(len(download_urls)):
            if download_urls[i].endswith(TARGET_SUFFIX):
                selected_asset = download_urls[i]

        if selected_asset:
            url = selected_asset
            filename = Path(url).name

            target_dir = Path("/opt/tbotspython/external_runtimes")
            tmp_path = Path("/tmp") / filename

            target_dir.mkdir(parents=True, exist_ok=True)

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

        if not selected_asset:
            logger.warning("Can't find binary")

        pass
