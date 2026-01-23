# TODO: #3559
import requests
import subprocess
from pathlib import Path


class RuntimeInstaller:
    """Delegate class for handling runtime installation and remote interfacing"""

    def __init__(self):
        pass

    def fetch_remote_runtimes(self) -> list[str]:
        """Requests a list of available runtimes from the remote. This is an expensive operation
        and should only be called when necessary.
        :return: A unique list of names for available runtimes
        """
        url = "https://api.github.com/repos/UBC-Thunderbots/Software/releases"

        headers = {"Accept": "application/vnd.github+json"}

        response = requests.get(url, headers=headers)
        response.raise_for_status()

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

        download_urls = []

        for release in releases:
            for asset in release.get("assets", []):
                url = asset["browser_download_url"]
                trimmed = url.removeprefix(PREFIX)
                download_urls.append(trimmed)
        return download_urls

    def install_runtime(self, version: str) -> None:
        """Installs the runtime of the specified version or throws an error upon failure.
        Ensures that the runtime is compatible with the current platform
        :param version: Version of the runtime hosted on the remote to install
        """
        url = "https://api.github.com/repos/UBC-Thunderbots/Software/releases"
        headers = {"Accept": "application/vnd.github+json"}

        response = requests.get(url, headers=headers)
        response.raise_for_status()

        releases = response.json()

        TARGET_SUFFIX = version

        selected_asset = None

        for release in releases:
            for asset in release.get("assets", []):
                if asset["browser_download_url"].endswith(TARGET_SUFFIX):
                    selected_asset = {
                        "name": asset["name"],
                        "download_url": asset["browser_download_url"],
                        "size_bytes": asset["size"],
                        "created_at": asset["created_at"],
                        "release_tag": release["tag_name"],
                    }
                    break
            if selected_asset:
                url = selected_asset["download_url"]
                filename = Path(url).name
                target_dir = Path("/opt/tbotspython/external_runtimes")

                # Ensure target directory exists
                subprocess.run(["sudo", "mkdir", "-p", str(target_dir)], check=True)

                tmp_path = Path("/tmp") / filename
                subprocess.run(["wget", "-O", str(tmp_path), url], check=True)

                if filename.endswith((".tar.gz", ".tgz")):
                    subprocess.run(
                        ["sudo", "tar", "-xvf", str(tmp_path), "-C", str(target_dir)],
                        check=True,
                    )

                elif filename.endswith(".zip"):
                    subprocess.run(
                        ["sudo", "unzip", str(tmp_path), "-d", str(target_dir)],
                        check=True,
                    )

                else:
                    subprocess.run(
                        ["cp", str(archive_path), str(INSTALL_DIR / filename)],
                        check=True,
                    )

            if not selected_asset:
                raise RuntimeError(f"No asset found with name: {version}")

            break

        pass
