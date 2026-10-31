import platform
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import requests
from software.thunderscope.binary_context_managers.runtime_constants import (
    LOCAL_RUNTIME_ID,
    MAX_RELEASES_FETCHED,
    RELEASES_URL,
    RUNTIME_ROOT,
)
from software.thunderscope.binary_context_managers.runtime_manifest import (
    ManifestError,
    RuntimeManifest,
)

_RELEASE_TARGETS = {
    ("Linux", "x86_64"): "ubuntu-x86",
    ("Darwin", "arm64"): "mac-arm64",
}


@dataclass(frozen=True)
class RuntimeRelease:
    """A release version and downloadable asset for one supported platform."""

    platform: str
    version: str
    download_url: str


def _fetch_github_releases() -> list[RuntimeRelease]:
    response = requests.get(
        RELEASES_URL,
        headers={"Accept": "application/vnd.github+json"},
        timeout=30,
    )
    response.raise_for_status()

    data = response.json()
    if not isinstance(data, list):
        raise ValueError("GitHub releases response must be a list")

    releases: list[RuntimeRelease] = []

    for release in data:
        version = release.get("tag_name")
        if not isinstance(version, str):
            continue

        for asset in release.get("assets", []):
            url = asset.get("browser_download_url")
            if not isinstance(url, str) or "unix_full_system" not in url:
                continue

            for target in _RELEASE_TARGETS.values():
                if target in url:
                    releases.append(RuntimeRelease(target, version, url))
                    break

    return releases


@dataclass(frozen=True)
class CatalogSnapshot:
    """Runtime discovery results from the packaged local and installed manifests."""

    selectable_manifests: list[RuntimeManifest]
    manifests: list[RuntimeManifest]
    diagnostics: list[ManifestError]

    def lookup(self, id: str) -> RuntimeManifest:
        """Get a selectable runtime by ID"""
        for manifest in self.selectable_manifests:
            if manifest.id == id:
                return manifest

        raise ValueError(f"ID {id!r} does not correspond to a single runtime")


class RuntimeCatalog:
    """Discover manifest-backed runtimes and compatible remote releases"""

    def get(self) -> CatalogSnapshot:
        """Discover runtimes from manifests"""
        manifests, diagnostics = self._read_manifests()
        selectable = self._filter_selectable_manifests(manifests, diagnostics)
        return CatalogSnapshot(selectable, manifests, diagnostics)

    def fetch_releases(self) -> list[RuntimeRelease]:
        """Fetch compatible Thunderbots releases from GitHub for installation"""
        platform_name = platform.system()
        machine = platform.machine()

        target = _RELEASE_TARGETS.get((platform_name, machine))
        if target is None:
            raise ValueError(f"No release assets for {platform_name}/{machine}")

        releases: list[RuntimeRelease] = []
        seen: set[str] = set()

        for release in _fetch_github_releases():
            version = release.version
            if release.platform != target or not version or version in seen:
                continue

            seen.add(version)

            releases.append(release)
            if len(releases) == MAX_RELEASES_FETCHED:
                break

        return releases

    def _read_manifests(
        self,
    ) -> tuple[list[RuntimeManifest], list[ManifestError]]:
        local_path = Path(__file__).with_name("local_runtime.json")
        local = RuntimeManifest.from_file(local_path)

        manifests = [local]
        diagnostics: list[ManifestError] = []

        for path in sorted((RUNTIME_ROOT / "manifests").glob("*.json")):
            try:
                manifest = RuntimeManifest.from_file(path)
                if manifest.id == LOCAL_RUNTIME_ID:
                    raise ManifestError(
                        path,
                        "id",
                        "reserved for the packaged local runtime",
                    )

                manifests.append(manifest)
            except ManifestError as error:
                diagnostics.append(error)

        return manifests, diagnostics

    def _filter_selectable_manifests(
        self,
        manifests: list[RuntimeManifest],
        diagnostics: list[ManifestError],
    ) -> list[RuntimeManifest]:
        by_id: defaultdict[str, list[RuntimeManifest]] = defaultdict(list)
        for manifest in manifests:
            by_id[manifest.id].append(manifest)

        selectable: list[RuntimeManifest] = []

        for manifest in manifests:
            same_id = by_id[manifest.id]
            if len(same_id) > 1:
                paths = ", ".join(str(entry.manifest_path) for entry in same_id)
                diagnostics.append(
                    ManifestError(
                        manifest.manifest_path,
                        "id",
                        f"duplicate ID {manifest.id}: {paths}",
                    )
                )
                continue

            # the local runtime should stay selectable before the binary is built, so
            # only run availability check on external runtimes
            if manifest.id != LOCAL_RUNTIME_ID:
                try:
                    manifest.check_available()
                except ManifestError as error:
                    diagnostics.append(error)
                    continue

            selectable.append(manifest)

        selectable.sort(
            key=lambda entry: (entry.id != LOCAL_RUNTIME_ID, entry.name, entry.id),
        )
        return selectable
