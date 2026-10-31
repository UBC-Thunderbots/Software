import json
from pathlib import Path
from typing import Any
from unittest.mock import Mock

import pytest
from software.gameplay_tests.util import pytest_main
from software.thunderscope.binary_context_managers import runtime_catalog
from software.thunderscope.binary_context_managers.runtime_catalog import (
    RuntimeCatalog,
    RuntimeRelease,
)


@pytest.fixture
def catalog(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> RuntimeCatalog:
    monkeypatch.setattr(runtime_catalog, "RUNTIME_ROOT", tmp_path)
    monkeypatch.setattr(runtime_catalog.platform, "system", lambda: "Linux")
    monkeypatch.setattr(runtime_catalog.platform, "machine", lambda: "x86_64")
    return RuntimeCatalog()


def _write_manifest(root: Path, name: str, data: dict[str, Any]) -> Path:
    path = root / "manifests" / name
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(data))
    return path


def _release(version: str, target: str) -> RuntimeRelease:
    return RuntimeRelease(
        platform=target,
        version=version,
        download_url=f"https://example.test/{version}/unix_full_system_{target}.tar.gz",
    )


def test_catalog_skips_invalid_manifests(
    tmp_path: Path,
    catalog: RuntimeCatalog,
    manifest_data: dict[str, Any],
) -> None:
    binary = tmp_path / "unix_full_system_v1"
    binary.write_text("#!/bin/sh\n")
    binary.chmod(0o755)
    _write_manifest(tmp_path, "valid.json", manifest_data)
    (tmp_path / "manifests" / "broken.json").write_text("{")

    snapshot = catalog.get()
    manifests = snapshot.selectable_manifests

    assert [entry.name for entry in manifests] == [
        "Current Fullsystem",
        "Test runtime",
    ]
    assert len(snapshot.diagnostics) == 1
    assert "broken.json" in str(snapshot.diagnostics[0])

    assert snapshot.lookup(manifests[1].id) == manifests[1]


def test_catalog_rejects_duplicate_ids(
    tmp_path: Path,
    catalog: RuntimeCatalog,
    manifest_data: dict[str, Any],
) -> None:
    for name in ("a.json", "b.json"):
        _write_manifest(tmp_path, name, manifest_data)

    snapshot = catalog.get()
    assert len(snapshot.selectable_manifests) == 1
    assert len(snapshot.manifests) == 3
    assert len(snapshot.diagnostics) == 2

    assert "a.json" in str(snapshot.diagnostics[0])
    assert "b.json" in str(snapshot.diagnostics[0])

    with pytest.raises(ValueError):
        snapshot.lookup("test.runtime")


def test_catalog_rejects_reserved_id(
    tmp_path: Path,
    catalog: RuntimeCatalog,
    manifest_data: dict[str, Any],
) -> None:
    manifest_data["id"] = "thunderbots.local"
    path = _write_manifest(tmp_path, "reserved.json", manifest_data)

    snapshot = catalog.get()
    assert len(snapshot.manifests) == 1
    assert len(snapshot.diagnostics) == 1
    assert snapshot.lookup("thunderbots.local").version == "local"

    error = snapshot.diagnostics[0]
    assert error.manifest_path == path
    assert error.field == "id"
    assert "reserved" in error.reason


@pytest.mark.parametrize(
    "platform_name, machine, target",
    [
        ("Linux", "x86_64", "ubuntu-x86"),
        ("Darwin", "arm64", "mac-arm64"),
    ],
)
def test_fetch_releases_filter_deduplicate_limit(
    catalog: RuntimeCatalog,
    monkeypatch: pytest.MonkeyPatch,
    platform_name: str,
    machine: str,
    target: str,
) -> None:
    releases = [
        _release(f"v{i}", target)
        for i in range(8)
        for target in ("mac-arm64", "ubuntu-x86")
    ]
    monkeypatch.setattr(
        runtime_catalog,
        "_fetch_github_releases",
        lambda: [releases[0], *releases],
    )
    monkeypatch.setattr(runtime_catalog.platform, "system", lambda: platform_name)
    monkeypatch.setattr(runtime_catalog.platform, "machine", lambda: machine)

    assert catalog.fetch_releases() == [_release(f"v{i}", target) for i in range(5)]


def test_fetch_releases_parses_assets(
    catalog: RuntimeCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    url = "https://example.test/v1/unix_full_system_ubuntu-x86.tar.gz"
    mac_url = "https://example.test/v1/unix_full_system_mac-arm64.tar.gz"
    response = Mock()
    response.json.return_value = [
        {"tag_name": None},
        {"tag_name": ""},
        {"tag_name": "no-assets"},
        {
            "tag_name": "v1",
            "assets": [
                {"browser_download_url": None},
                {"browser_download_url": "https://example.test/notes.txt"},
                {"browser_download_url": mac_url},
                {"browser_download_url": url},
            ],
        },
    ]
    monkeypatch.setattr(runtime_catalog.requests, "get", Mock(return_value=response))

    assert catalog.fetch_releases() == [RuntimeRelease("ubuntu-x86", "v1", url)]
    response.raise_for_status.assert_called_once()


if __name__ == "__main__":
    pytest_main(__file__)
