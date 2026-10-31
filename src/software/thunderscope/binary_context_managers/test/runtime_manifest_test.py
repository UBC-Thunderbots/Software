import json
from pathlib import Path
from typing import Any

import pytest
from software.gameplay_tests.util import pytest_main
from software.thunderscope.binary_context_managers.runtime_manifest import (
    ManifestError,
    RuntimeManifest,
)


@pytest.fixture
def manifest_path(tmp_path: Path) -> Path:
    path = tmp_path / "manifests" / "runtime.json"
    path.parent.mkdir()
    return path


def test_relative_executable_path(
    tmp_path: Path,
    manifest_path: Path,
    manifest_data: dict[str, Any],
) -> None:
    binary = tmp_path / "unix_full_system_v1"
    binary.write_text("#!/bin/sh\nexit 0\n")
    binary.chmod(0o755)
    manifest_path.write_text(json.dumps(manifest_data))

    manifest = RuntimeManifest.from_file(manifest_path)
    assert manifest.executable_path() == binary
    assert manifest.version == "v1"
    manifest.check_available()


@pytest.mark.parametrize(
    "field, value",
    [
        ("schema_version", True),
        ("id", "Bad ID"),
        ("name", None),
        ("launch_path", None),
        ("monitor.kind", "exists"),
        ("monitor.timeout_seconds", -1),
        ("monitor.timeout_seconds", float("inf")),
        ("monitor.timeout_seconds", True),
        ("monitor.sockets", []),
        ("monitor.sockets", ["../escape"]),
        ("monitor.sockets", ["/absolute"]),
    ],
)
def test_invalid_field_diagnostics(
    manifest_path: Path,
    manifest_data: dict[str, Any],
    field: str,
    value: object,
) -> None:
    parent = manifest_data
    *parent_keys, key = field.split(".")
    for parent_key in parent_keys:
        parent = parent[parent_key]
    parent[key] = value

    manifest_path.write_text(json.dumps(manifest_data))
    with pytest.raises(ManifestError) as raised:
        RuntimeManifest.from_file(manifest_path)

    assert str(manifest_path) in str(raised.value)
    assert raised.value.field.startswith(field)
    assert raised.value.reason


@pytest.mark.parametrize(
    "content, field",
    [("{", "$"), ('{"id": 1, "id": 2}', "id")],
)
def test_invalid_json(
    manifest_path: Path,
    content: str,
    field: str,
) -> None:
    manifest_path.write_text(content)
    with pytest.raises(ManifestError) as raised:
        RuntimeManifest.from_file(manifest_path)

    assert raised.value.field == field


def test_missing_unknown_fields(
    manifest_path: Path,
    manifest_data: dict[str, Any],
) -> None:
    del manifest_data["name"]

    manifest_path.write_text(json.dumps(manifest_data))
    with pytest.raises(ManifestError, match="name: required"):
        RuntimeManifest.from_file(manifest_path)

    manifest_data["name"] = "test"
    manifest_data["typo"] = 1

    manifest_path.write_text(json.dumps(manifest_data))
    with pytest.raises(ManifestError, match="typo: unknown"):
        RuntimeManifest.from_file(manifest_path)


def test_parse_unavailable_executable(
    tmp_path: Path,
    manifest_path: Path,
    manifest_data: dict[str, Any],
) -> None:
    binary = tmp_path / "binary"
    manifest_data["launch_path"] = str(binary)
    manifest_path.write_text(json.dumps(manifest_data))
    manifest = RuntimeManifest.from_file(manifest_path)

    # nonexistent at this point
    with pytest.raises(ManifestError, match="launch_path"):
        manifest.check_available()

    binary.write_text("exist but not executable")
    with pytest.raises(ManifestError):
        manifest.check_available()


if __name__ == "__main__":
    pytest_main(__file__)
