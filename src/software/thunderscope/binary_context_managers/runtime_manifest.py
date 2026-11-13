import json
import math
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, NoReturn, cast

type _JsonObj = dict[str, Any]


class ManifestError(ValueError):
    """Identifies an invalid manifest field and the error cause"""

    def __init__(self, manifest_path: Path, field: str, reason: str) -> None:
        self.manifest_path = manifest_path
        self.field = field
        self.reason = reason
        super().__init__(f"{manifest_path}: {field}: {reason}")


@dataclass(frozen=True)
class RuntimeManifest:
    """A validated runtime manifest, with a potentially absent executable"""

    id: str
    name: str
    version: str

    sockets: list[str]
    timeout_seconds: float

    launch_path: str
    manifest_path: Path

    @staticmethod
    def from_file(path: Path) -> "RuntimeManifest":
        """Read and validate a runtime manifest file."""
        return _ManifestParser(path).parse()

    def executable_path(self) -> Path:
        """Get the runtime's executable path relative to its manifest directory"""
        return (self.manifest_path.parent / self.launch_path).resolve()

    def check_available(self) -> None:
        """Raise a manifest diagnostic if the executable is not available"""
        try:
            path = self.executable_path()
            available = path.is_file() and os.access(path, os.X_OK)
        except (OSError, RuntimeError) as error:
            raise ManifestError(
                self.manifest_path,
                "launch_path",
                str(error),
            ) from error

        if not available:
            raise ManifestError(
                self.manifest_path,
                "launch_path",
                f"not an executable file: {path}",
            )


class _ManifestParser:
    def __init__(self, path: Path) -> None:
        self.path = path

    def parse(self) -> RuntimeManifest:
        data = self._read_document()
        runtime_id, name, version = self._parse_base(data)
        sockets, timeout = self._parse_monitor(data["monitor"])
        launch_path = self._expect_str(data, "launch_path")

        return RuntimeManifest(
            id=runtime_id,
            name=name,
            version=version,
            sockets=sockets,
            timeout_seconds=timeout,
            launch_path=launch_path,
            manifest_path=self.path.absolute(),
        )

    def _fail(self, field: str, reason: str) -> NoReturn:
        raise ManifestError(self.path, field, reason)

    def _expect_object(
        self,
        value: object,
        expected: set[str],
        prefix: str = "",
    ) -> _JsonObj:
        if not isinstance(value, dict):
            self._fail(prefix.rstrip(".") or "$", "expected an object")

        obj = cast(_JsonObj, value)
        for key in sorted(expected):
            if key not in obj:
                self._fail(prefix + key, "required field is missing")

        for key in obj:
            if key not in expected:
                self._fail(prefix + key, "unknown field")

        return obj

    def _expect_str(self, data: _JsonObj, key: str) -> str:
        value = data[key]
        if not isinstance(value, str):
            self._fail(key, "expected a string")

        return value

    def _read_document(self) -> _JsonObj:
        try:

            def unique_keys(pairs: list[tuple[str, Any]]) -> _JsonObj:
                obj: _JsonObj = {}
                for k, v in pairs:
                    if k in obj:
                        self._fail(k, "duplicate JSON key")
                    obj[k] = v
                return obj

            data = json.loads(
                self.path.read_text(),
                object_pairs_hook=unique_keys,
            )
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            self._fail("$", str(error))

        return self._expect_object(
            data,
            {
                "schema_version",
                "id",
                "name",
                "version",
                "monitor",
                "launch_path",
            },
        )

    def _parse_base(self, data: _JsonObj) -> tuple[str, str, str]:
        if type(data["schema_version"]) is not int or data["schema_version"] != 1:
            self._fail("schema_version", "unsupported schema version")

        runtime_id = data["id"]
        if not isinstance(runtime_id, str):
            self._fail("id", "expected string")
        if re.fullmatch(r"[a-z0-9._-]+", runtime_id) is None:
            self._fail("id", "expected characters from [a-z0-9._-]")

        name = self._expect_str(data, "name")
        version = self._expect_str(data, "version")

        return runtime_id, name, version

    def _parse_monitor(self, value: object) -> tuple[list[str], float]:
        monitor = self._expect_object(
            value,
            {"kind", "sockets", "timeout_seconds"},
            "monitor.",
        )
        if monitor["kind"] != "unix_datagram":
            self._fail("monitor.kind", "only unix_datagram is supported")

        sockets = self._parse_sockets(monitor["sockets"])
        timeout = self._parse_timeout(monitor["timeout_seconds"])

        return sockets, timeout

    def _parse_sockets(self, value: object) -> list[str]:
        if not isinstance(value, list) or not value:
            self._fail("monitor.sockets", "expected a nonempty list")

        for i, entry in enumerate(value):
            if (
                not isinstance(entry, str)
                or Path(entry).is_absolute()
                or ".." in Path(entry).parts
                or entry == "."
            ):
                self._fail(
                    f"monitor.sockets[{i}]",
                    "expected a relative socket path within the runtime directory",
                )

        return cast(list[str], value)

    def _parse_timeout(self, value: object) -> float:
        if type(value) not in (int, float):
            self._fail("monitor.timeout_seconds", "expected a positive finite number")

        number = cast(int | float, value)
        try:
            timeout = float(number)
        except OverflowError:
            self._fail("monitor.timeout_seconds", "expected a positive finite number")

        if not math.isfinite(timeout) or timeout <= 0:
            self._fail("monitor.timeout_seconds", "expected a positive finite number")

        return timeout
