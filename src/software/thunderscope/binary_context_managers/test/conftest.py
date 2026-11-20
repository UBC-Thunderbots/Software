from typing import Any

import pytest


@pytest.fixture
def manifest_data() -> dict[str, Any]:
    return {
        "schema_version": 1,
        "id": "test.runtime",
        "name": "Test runtime",
        "version": "v1",
        "monitor": {
            "kind": "unix_datagram",
            "sockets": ["sensor_proto", "play_override"],
            "timeout_seconds": 10,
        },
        "launch_path": "../unix_full_system_v1",
    }
