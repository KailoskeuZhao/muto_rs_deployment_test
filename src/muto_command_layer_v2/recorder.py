"""High-level mission recorder.

The recorder accepts already-typed board/event projections and writes a
mission-scoped manifest before the first board record. It deliberately does
not subscribe to raw sensors, call tools, or influence terminal state. The ROS
wrapper adds rosbag2/MCAP lifecycle and recorder-status publication.
"""

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Iterable, List, Optional, TextIO

from .contracts import MissionBoard, MissionEvent, SCHEMA_VERSION


@dataclass(frozen=True)
class RecorderManifest:
    schema_version: str = SCHEMA_VERSION
    mission_id: str = ""
    request_id: str = ""
    build_revision: str = "unknown"
    commander_model: str = "unknown"
    profile: str = "high_level"
    scenario_id: str = ""
    completion_policy: str = ""


class HighLevelRecorder:
    """Append board and event projections to an optional JSONL file."""

    def __init__(
        self,
        path: Optional[str] = None,
        *,
        build_revision: str = "unknown",
        commander_model: str = "unknown",
    ) -> None:
        self._path = Path(path) if path else None
        self._stream: Optional[TextIO] = None
        self._records: List[dict] = []
        self._available = path is None
        self._manifest = RecorderManifest(
            build_revision=build_revision,
            commander_model=commander_model,
        )

    @property
    def records(self):
        return tuple(self._records)

    @property
    def available(self) -> bool:
        return self._available

    def start(self, board: MissionBoard) -> None:
        # A recorder instance may be reused by a long-lived node, but each
        # mission gets an independent output stream and manifest.
        if self._stream is not None:
            self.close()
        self._manifest = RecorderManifest(
            mission_id=board.mission_id,
            request_id=board.request_id,
            build_revision=self._manifest.build_revision,
            commander_model=self._manifest.commander_model,
            scenario_id=board.scenario_id,
            completion_policy=(
                board.completion_policy.value if board.completion_policy else ""
            ),
        )
        if self._path is not None:
            try:
                self._path.parent.mkdir(parents=True, exist_ok=True)
                self._stream = self._path.open("w", encoding="utf-8")
                self._available = True
            except OSError:
                self._available = False
        self._write("manifest", asdict(self._manifest))
        self.record_board(board)

    def record_event(self, event: MissionEvent) -> None:
        self._write("event", _enum_safe(event))

    def record_board(self, board: MissionBoard) -> None:
        self._write("board", _enum_safe(board))

    def close(self) -> None:
        if self._stream is not None:
            self._stream.flush()
            self._stream.close()
            self._stream = None

    def _write(self, kind: str, payload: dict) -> None:
        record = {"kind": kind, "schema_version": SCHEMA_VERSION, "payload": payload}
        self._records.append(record)
        if self._stream is not None:
            self._stream.write(json.dumps(record, sort_keys=True) + "\n")
            self._stream.flush()


def _enum_safe(value):
    if hasattr(value, "__dataclass_fields__"):
        return {name: _enum_safe(getattr(value, name)) for name in value.__dataclass_fields__}
    if isinstance(value, tuple):
        return [_enum_safe(item) for item in value]
    if isinstance(value, list):
        return [_enum_safe(item) for item in value]
    if hasattr(value, "value"):
        return value.value
    return value
