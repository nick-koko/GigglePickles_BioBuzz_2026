#!/usr/bin/env python3
"""Convert 2026 match/detail CSVs to a modern, typed AdvantageScope WPILOG.

Python standard library only. No changes to the robot or WPILib dependency needed.
Format: https://github.com/wpilibsuite/allwpilib/blob/v2026.2.2/wpiutil/doc/datalog.adoc
Geometry schemas: WPILib Translation2d, Rotation2d and Pose2d structs.
"""
from __future__ import annotations

import argparse
import csv
import glob
import heapq
import math
from pathlib import Path
import struct
import sys
from typing import BinaryIO, Iterator


class WpiLog:
    """Small write-only implementation of the public WPILOG 1.0 format."""
    def __init__(self, stream: BinaryIO):
        self.stream = stream
        self.entries: dict[str, tuple[int, str]] = {}
        self.last: dict[str, bytes] = {}
        extra = b"Pickles 2026 CSV conversion; measured data, no simulated ball trajectories"
        stream.write(b"WPILOG" + struct.pack("<HI", 0x0100, len(extra)) + extra)

    def record(self, entry: int, timestamp_us: int, data: bytes) -> None:
        # Fixed legal field widths: ID=4, length=4, timestamp=8. Bit 7 remains zero.
        self.stream.write(b"\x7f" + struct.pack("<IIQ", entry, len(data), timestamp_us) + data)

    @staticmethod
    def text(value: str) -> bytes:
        data = value.encode("utf-8")
        return struct.pack("<I", len(data)) + data

    def put(self, name: str, kind: str, data: bytes, timestamp_us: int, force: bool = False) -> None:
        if name not in self.entries:
            entry = len(self.entries) + 1
            control = b"\0" + struct.pack("<I", entry) + self.text(name) + self.text(kind) + self.text("")
            self.record(0, timestamp_us, control)
            self.entries[name] = (entry, kind)
        entry, original_kind = self.entries[name]
        if kind != original_kind:
            raise ValueError(f"Field type changed: {name}: {original_kind} -> {kind}")
        if force or self.last.get(name) != data:
            self.record(entry, timestamp_us, data)
            self.last[name] = data

    def geometry_schemas(self) -> None:
        for name, schema in (
            ("Translation2d", "double x;double y"),
            ("Rotation2d", "double value"),
            ("Pose2d", "Translation2d translation;Rotation2d rotation"),
        ):
            self.put("/.schema/struct:" + name, "structschema", schema.encode(), 0)


TEXT_FIELDS = set("session_id opmode phase profile event note stage plan rpm_source "
                  "sequence_end_reason intake_mode flower_position turret_mode turret_state "
                  "intake_applied_control_mode manual_motor_test turret_status".split())


def scalar(value: str, field: str) -> tuple[str, bytes] | None:
    if value is None or value == "":
        return None
    if field in TEXT_FIELDS:
        return "string", value.encode("utf-8")
    if value in ("true", "false"):
        return "boolean", bytes([value == "true"])
    try:
        return "double", struct.pack("<d", float(value))
    except ValueError as exception:
        raise ValueError(f"Invalid numeric value in {field}: {value!r}") from exception


def finite(row: dict[str, str], key: str) -> float | None:
    try:
        value = float(row.get(key, ""))
        return value if math.isfinite(value) else None
    except (ValueError, TypeError):
        return None


def transform(x: float, y: float, heading_deg: float, frame: str) -> tuple[float, float, float]:
    if frame == "ftc":
        # Matches this repository's existing converter: center origin and +90 degree rotation.
        return (72 - y) * 0.0254, (x - 72) * 0.0254, math.radians(heading_deg + 90)
    return x * 0.0254, y * 0.0254, math.radians(heading_deg)


def input_files(patterns: list[str]) -> list[Path]:
    result: set[Path] = set()
    for pattern in patterns:
        for name in glob.glob(pattern, recursive=True) or [pattern]:
            path = Path(name)
            if path.is_dir():
                result.update(p.resolve() for p in path.rglob("*.csv")
                              if p.name in ("match.csv", "shot_detail.csv"))
            elif path.is_file():
                result.add(path.resolve())
            else:
                raise ValueError(f"Input not found: {pattern}")
    if not result:
        raise ValueError("No match.csv or shot_detail.csv files found")
    return sorted(result)


def file_info(path: Path) -> tuple[str, bool]:
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        first = next(reader, None)
        if first is None or first.get("schema_version") != "1":
            raise ValueError(f"Not a nonempty 2026 schema-1 log: {path}")
        return first["session_id"], "controller_power_before_boost" in reader.fieldnames


def rows(path: Path, timeline: str) -> Iterator[tuple[float, dict[str, str]]]:
    previous = -math.inf
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            timestamp = finite(row, "epoch_ms" if timeline == "wall" else "match_t_ms")
            if timestamp is None or None in row or any(value is None for value in row.values()):
                print(f"Skipping incomplete row {reader.line_num}: {path}", file=sys.stderr)
                continue
            if timestamp < previous:
                raise ValueError(f"Time moved backwards at row {reader.line_num}: {path}")
            previous = timestamp
            yield timestamp, row


def convert(paths: list[Path], output: Path, timeline: str = "run", frame: str = "ftc") -> int:
    info = {path: file_info(path) for path in paths}
    sessions = {session for session, _ in info.values()}
    if timeline == "run" and len(sessions) != 1:
        raise ValueError("Different runs selected. Use --timeline wall to join auto/teleop or practice runs.")
    def source(index: int, path: Path):
        for timestamp, row in rows(path, timeline):
            yield timestamp, index, row
    merged = heapq.merge(*(source(index, path) for index, path in enumerate(paths)),
                         key=lambda item: (item[0], item[1]))
    count, origin = 0, None
    shot_poses: list[float] = []
    seen_sequences: set[tuple[str, str]] = set()
    with output.open("wb") as stream:
        log = WpiLog(stream)
        log.geometry_schemas()
        log.put("/Replay/Frame", "string", frame.encode(), 0)
        for timestamp, index, row in merged:
            if origin is None:
                # Single run retains time since Play; wall merges start at the first logged sample.
                origin = timestamp if timeline == "wall" else 0.0
            time_us = max(0, round((timestamp - origin) * 1000))
            path = paths[index]
            session, detail = info[path]
            prefix = "/ShotDetail/" if detail else "/Match/"
            for field, value in row.items():
                encoded = scalar(value, field)
                if encoded is not None:
                    log.put(prefix + field, encoded[0], encoded[1], time_us,
                            force=field in ("event", "note"))
            count += 1
            log.put("/Replay/Enabled", "boolean", b"\0" if row.get("event") == "SESSION_END" else b"\1", time_us)
            log.put("/Replay/Autonomous", "boolean", bytes([row.get("phase") == "AUTO"]), time_us)
            coordinates = [finite(row, key) for key in ("robot_x_in", "robot_y_in", "robot_heading_deg")]
            if all(value is not None for value in coordinates):
                x, y, heading = transform(*coordinates, frame)
                log.put("/Replay/RobotPose", "struct:Pose2d", struct.pack("<ddd", x, y, heading), time_us)
                relative = finite(row, "turret_robot_relative_deg")
                if relative is not None:
                    log.put("/Replay/ShooterDirection", "struct:Pose2d",
                            struct.pack("<ddd", x, y, heading + math.radians(relative)), time_us)
                sequence = (session, row.get("sequence_id", "0"))
                if sequence[1] not in ("0", "0.0", "") and sequence not in seen_sequences:
                    seen_sequences.add(sequence)
                    shot_poses.extend((x, y, heading))
                    log.put("/Replay/CommandedVolleyLocations", "struct:Pose2d[]",
                            struct.pack("<" + "d" * len(shot_poses), *shot_poses), time_us)
            tx, ty = finite(row, "target_x_in"), finite(row, "target_y_in")
            if tx is not None and ty is not None:
                tx, ty, _ = transform(tx, ty, 0, frame)
                log.put("/Replay/TargetReference", "struct:Translation2d", struct.pack("<dd", tx, ty), time_us)
    return count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", nargs="+", required=True, help="Run folders, CSV paths or globs")
    parser.add_argument("--output", type=Path, required=True, help="Output .wpilog file")
    parser.add_argument("--timeline", choices=("run", "wall"), default="run")
    parser.add_argument("--frame", choices=("ftc", "pedro"), default="ftc")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    try:
        paths = input_files(args.input)
        if args.output.resolve() in paths:
            raise ValueError("Output must not replace an input")
        if args.output.exists() and not args.overwrite:
            raise ValueError("Output exists; choose a new name or use --overwrite")
        count = convert(paths, args.output, args.timeline, args.frame)
        print(f"Converted {len(paths)} input files; {count} replay rows -> {args.output}")
        return 0
    except (OSError, ValueError) as exception:
        print(f"Conversion failed: {exception}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
