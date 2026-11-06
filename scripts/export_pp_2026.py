#!/usr/bin/env python3
"""
Export the 2026 autonomous OpModes into Pedro visualizer (.pp v2.0.0) files.

For each OpMode the script reads:
- poses:   private final Pose name = poseFactory.of(x, y, deg);
- paths:   public Path name() { return line(...)/curve(...).linear(...)/.heading(...); }
- chains:  chain = PedroComponent.follower().pathBuilder().addPath(name())...build();
- order:   autonomousSequence() and the Command helper methods it calls.

Every selectable branch becomes its own .pp file:
- init selections written as `flag ? a() : b()`
- runtime branches written as `new IfElseCommand(condition, a, b)`

Delays in a sequence become the previous path's "wait after" time, and each
shot adds the shot feed time, so the visualizer timeline is roughly right.
Delays inside a ParallelGroup and WaitUntil conditions (partner delays) are
ignored.

Usage (from the repo root):
    python scripts/export_pp_2026.py
    python scripts/export_pp_2026.py --java TeamCode/.../FarWithPartner.java --outdir pedro_paths
"""

import argparse
import ast
import datetime
import itertools
import json
import operator
import pathlib
import re
import sys
import uuid
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
PEDRO_DIR = REPO_ROOT / "TeamCode" / "src" / "main" / "java" / "org" / "firstinspires" / "ftc" / "teamcode" / "pedroPathing"
SHOT_CONTROLLER = (
    REPO_ROOT / "TeamCode" / "src" / "main" / "java" / "org" / "firstinspires" / "ftc" / "teamcode"
    / "subsystems" / "ShotSequenceController2026.java"
)
DEFAULT_OPMODES = ["CloseShootingAlone.java", "CloseWithPartner.java", "FarWithPartner.java"]

# Readable names for branch conditions: key -> (name when true, name when false).
BRANCH_LABELS = {
    "shootGardenBallsFromFar": ("GardenShotFar", "GardenShotClose"),
    "helpShootFar": ("HelpFar", "TrustPartner"),
    "partnerOnlyShootsPreloads": ("PartnerPreloadsOnly", "TwoFarShotsPark"),
    "hasTimeForFinalCloseShot": ("FinalCloseShot", "SkipFinalShot"),
    "getBallCount": ("GardenRetry", "NoGardenRetry"),
}

COLORS = ["#9c755f", "#ffc516", "#b07aa1", "#4e79a7", "#59a14f", "#f28e2b",
          "#76b7b2", "#e15759", "#edc948", "#ff9da7"]

FIELD_SHAPES = [
    {
        "id": "triangle-1",
        "name": "Red Goal",
        "vertices": [{"x": 141.5, "y": 70}, {"x": 141.5, "y": 141.5}, {"x": 118.3, "y": 141.5},
                     {"x": 135.5, "y": 118}, {"x": 136.3, "y": 70.2}],
        "color": "#dc2626",
        "fillColor": "#ff6b6b",
    },
    {
        "id": "triangle-2",
        "name": "Blue Goal",
        "vertices": [{"x": 6.2, "y": 116.9}, {"x": 25, "y": 141.5}, {"x": 0, "y": 141.5},
                     {"x": 0, "y": 70}, {"x": 6, "y": 70}],
        "color": "#2563eb",
        "fillColor": "#60a5fa",
    },
]


# ---------------------------------------------------------------------------
# Small Java text helpers
# ---------------------------------------------------------------------------

def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def matching_paren(text: str, open_idx: int) -> int:
    """Index of the ')' matching the '(' at open_idx."""
    depth = 0
    in_string = False
    for i in range(open_idx, len(text)):
        ch = text[i]
        if ch == '"' and text[i - 1] != "\\":
            in_string = not in_string
        if in_string:
            continue
        if ch in "([{":
            depth += 1
        elif ch in ")]}":
            depth -= 1
            if depth == 0:
                return i
    raise ValueError(f"Unbalanced parentheses near: {text[open_idx:open_idx + 60]}")


def split_top_level(text: str, sep: str = ",") -> List[str]:
    parts, depth, start = [], 0, 0
    for i, ch in enumerate(text):
        if ch in "([{":
            depth += 1
        elif ch in ")]}":
            depth -= 1
        elif ch == sep and depth == 0:
            parts.append(text[start:i].strip())
            start = i + 1
    tail = text[start:].strip()
    if tail:
        parts.append(tail)
    return parts


def find_top_level_ternary(text: str) -> Optional[Tuple[str, str, str]]:
    """Split `cond ? a : b` at depth 0, ignoring `::` method references."""
    depth, question = 0, -1
    i = 0
    while i < len(text):
        ch = text[i]
        if ch in "([{":
            depth += 1
        elif ch in ")]}":
            depth -= 1
        elif depth == 0:
            if ch == "?" and question < 0:
                question = i
            elif ch == ":" and question >= 0:
                if text[i:i + 2] == "::" or text[i - 1] == ":":
                    i += 2
                    continue
                return text[:question].strip(), text[question + 1:i].strip(), text[i + 1:].strip()
        i += 1
    return None


_OPS = {ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul, ast.Div: operator.truediv}


def eval_number(expr: str, constants: Dict[str, float]) -> float:
    """Evaluate simple Java arithmetic using numeric literals and static constants."""
    expr = re.sub(r"(?<=\d)[dDfF]\b", "", expr.strip())

    def walk(node):
        if isinstance(node, ast.Expression):
            return walk(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
            return float(node.value)
        if isinstance(node, ast.Name):
            if node.id in constants:
                return constants[node.id]
            raise KeyError(f"Unknown constant {node.id}")
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
            return -walk(node.operand)
        if isinstance(node, ast.BinOp) and type(node.op) in _OPS:
            return _OPS[type(node.op)](walk(node.left), walk(node.right))
        raise ValueError(f"Unsupported expression: {expr}")

    return walk(ast.parse(expr, mode="eval"))


# ---------------------------------------------------------------------------
# OpMode parsing
# ---------------------------------------------------------------------------

@dataclass
class PathDef:
    name: str
    points: List[str]          # pose names: start, controls..., end
    heading: dict              # pp heading object (without pose resolution)


@dataclass
class OpModeSource:
    name: str
    constants: Dict[str, float]
    poses: Dict[str, Tuple[float, float, float]]
    paths: Dict[str, PathDef]
    chains: Dict[str, List[str]]
    commands: Dict[str, Tuple[List[str], str]]   # name -> (param names, return expression)


def parse_constants(text: str) -> Dict[str, float]:
    constants = {}
    for m in re.finditer(r"static\s+(?:final\s+)?(?:double|int|long|float)\s+(\w+)\s*=\s*([^;]+);", text):
        try:
            constants[m.group(1)] = eval_number(m.group(2), constants)
        except (KeyError, ValueError, SyntaxError):
            pass
    return constants


def parse_poses(text: str, constants: Dict[str, float]) -> Dict[str, Tuple[float, float, float]]:
    poses = {}
    for m in re.finditer(r"Pose\s+(\w+)\s*=\s*poseFactory\.of\(", text):
        open_idx = m.end() - 1
        args = split_top_level(text[open_idx + 1:matching_paren(text, open_idx)])
        values = [eval_number(a, constants) for a in args]
        heading = values[2] if len(values) > 2 else 0.0
        poses[m.group(1)] = (values[0], values[1], heading)
    return poses


def parse_interpolator(expr: str) -> dict:
    """Interpolator.linear(a, b) / constant(p) / tangent() -> pp piecewise segment parts."""
    expr = expr.strip()
    m = re.fullmatch(r"Interpolator\.linear\(\s*(\w+)\s*,\s*(\w+)\s*\)", expr)
    if m:
        return {"interpolationType": "linear", "poses": (m.group(1), m.group(2))}
    m = re.fullmatch(r"Interpolator\.constant\(\s*(\w+)\s*\)", expr)
    if m:
        return {"interpolationType": "constant", "poses": (m.group(1),)}
    if re.fullmatch(r"Interpolator\.tangent(?:\(\s*\))?", expr):
        return {"interpolationType": "tangential", "poses": ()}
    raise ValueError(f"Unsupported interpolator: {expr}")


def parse_heading_suffix(suffix: str) -> dict:
    suffix = suffix.strip()
    m = re.fullmatch(r"\.linear\(\s*(\w+)\s*,\s*(\w+)\s*\)", suffix)
    if m:
        return {"type": "linear", "poses": (m.group(1), m.group(2))}
    m = re.fullmatch(r"\.constant\(\s*(\w+)\s*\)", suffix)
    if m:
        return {"type": "constant", "poses": (m.group(1),)}
    if re.fullmatch(r"\.reverseTangent\(\s*\)", suffix):
        return {"type": "tangential", "reverse": True}
    if re.fullmatch(r"\.tangent\(\s*\)", suffix) or suffix == "":
        return {"type": "tangential", "reverse": False}
    if suffix.startswith(".heading("):
        inner = suffix[len(".heading("):matching_paren(suffix, len(".heading(") - 1)].strip()
        if inner.startswith("Interpolator.piecewise()"):
            segments = []
            rest = inner[len("Interpolator.piecewise()"):]
            for um in re.finditer(r"\.until\(", rest):
                open_idx = um.end() - 1
                args = split_top_level(rest[open_idx + 1:matching_paren(rest, open_idx)])
                segments.append((float(args[0]), parse_interpolator(args[1])))
            return {"type": "piecewise", "segments": segments}
        single = parse_interpolator(inner)
        if single["interpolationType"] == "constant":
            return {"type": "constant", "poses": single["poses"]}
        if single["interpolationType"] == "linear":
            return {"type": "linear", "poses": single["poses"]}
        return {"type": "tangential", "reverse": False}
    raise ValueError(f"Unsupported heading: {suffix}")


def parse_paths(text: str) -> Dict[str, PathDef]:
    paths = {}
    for m in re.finditer(r"public\s+Path\s+(\w+)\s*\(\s*\)\s*\{\s*return\s+", text):
        body_end = text.index(";", m.end())
        expr = text[m.end():body_end].strip()
        head = re.match(r"(?:Paths\.)?(line|curve)\(", expr)
        if not head:
            continue
        open_idx = head.end() - 1
        close_idx = matching_paren(expr, open_idx)
        points = [p.strip() for p in split_top_level(expr[open_idx + 1:close_idx])]
        paths[m.group(1)] = PathDef(m.group(1), points, parse_heading_suffix(expr[close_idx + 1:]))
    return paths


def parse_chains(text: str) -> Dict[str, List[str]]:
    chains = {}
    for m in re.finditer(r"(\w+)\s*=\s*PedroComponent\.follower\(\)\s*\.pathBuilder\(\)(.*?)\.build\(\)\s*;", text, re.S):
        chains[m.group(1)] = re.findall(r"\.addPath\(\s*(\w+)\(\s*\)\s*\)", m.group(2))
    return chains


def parse_commands(text: str) -> Dict[str, Tuple[List[str], str]]:
    commands = {}
    for m in re.finditer(r"(?:private|public|protected)\s+Command\s+(\w+)\s*\(([^)]*)\)\s*\{", text):
        params = [p.split()[-1] for p in split_top_level(m.group(2))] if m.group(2).strip() else []
        body_open = m.end() - 1
        depth = 0
        for i in range(body_open, len(text)):
            if text[i] == "{":
                depth += 1
            elif text[i] == "}":
                depth -= 1
                if depth == 0:
                    body = text[body_open + 1:i]
                    break
        rm = re.search(r"\breturn\b", body)
        if not rm:
            continue
        expr = body[rm.end():].strip()
        expr = expr[:expr.rindex(";")].strip()
        commands[m.group(1)] = (params, expr)
    return commands


def load_opmode(java_path: pathlib.Path, shared_constants: Dict[str, float]) -> OpModeSource:
    text = strip_comments(java_path.read_text(encoding="utf-8"))
    constants = dict(shared_constants)
    constants.update(parse_constants(text))
    return OpModeSource(
        name=java_path.stem,
        constants=constants,
        poses=parse_poses(text, constants),
        paths=parse_paths(text),
        chains=parse_chains(text),
        commands=parse_commands(text),
    )


# ---------------------------------------------------------------------------
# Command tree -> ordered events per branch
# ---------------------------------------------------------------------------

@dataclass
class Event:
    kind: str                  # "path", "wait", "shoot"
    value: object = None       # chain name or seconds
    label: str = ""


@dataclass
class Variant:
    labels: List[str] = field(default_factory=list)
    events: List[Event] = field(default_factory=list)
    # Branch key -> chosen side, so a flag tested twice stays consistent.
    decisions: Dict[str, bool] = field(default_factory=dict)


def with_decision(v: Variant, key: str, value: bool, label: str) -> Variant:
    return Variant([label] + v.labels, v.events, {key: value, **v.decisions})


def branch_key(condition: str) -> str:
    m = re.search(r"::(\w+)", condition)
    if m:
        return m.group(1)
    calls = re.findall(r"(\w+)\s*\(", condition)
    if calls:
        return calls[-1]
    return re.findall(r"\w+", condition)[-1]


def branch_names(condition: str) -> Tuple[str, str]:
    key = branch_key(condition)
    return BRANCH_LABELS.get(key, (key, "not_" + key))


def combine(sequence_of_variant_lists: List[List[Variant]]) -> List[Variant]:
    results = []
    for combo in itertools.product(*sequence_of_variant_lists):
        merged = Variant()
        consistent = True
        for v in combo:
            for key, value in v.decisions.items():
                if merged.decisions.setdefault(key, value) != value:
                    consistent = False
            merged.labels.extend(label for label in v.labels if label not in merged.labels)
            merged.events.extend(v.events)
        if consistent:
            results.append(merged)
    return results


def expand(expr: str, src: OpModeSource, in_parallel: bool = False, depth: int = 0) -> List[Variant]:
    if depth > 40:
        raise RecursionError("Command expansion too deep")
    expr = expr.strip()

    ternary = find_top_level_ternary(expr)
    if ternary:
        cond, when_true, when_false = ternary
        key = branch_key(cond)
        true_name, false_name = branch_names(cond)
        out = [with_decision(v, key, True, true_name)
               for v in expand(when_true, src, in_parallel, depth + 1)]
        out += [with_decision(v, key, False, false_name)
                for v in expand(when_false, src, in_parallel, depth + 1)]
        return out

    m = re.match(r"new\s+(\w+)\s*\(", expr)
    if m:
        cls = m.group(1)
        open_idx = m.end() - 1
        args = split_top_level(expr[open_idx + 1:matching_paren(expr, open_idx)])
        if cls in ("SequentialGroup", "ParallelGroup", "ParallelRaceGroup", "ParallelDeadlineGroup"):
            parallel = in_parallel or cls != "SequentialGroup"
            return combine([expand(a, src, parallel, depth + 1) for a in args])
        if cls == "IfElseCommand":
            key = branch_key(args[0])
            true_name, false_name = branch_names(args[0])
            out = [with_decision(v, key, True, true_name)
                   for v in expand(args[1], src, in_parallel, depth + 1)]
            if len(args) > 2:
                out += [with_decision(v, key, False, false_name)
                        for v in expand(args[2], src, in_parallel, depth + 1)]
            else:
                out.append(Variant([false_name], [], {key: False}))
            return out
        if cls == "FollowPath":
            return [Variant([], [Event("path", args[0].strip())])]
        if cls == "Delay":
            if in_parallel:
                return [Variant()]
            try:
                seconds = eval_number(args[0], src.constants)
            except (KeyError, ValueError, SyntaxError):
                return [Variant()]
            return [Variant([], [Event("wait", seconds, args[0].strip())])]
        if cls == "InstantCommand" and args and "shotSequence::start" in args[0]:
            return [Variant([], [Event("shoot")])]
        return [Variant()]

    m = re.match(r"(?:this\.)?(\w+)\s*\(", expr)
    if m and m.group(1) in src.commands:
        open_idx = m.end() - 1
        call_args = split_top_level(expr[open_idx + 1:matching_paren(expr, open_idx)])
        params, body = src.commands[m.group(1)]
        for param, arg in zip(params, call_args):
            body = re.sub(rf"\b{re.escape(param)}\b", arg, body)
        return expand(body, src, in_parallel, depth + 1)

    return [Variant()]


# ---------------------------------------------------------------------------
# .pp output
# ---------------------------------------------------------------------------

def heading_json(path: PathDef, src: OpModeSource) -> dict:
    h = path.heading

    def deg(pose_name: str) -> float:
        return src.poses[pose_name][2]

    base = {"reverse": h.get("reverse", False), "degrees": 0, "startDeg": 0, "endDeg": 0}
    if h["type"] == "linear":
        start, end = h["poses"]
        base.update({"type": "linear", "startDeg": deg(start), "endDeg": deg(end)})
    elif h["type"] == "constant":
        base.update({"type": "constant", "degrees": deg(h["poses"][0])})
    elif h["type"] == "tangential":
        return {"type": "tangential", "reverse": h.get("reverse", False)}
    elif h["type"] == "piecewise":
        segments, start_t = [], 0.0
        for end_t, interp in h["segments"]:
            segment = {
                "startProgress": start_t,
                "endProgress": end_t,
                "interpolationType": interp["interpolationType"],
                "reversed": False,
                "continueFromPrevious": False,
            }
            # Tangential segments have no "parameters" in visualizer-saved files.
            if interp["interpolationType"] == "linear":
                segment["parameters"] = {"startDeg": deg(interp["poses"][0]), "endDeg": deg(interp["poses"][1])}
            elif interp["interpolationType"] == "constant":
                segment["parameters"] = {"degrees": deg(interp["poses"][0])}
            segments.append(segment)
            start_t = end_t
        first = segments[0].get("parameters", {})
        last = segments[-1].get("parameters", {})
        base.update({
            "type": "piecewise",
            "startDeg": first.get("startDeg", first.get("degrees", 0)),
            "endDeg": last.get("endDeg", last.get("degrees", 0)),
            "piecewiseHeading": {"segments": segments},
        })
        return base
    base["piecewiseHeading"] = {"segments": [{
        "startProgress": 0, "endProgress": 1, "interpolationType": "linear", "reversed": False,
        "parameters": {"startDeg": 0, "endDeg": 0},
    }]}
    return base


def build_pp(src: OpModeSource, variant: Variant, robot: dict, shot_seconds: float) -> dict:
    lines = []
    pending_wait_before = 0.0
    pending_wait_before_names: List[str] = []

    def add_wait(seconds: float, name: str):
        nonlocal pending_wait_before
        if lines:
            lines[-1]["waitAfterMs"] += int(round(seconds * 1000))
            existing = lines[-1]["waitAfterName"]
            lines[-1]["waitAfterName"] = f"{existing} + {name}" if existing else name
        else:
            pending_wait_before += seconds
            pending_wait_before_names.append(name)

    for event in variant.events:
        if event.kind == "wait":
            add_wait(event.value, event.label)
        elif event.kind == "shoot":
            add_wait(shot_seconds, "shoot")
        elif event.kind == "path":
            if event.value not in src.chains:
                print(f"  warning: chain {event.value} not found in buildPaths()", file=sys.stderr)
                continue
            for path_name in src.chains[event.value]:
                path = src.paths[path_name]
                end_x, end_y, _ = src.poses[path.points[-1]]
                lines.append({
                    "kind": "atomic",
                    "id": f"line-{uuid.uuid4().hex[:8]}-{uuid.uuid4().hex[:6]}",
                    "endPoint": {"x": end_x, "y": end_y},
                    "controlPoints": [{"x": src.poses[p][0], "y": src.poses[p][1]} for p in path.points[1:-1]],
                    "heading": heading_json(path, src),
                    "color": COLORS[len(lines) % len(COLORS)],
                    "locked": False,
                    "waitBeforeMs": int(round(pending_wait_before * 1000)),
                    "waitAfterMs": 0,
                    "waitBeforeName": " + ".join(pending_wait_before_names),
                    "waitAfterName": "",
                    "name": path_name,
                })
                pending_wait_before = 0.0
                pending_wait_before_names = []

    if not lines:
        raise ValueError("No paths found for this variant")

    first_path = src.paths[lines[0]["name"]]
    sx, sy, sdeg = src.poses[first_path.points[0]]
    return {
        "version": "2.0.0",
        "timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z"),
        "startPoint": {"x": sx, "y": sy, "locked": False, "headingDeg": sdeg},
        "lines": lines,
        "shapes": FIELD_SHAPES,
        "sequence": [{"kind": "path", "lineId": line["id"]} for line in lines],
        "fieldPoints": [],
        "robot": robot,
        "activePaths": [],
    }


def robot_settings() -> dict:
    robot = {
        "forwardVelocity": 75,
        "strafeVelocity": 65,
        "aVelocity": 3.141592653589793,
        "kFriction": 0.1,
        "rLength": 16,
        "rWidth": 16,
        "safetyMargin": 1,
        "maxAcceleration": 30,
        "maxDeceleration": 30,
    }
    constants_path = PEDRO_DIR / "Constants.java"
    if constants_path.exists():
        text = strip_comments(constants_path.read_text(encoding="utf-8"))
        m = re.search(r"\.xVelocity\(\s*([-\d.]+)\s*\)", text)
        if m:
            robot["forwardVelocity"] = float(m.group(1))
        m = re.search(r"\.yVelocity\(\s*([-\d.]+)\s*\)", text)
        if m:
            robot["strafeVelocity"] = float(m.group(1))
    return robot


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--java", nargs="*", help="OpMode .java files (default: the 2026 autonomous OpModes)")
    parser.add_argument("--outdir", default=str(REPO_ROOT / "pedro_paths"), help="Output folder for .pp files")
    args = parser.parse_args()

    java_files = [pathlib.Path(p) for p in args.java] if args.java else [PEDRO_DIR / f for f in DEFAULT_OPMODES]

    shared_constants = {}
    shot_seconds = 1.2
    if SHOT_CONTROLLER.exists():
        shared_constants = parse_constants(strip_comments(SHOT_CONTROLLER.read_text(encoding="utf-8")))
        shot_seconds = shared_constants.get("FEED_SECONDS", shot_seconds)

    outdir = pathlib.Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    robot = robot_settings()

    for java_path in java_files:
        src = load_opmode(java_path, shared_constants)
        if "autonomousSequence" not in src.commands:
            print(f"{src.name}: no autonomousSequence() found, skipping", file=sys.stderr)
            continue
        print(f"{src.name}:")
        variants = expand(src.commands["autonomousSequence"][1], src)
        for variant in variants:
            suffix = "_".join(variant.labels) if variant.labels else "Default"
            pp = build_pp(src, variant, robot, shot_seconds)
            out_path = outdir / f"{src.name}__{suffix}.pp"
            out_path.write_text(json.dumps(pp, indent=2), encoding="utf-8")
            names = " -> ".join(line["name"] for line in pp["lines"])
            print(f"  wrote {out_path.name}: {names}")


if __name__ == "__main__":
    main()
