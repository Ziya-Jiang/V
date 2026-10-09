#!/usr/bin/env python3
"""Render ConceptGraphs JSON files as a clean static graph image."""

from __future__ import annotations

import argparse
import hashlib
import html
import json
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any


TAG_COLORS = {
    "table": "#6366F1",
    "bowl": "#0D9488",
    "cup": "#3B82F6",
    "plate": "#E89B0C",
    "bread": "#C26D3A",
}
FALLBACK_COLORS = ("#8B5CF6", "#EC4899", "#14B8A6", "#F97316", "#64748B")
SUPPORTED_FORMATS = {"png", "svg", "pdf"}


@dataclass(frozen=True)
class SceneObject:
    object_id: int
    semantic: str
    center: tuple[float, float, float]


@dataclass(frozen=True)
class Relation:
    source_id: int
    target_id: int
    relationship: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Draw semantic objects, coordinates, and spatial relations as one image.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("scene_dir", type=Path, help="Experiment directory.")
    parser.add_argument("--objects", type=Path, help="Object JSON; auto-detected by default.")
    parser.add_argument("--edges", type=Path, help="Edge JSON; auto-detected by default.")
    parser.add_argument("-o", "--output", type=Path, help="Output .png, .svg, or .pdf path.")
    parser.add_argument("--title", default="Spatial Scene Graph", help="Image title.")
    parser.add_argument("--dpi", type=int, default=220, help="PNG output resolution.")
    return parser.parse_args()


def load_json(path: Path) -> Any:
    try:
        with path.open("r", encoding="utf-8") as file:
            return json.load(file)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in {path}: {exc}") from exc


def records(data: Any, path: Path) -> list[dict[str, Any]]:
    values = list(data.values()) if isinstance(data, dict) else data
    if not isinstance(values, list) or not all(isinstance(item, dict) for item in values):
        raise ValueError(f"{path} must contain an object or array of records.")
    return values


def vector3(value: Any, field: str, path: Path) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{field} in {path} must contain three numbers.")
    try:
        return float(value[0]), float(value[1]), float(value[2])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} in {path} must contain numbers.") from exc


def load_objects(path: Path) -> list[SceneObject]:
    objects = []
    for index, item in enumerate(records(load_json(path), path)):
        try:
            objects.append(
                SceneObject(
                    object_id=int(item["id"]),
                    semantic=str(item["object_tag"]).strip() or "object",
                    center=vector3(item["bbox_center"], "bbox_center", path),
                )
            )
        except KeyError as exc:
            raise ValueError(f"Object record {index} in {path} is missing {exc}.") from exc
    if not objects:
        raise ValueError(f"No objects found in {path}.")
    if len({obj.object_id for obj in objects}) != len(objects):
        raise ValueError(f"Object IDs must be unique in {path}.")
    return sorted(objects, key=lambda obj: obj.object_id)


def load_relations(path: Path, valid_ids: set[int]) -> list[Relation]:
    relations = []
    for index, item in enumerate(records(load_json(path), path)):
        try:
            source_id = int(item["object_1_id"])
            target_id = int(item["object_2_id"])
        except KeyError as exc:
            raise ValueError(f"Edge record {index} in {path} is missing {exc}.") from exc
        if source_id not in valid_ids or target_id not in valid_ids:
            continue
        relations.append(
            Relation(
                source_id=source_id,
                target_id=target_id,
                relationship=str(item.get("relationship", "related to")).strip() or "related to",
            )
        )
    return relations


def discover(scene_dir: Path, explicit: Path | None, pattern: str) -> Path:
    if explicit:
        path = explicit.expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        return path
    matches = sorted(scene_dir.glob(pattern))
    if len(matches) != 1:
        names = ", ".join(path.name for path in matches) or "none"
        raise ValueError(f"Expected one {pattern} in {scene_dir}; found {names}.")
    return matches[0]


def semantic_color(semantic: str) -> str:
    known = TAG_COLORS.get(semantic.casefold())
    if known:
        return known
    digest = hashlib.sha256(semantic.casefold().encode("utf-8")).digest()
    return FALLBACK_COLORS[int.from_bytes(digest[:2], "big") % len(FALLBACK_COLORS)]


def escape(value: object) -> str:
    return html.escape(str(value), quote=True)


def node_label(obj: SceneObject) -> str:
    color = semantic_color(obj.semantic)
    x, y, z = obj.center
    coordinate = f"x {x:+.2f}   y {y:+.2f}   z {z:+.2f} m"
    return f"""<
      <TABLE BORDER="1" COLOR="#D9DEE8" CELLBORDER="0" CELLSPACING="0"
             CELLPADDING="0" BGCOLOR="#FFFFFF" STYLE="ROUNDED">
        <TR>
          <TD ROWSPAN="2" WIDTH="7" BGCOLOR="{color}"></TD>
          <TD ALIGN="LEFT" CELLPADDING="10">
            <FONT FACE="Arial" COLOR="{color}" POINT-SIZE="12"><B>{escape(obj.semantic.upper())}</B></FONT>
          </TD>
          <TD ALIGN="RIGHT" CELLPADDING="10">
            <FONT FACE="Arial" COLOR="#98A2B3" POINT-SIZE="9">#{obj.object_id}</FONT>
          </TD>
        </TR>
        <TR>
          <TD COLSPAN="2" ALIGN="LEFT" CELLPADDING="10">
            <FONT FACE="Courier New" COLOR="#475467" POINT-SIZE="9">{escape(coordinate)}</FONT>
          </TD>
        </TR>
      </TABLE>
    >"""


def edge_label(relationship: str) -> str:
    return f"""<
      <TABLE BORDER="0" CELLBORDER="0" CELLSPACING="0" CELLPADDING="4" BGCOLOR="#FFFFFF">
        <TR><TD><FONT FACE="Arial" COLOR="#475467" POINT-SIZE="9">{escape(relationship)}</FONT></TD></TR>
      </TABLE>
    >"""


def build_dot(
    objects: list[SceneObject],
    relations: list[Relation],
    title: str,
    dpi: int,
) -> str:
    lines = [
        "digraph scene_graph {",
        "  graph [",
        '    bgcolor="#F7F8FC",',
        '    pad="0.42",',
        '    margin="0",',
        '    rankdir="TB",',
        '    nodesep="0.50",',
        '    ranksep="0.78",',
        '    splines="spline",',
        '    outputorder="edgesfirst",',
        f'    dpi="{dpi}",',
        '    fontname="Arial",',
        '    labelloc="t",',
        '    labeljust="l",',
        "    label=<",
        '      <TABLE BORDER="0" CELLBORDER="0" CELLSPACING="0" CELLPADDING="2">',
        f'        <TR><TD ALIGN="LEFT"><FONT FACE="Arial" COLOR="#101828" POINT-SIZE="22"><B>{escape(title)}</B></FONT></TD></TR>',
        f'        <TR><TD ALIGN="LEFT"><FONT FACE="Arial" COLOR="#667085" POINT-SIZE="9">{len(objects)} objects  ·  {len(relations)} spatial relationships</FONT></TD></TR>',
        "      </TABLE>",
        "    >",
        "  ];",
        '  node [shape="plain", margin="0", fontname="Arial"];',
        '  edge [color="#667085", penwidth="1.35", arrowsize="0.72", arrowhead="vee", fontname="Arial"];',
        "",
    ]

    for obj in objects:
        tooltip = escape(
            f"{obj.semantic} #{obj.object_id}: "
            f"({obj.center[0]:.2f}, {obj.center[1]:.2f}, {obj.center[2]:.2f}) m"
        )
        lines.append(f'  n_{obj.object_id} [label={node_label(obj)}, tooltip="{tooltip}"];')

    lines.append("")
    for relation in relations:
        lines.append(
            f"  n_{relation.source_id} -> n_{relation.target_id} "
            f"[label={edge_label(relation.relationship)}, "
            f'tooltip="{escape(relation.relationship)}"];'
        )

    lines.append("}")
    return "\n".join(lines)


def render(dot_source: str, output: Path, output_format: str) -> None:
    dot_executable = shutil.which("dot")
    if dot_executable is None:
        raise RuntimeError("Graphviz 'dot' was not found. Install graphviz to render the image.")
    output.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run(
        [dot_executable, f"-T{output_format}", "-o", str(output)],
        input=dot_source,
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(f"Graphviz failed:\n{result.stderr.strip()}")


def main() -> None:
    args = parse_args()
    scene_dir = args.scene_dir.expanduser().resolve()
    if not scene_dir.is_dir():
        raise NotADirectoryError(scene_dir)

    object_path = discover(scene_dir, args.objects, "obj_json_*.json")
    edge_path = discover(scene_dir, args.edges, "edge_json_*.json")
    all_objects = load_objects(object_path)
    relations = load_relations(edge_path, {obj.object_id for obj in all_objects})
    connected_ids = {
        endpoint
        for relation in relations
        for endpoint in (relation.source_id, relation.target_id)
    }
    objects = [obj for obj in all_objects if obj.object_id in connected_ids]
    if not objects:
        raise ValueError("No connected objects found; the scene graph has no valid relations.")
    output = (
        args.output.expanduser().resolve()
        if args.output
        else scene_dir / "scene_graph.png"
    )
    output_format = output.suffix.casefold().lstrip(".")
    if output_format not in SUPPORTED_FORMATS:
        formats = ", ".join(f".{name}" for name in sorted(SUPPORTED_FORMATS))
        raise ValueError(f"Output extension must be one of: {formats}.")

    dot_source = build_dot(objects, relations, args.title, args.dpi)
    render(dot_source, output, output_format)
    print(f"Rendered {len(objects)} objects and {len(relations)} relations: {output}")


if __name__ == "__main__":
    main()
