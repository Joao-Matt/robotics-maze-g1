"""Shared artifact helpers for production oracle runs."""

from __future__ import annotations

from pathlib import Path
import json
import xml.etree.ElementTree as ET

from sim.world_builder import WorldBuildResult, cell_to_world_xy


def apply_cell_size_override(config: dict, cell_size_m: float | None) -> float:
    if cell_size_m is None:
        return float(config["maze"].get("cell_width_m", config["maze"]["cell_size_m"]))
    if cell_size_m < 1.0 or cell_size_m > 4.0:
        raise ValueError(f"cell size must be between 1.0 and 4.0 meters, got {cell_size_m:.3g}.")
    config["maze"]["cell_size_m"] = float(cell_size_m)
    config["maze"]["cell_width_m"] = float(cell_size_m)
    config["maze"]["cell_length_m"] = float(cell_size_m)
    return float(cell_size_m)


def append_path_markers(world_xml: Path, maze, cells: list[tuple[int, int]]) -> None:
    tree = ET.parse(world_xml)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        worldbody = ET.SubElement(root, "worldbody")
    stride = max(1, len(cells) // 18)
    for index, cell in enumerate(cells):
        if index % stride != 0 and index != len(cells) - 1:
            continue
        x, y = cell_to_world_xy(maze, cell)
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": f"oracle_path_marker_{index}",
                "type": "sphere",
                "pos": f"{x:.6g} {y:.6g} 0.08",
                "size": "0.08",
                "rgba": "0.10 0.35 1.0 0.85",
                "contype": "0",
                "conaffinity": "0",
            },
        )
    tree.write(world_xml, encoding="utf-8", xml_declaration=True)


def world_with_xml_path(world: WorldBuildResult, path: Path) -> WorldBuildResult:
    return WorldBuildResult(**{**world.to_dict(), "model_xml_path": str(path)})


def world_with_topdown_path(world: WorldBuildResult, path: Path) -> WorldBuildResult:
    return WorldBuildResult(**{**world.to_dict(), "topdown_svg_path": str(path)})


def new_contact_stats() -> dict[str, object]:
    return {
        "steps_with_contacts": 0,
        "steps_with_wall_contacts": 0,
        "max_contact_count": 0,
        "max_wall_contact_count": 0,
        "first_wall_contact_time_s": None,
        "first_wall_contact_pairs": [],
        "last_wall_contact_time_s": None,
        "last_wall_contact_pairs": [],
    }


def contact_summary(mujoco, model, data) -> dict[str, object]:
    contact_count = int(getattr(data, "ncon", 0))
    wall_pairs: list[str] = []
    for index in range(contact_count):
        contact = data.contact[index]
        geom1 = _geom_name(mujoco, model, int(contact.geom1))
        geom2 = _geom_name(mujoco, model, int(contact.geom2))
        if _is_robot_wall_contact(geom1, geom2):
            wall_pairs.append(f"{geom1}<->{geom2}")
    return {
        "contact_count": contact_count,
        "wall_contact_count": len(wall_pairs),
        "wall_contact_pairs": wall_pairs[:6],
    }


def update_contact_stats(contact_stats: dict[str, object], sim_time: float, contacts: dict[str, object]) -> None:
    contact_count = int(contacts["contact_count"])
    wall_contact_count = int(contacts["wall_contact_count"])
    if contact_count > 0:
        contact_stats["steps_with_contacts"] = int(contact_stats["steps_with_contacts"]) + 1
    if wall_contact_count > 0:
        contact_stats["steps_with_wall_contacts"] = int(contact_stats["steps_with_wall_contacts"]) + 1
        contact_stats["last_wall_contact_time_s"] = round(float(sim_time), 6)
        contact_stats["last_wall_contact_pairs"] = list(contacts["wall_contact_pairs"])
        if contact_stats["first_wall_contact_time_s"] is None:
            contact_stats["first_wall_contact_time_s"] = round(float(sim_time), 6)
            contact_stats["first_wall_contact_pairs"] = list(contacts["wall_contact_pairs"])
    contact_stats["max_contact_count"] = max(int(contact_stats["max_contact_count"]), contact_count)
    contact_stats["max_wall_contact_count"] = max(int(contact_stats["max_wall_contact_count"]), wall_contact_count)


def final_contact_summary(contact_stats: dict[str, object]) -> dict[str, object]:
    return dict(contact_stats)


def print_artifacts(summary: dict[str, object], artifacts: dict[str, Path]) -> None:
    print(json.dumps(summary, indent=2, sort_keys=True))
    for key, path in artifacts.items():
        print(f"{key}_artifact: {path}")


def _geom_name(mujoco, model, geom_id: int) -> str:
    if geom_id < 0:
        return ""
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
    if name:
        return name
    body_id = int(model.geom_bodyid[geom_id])
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
    return f"geom_{geom_id}:{body_name or f'body_{body_id}'}"


def _is_robot_wall_contact(geom1: str, geom2: str) -> bool:
    return (geom1.startswith("maze_wall_") and _is_robot_geom(geom2)) or (
        geom2.startswith("maze_wall_") and _is_robot_geom(geom1)
    )


def _is_robot_geom(name: str) -> bool:
    if not name:
        return False
    return not (
        name.startswith("maze_")
        or name.startswith("oracle_path_marker_")
        or name in {"floor", "maze_floor", "world", "groundplane"}
    )
