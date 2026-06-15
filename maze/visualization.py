"""Maze visualization helpers."""

from __future__ import annotations

from pathlib import Path
from xml.sax.saxutils import escape

from maze.grid import FREE, Maze, Cell


def maze_to_ascii(maze: Maze, path: list[Cell] | None = None) -> str:
    """Render a maze as ASCII text."""
    path_cells = set(path or [])
    lines: list[str] = []

    for row in range(maze.spec.height_cells):
        chars: list[str] = []
        for col in range(maze.spec.width_cells):
            cell = (row, col)
            if cell == maze.spec.start_cell:
                chars.append("S")
            elif cell == maze.spec.goal_cell:
                chars.append("G")
            elif cell in path_cells:
                chars.append(".")
            elif maze.grid[cell] == FREE:
                chars.append(" ")
            else:
                chars.append("#")
        lines.append("".join(chars))

    return "\n".join(lines)


def save_ascii(maze: Maze, path: Path, route: list[Cell] | None = None) -> None:
    """Save an ASCII maze rendering to disk."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(maze_to_ascii(maze, route) + "\n", encoding="utf-8")


def save_pgm(maze: Maze, path: Path, route: list[Cell] | None = None) -> None:
    """Save a dependency-free PGM image for quick visual inspection."""
    path.parent.mkdir(parents=True, exist_ok=True)
    route_cells = set(route or [])

    pixels: list[str] = []
    for row in range(maze.spec.height_cells):
        for col in range(maze.spec.width_cells):
            cell = (row, col)
            if cell == maze.spec.start_cell:
                pixels.append("90")
            elif cell == maze.spec.goal_cell:
                pixels.append("150")
            elif cell in route_cells:
                pixels.append("210")
            elif maze.grid[cell] == FREE:
                pixels.append("255")
            else:
                pixels.append("0")

    header = f"P2\n{maze.spec.width_cells} {maze.spec.height_cells}\n255\n"
    path.write_text(header + "\n".join(pixels) + "\n", encoding="ascii")


def save_svg(maze: Maze, path: Path, route: list[Cell] | None = None, cell_px: int = 36) -> None:
    """Save a readable, dependency-free SVG maze rendering."""
    path.parent.mkdir(parents=True, exist_ok=True)
    route_cells = set(route or [])
    width = maze.spec.width_cells * cell_px
    height = maze.spec.height_cells * cell_px
    font_size = max(12, int(cell_px * 0.48))
    route_radius = max(3, int(cell_px * 0.16))

    elements = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
            f'viewBox="0 0 {width} {height}" role="img">'
        ),
        f"<title>{escape(f'Maze seed {maze.spec.seed}')}</title>",
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
    ]

    for row in range(maze.spec.height_cells):
        for col in range(maze.spec.width_cells):
            cell = (row, col)
            x = col * cell_px
            y = row * cell_px

            if maze.grid[cell] == FREE:
                fill = "#ffffff"
            else:
                fill = "#111827"
            if cell in route_cells:
                fill = "#dbeafe"
            if cell == maze.spec.start_cell:
                fill = "#bbf7d0"
            elif cell == maze.spec.goal_cell:
                fill = "#fecaca"

            elements.append(
                f'<rect x="{x}" y="{y}" width="{cell_px}" height="{cell_px}" '
                f'fill="{fill}" stroke="#cbd5e1" stroke-width="1"/>'
            )

    for row, col in route or []:
        if (row, col) in (maze.spec.start_cell, maze.spec.goal_cell):
            continue
        cx = col * cell_px + cell_px / 2
        cy = row * cell_px + cell_px / 2
        elements.append(f'<circle cx="{cx}" cy="{cy}" r="{route_radius}" fill="#2563eb"/>')

    for label, cell, fill in (
        ("S", maze.spec.start_cell, "#166534"),
        ("G", maze.spec.goal_cell, "#991b1b"),
    ):
        row, col = cell
        x = col * cell_px + cell_px / 2
        y = row * cell_px + cell_px / 2
        elements.append(
            f'<text x="{x}" y="{y}" text-anchor="middle" dominant-baseline="central" '
            f'font-family="monospace" font-size="{font_size}" font-weight="700" '
            f'fill="{fill}">{label}</text>'
        )

    elements.append("</svg>")
    path.write_text("\n".join(elements) + "\n", encoding="utf-8")
