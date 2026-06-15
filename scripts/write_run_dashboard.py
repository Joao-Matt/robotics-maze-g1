"""Write a side-by-side visible dashboard for run artifacts."""

from __future__ import annotations

from pathlib import Path
import argparse
import html
import os
from datetime import datetime, timezone


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Write an HTML dashboard for visual run artifacts.")
    parser.add_argument("--html", type=Path, required=True, help="Dashboard HTML output path.")
    parser.add_argument("--seed", type=int, required=True, help="Run seed.")
    parser.add_argument("--mode", default="maze", help="World mode.")
    parser.add_argument("--topdown-svg", type=Path, required=True, help="Top-down maze SVG path.")
    parser.add_argument("--render-image", type=Path, required=True, help="MuJoCo render image path.")
    parser.add_argument("--world-xml", type=Path, required=True, help="Generated MuJoCo XML path.")
    parser.add_argument("--world-summary", type=Path, required=True, help="Generated world JSON summary path.")
    parser.add_argument("--run-summary", type=Path, required=True, help="Run JSON summary path.")
    return parser.parse_args()


def rel(path: Path, base: Path) -> str:
    try:
        return os.path.relpath(path, start=base)
    except ValueError:
        return str(path)


def panel_for_image(title: str, image_path: Path, base: Path, *, iframe: bool = False) -> str:
    src = html.escape(rel(image_path, base))
    title_text = html.escape(title)
    if image_path.exists():
        if iframe:
            media = f'<iframe src="{src}" title="{title_text}"></iframe>'
        else:
            media = f'<img src="{src}" alt="{title_text}">'
    else:
        media = f'<div class="missing">Missing artifact: {html.escape(str(image_path))}</div>'
    return f"""<section class="panel">
  <h2>{title_text}</h2>
  <div class="media">{media}</div>
</section>"""


def main() -> int:
    args = parse_args()
    args.html.parent.mkdir(parents=True, exist_ok=True)
    base = args.html.parent
    generated = datetime.now(timezone.utc).isoformat(timespec="seconds")
    links = [
        ("Generated XML", args.world_xml),
        ("World summary", args.world_summary),
        ("Run summary", args.run_summary),
        ("Top-down SVG", args.topdown_svg),
        ("MuJoCo render", args.render_image),
    ]
    link_items = "\n".join(
        f'<a href="{html.escape(rel(path, base))}">{html.escape(label)}</a>'
        for label, path in links
    )
    document = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Robotics Maze G1 Run Seed {args.seed}</title>
  <style>
    :root {{ color-scheme: light; }}
    body {{ margin: 0; font-family: system-ui, sans-serif; color: #111827; background: #f3f4f6; }}
    header {{ padding: 14px 18px; background: #111827; color: #f9fafb; }}
    h1 {{ margin: 0; font-size: 18px; font-weight: 700; }}
    .meta {{ margin-top: 4px; color: #cbd5e1; font-size: 13px; }}
    main {{ display: grid; grid-template-columns: 1fr 1fr; gap: 12px; padding: 12px; min-height: calc(100vh - 108px); }}
    .panel {{ background: #ffffff; border: 1px solid #d1d5db; border-radius: 6px; min-width: 0; display: flex; flex-direction: column; }}
    h2 {{ margin: 0; padding: 10px 12px; font-size: 15px; border-bottom: 1px solid #e5e7eb; }}
    .media {{ flex: 1; min-height: 420px; display: flex; align-items: center; justify-content: center; overflow: auto; background: #e5e7eb; }}
    iframe {{ width: 100%; height: 100%; min-height: 680px; border: 0; background: #ffffff; }}
    img {{ max-width: 100%; max-height: 78vh; object-fit: contain; image-rendering: auto; }}
    .missing {{ padding: 18px; color: #991b1b; font-family: ui-monospace, SFMono-Regular, Consolas, monospace; }}
    footer {{ display: flex; flex-wrap: wrap; gap: 10px; padding: 10px 12px 14px; background: #f9fafb; border-top: 1px solid #d1d5db; }}
    footer a {{ color: #1d4ed8; font-size: 13px; }}
    @media (max-width: 900px) {{ main {{ grid-template-columns: 1fr; }} }}
  </style>
</head>
<body>
  <header>
    <h1>Robotics Maze G1 Run</h1>
    <div class="meta">seed={args.seed} · mode={html.escape(args.mode)} · generated={html.escape(generated)}</div>
  </header>
  <main>
    {panel_for_image("2D Maze Representation", args.topdown_svg, base, iframe=True)}
    {panel_for_image("Generated MuJoCo Maze World", args.render_image, base)}
  </main>
  <footer>{link_items}</footer>
</body>
</html>
"""
    args.html.write_text(document, encoding="utf-8")
    print(f"run_dashboard_artifact: {args.html}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
