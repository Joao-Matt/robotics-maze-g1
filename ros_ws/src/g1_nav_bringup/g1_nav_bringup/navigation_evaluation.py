"""Pure helpers for Phase 6 goal selection and command evaluation."""

from __future__ import annotations

from collections import deque
import math

import numpy as np


def largest_connected_component(mask: np.ndarray) -> np.ndarray:
    """Return only the largest 4-connected true component."""
    mask=np.asarray(mask,dtype=bool); visited=np.zeros_like(mask); best=[]
    height,width=mask.shape
    for row,col in zip(*np.nonzero(mask)):
        if visited[row,col]:continue
        queue=deque([(int(row),int(col))]); visited[row,col]=True; component=[]
        while queue:
            y,x=queue.popleft(); component.append((y,x))
            for ny,nx in ((y+1,x),(y-1,x),(y,x+1),(y,x-1)):
                if 0<=ny<height and 0<=nx<width and mask[ny,nx] and not visited[ny,nx]:visited[ny,nx]=True; queue.append((ny,nx))
        if len(component)>len(best):best=component
    result=np.zeros_like(mask)
    if best:
        ys,xs=zip(*best); result[np.asarray(ys),np.asarray(xs)]=True
    return result


def farthest_reachable_free(data, width: int, height: int, start: tuple[int, int], max_distance: int | None = None) -> tuple[int, int] | None:
    """Return the most distant map-connected known-free cell."""
    if width <= 0 or height <= 0 or len(data) != width * height:
        return None
    sx, sy = start
    if not (0 <= sx < width and 0 <= sy < height):
        return None
    free = lambda x, y: int(data[y * width + x]) == 0
    if not free(sx, sy):
        candidates = [(x, y) for y in range(height) for x in range(width) if free(x, y)]
        if not candidates:
            return None
        sx, sy = min(candidates, key=lambda cell: (cell[0] - sx) ** 2 + (cell[1] - sy) ** 2)
    queue, distances = deque([(sx, sy)]), {(sx, sy): 0}
    while queue:
        x, y = queue.popleft()
        if max_distance is not None and distances[(x, y)] >= max_distance:
            continue
        for nxt in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            nx, ny = nxt
            if 0 <= nx < width and 0 <= ny < height and nxt not in distances and free(nx, ny):
                distances[nxt] = distances[(x, y)] + 1
                queue.append(nxt)
    return max(distances, key=lambda cell: (distances[cell], -abs(cell[0] - sx), -abs(cell[1] - sy))) if distances else None


def path_is_known_free(data, width: int, height: int, origin: tuple[float, float], resolution: float, points, max_cost: int = 0) -> bool:
    if resolution <= 0.0 or not points:
        return False
    for x, y in points:
        col = int(math.floor((x - origin[0]) / resolution))
        row = int(math.floor((y - origin[1]) / resolution))
        value = int(data[row * width + col]) if 0 <= col < width and 0 <= row < height else -1
        if value < 0 or value > max_cost:
            return False
    return True


def stop_status(nav_result: str | None, motion_status: str, actual_goal_error_m: float | None, tolerance_m: float = 0.5) -> str:
    terminal_motion = {"TIMEOUT", "STUCK", "FALL_DETECTED", "COLLISION_ABORT", "NAV2_ABORTED", "COMMAND_TIMEOUT"}
    if motion_status in terminal_motion:
        return motion_status
    if nav_result == "SUCCEEDED":
        return "GOAL_REACHED" if actual_goal_error_m is not None and actual_goal_error_m <= tolerance_m else "NAV2_GOAL_WITH_PHYSICAL_ERROR"
    if nav_result in {"ABORTED", "CANCELED"}:
        return f"NAV2_{nav_result}"
    return "RUNNING"


def frontier_clusters(data, width: int, height: int, minimum_size: int = 5) -> list[list[tuple[int, int]]]:
    """Cluster known-free cells that border unknown occupancy."""
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    frontier = np.zeros_like(grid, dtype=bool)
    for row in range(1, height - 1):
        for col in range(1, width - 1):
            if grid[row, col] == 0 and any(grid[row + dr, col + dc] < 0 for dr, dc in ((1,0),(-1,0),(0,1),(0,-1))):
                frontier[row, col] = True
    seen: set[tuple[int, int]] = set(); clusters=[]
    for row, col in zip(*np.nonzero(frontier)):
        start=(int(col),int(row))
        if start in seen: continue
        queue=deque([start]); seen.add(start); cluster=[]
        while queue:
            x,y=queue.popleft(); cluster.append((x,y))
            for nx,ny in ((x+1,y),(x-1,y),(x,y+1),(x,y-1),(x+1,y+1),(x-1,y-1),(x+1,y-1),(x-1,y+1)):
                if 0<=nx<width and 0<=ny<height and frontier[ny,nx] and (nx,ny) not in seen:
                    seen.add((nx,ny)); queue.append((nx,ny))
        if len(cluster)>=minimum_size: clusters.append(cluster)
    return clusters


def frontier_goal(cluster, data, width: int, height: int, setback_cells: int = 5) -> tuple[int, int] | None:
    """Choose a free goal near a frontier but biased into observed space."""
    grid=np.asarray(data,dtype=np.int16).reshape(height,width)
    cx=sum(p[0] for p in cluster)/len(cluster); cy=sum(p[1] for p in cluster)/len(cluster)
    candidates=[]
    radius=max(1,setback_cells)
    for y in range(max(0,int(cy)-radius),min(height,int(cy)+radius+1)):
        for x in range(max(0,int(cx)-radius),min(width,int(cx)+radius+1)):
            if grid[y,x]!=0: continue
            unknown=sum(grid[ny,nx]<0 for ny in range(max(0,y-2),min(height,y+3)) for nx in range(max(0,x-2),min(width,x+3)))
            candidates.append((unknown,(x-cx)**2+(y-cy)**2,x,y))
    if not candidates:return None
    _,_,x,y=min(candidates)
    return int(x),int(y)
