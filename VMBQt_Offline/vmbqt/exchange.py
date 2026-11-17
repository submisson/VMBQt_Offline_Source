"""CSV/project exchange. CSV extras survive a round trip; no FMS API assumptions."""
from pathlib import Path
import csv
import io
import json
import math
import os

from .model import Document, StopPoint, TrajectoryPoint
from .storage import atomic_text


def read_csv(path, kind="trajectory"):
    result = []
    with open(path, encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if not {"x", "y", "z"}.issubset(reader.fieldnames or []):
            raise ValueError("CSV 必须有 x,y,z 列，可选 yaw（弧度）、velocity、id、lanelet_id、name")
        seen = set()
        for i, row in enumerate(reader, 1):
            numbers = {k: float(row.get(k, 0) or 0) for k in ("x", "y", "z", "yaw")}
            if not all(math.isfinite(v) for v in numbers.values()):
                raise ValueError(f"CSV 第 {i+1} 行包含无效坐标")
            oid = int(row.get("id", i) or i)
            if oid in seen:
                raise ValueError(f"CSV ID 重复：{oid}")
            seen.add(oid)
            common = dict(id=oid, **numbers, lanelet_id=int(row.get("lanelet_id", 0) or 0))
            if kind == "stop":
                obj = StopPoint(**common, name=row.get("name", f"stop_{oid}"))
            else:
                obj = TrajectoryPoint(**common, velocity=float(row.get("velocity", 0) or 0))
            obj.extra = {k: v for k, v in row.items() if k and k not in obj.__dataclass_fields__}
            result.append(obj)
    return result


def write_csv(path, objects, kind="trajectory"):
    objects = list(objects)
    base = ["id", "x", "y", "z", "yaw", "name" if kind == "stop" else "velocity", "lanelet_id"]
    fields = base + sorted({k for o in objects for k in o.extra if k not in base})
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
    for obj in objects:
        row = dict(obj.extra)
        row.update({k: getattr(obj, k) for k in base})
        writer.writerow(row)
    atomic_text(path, stream.getvalue())


def save_project(path, document, assets, settings):
    parent = Path(path).resolve().parent
    paths = []
    for p in assets:
        p = dict(p)
        p["source"] = os.path.relpath(p["source"], parent)
        paths.append(p)
    payload = {"application": "VMBQt", "version": 1, "document": document.state(),
               "assets": paths, "settings": settings}
    atomic_text(path, json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))


def load_project(path):
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if value.get("application") != "VMBQt" or value.get("version") != 1:
        raise ValueError("不是本程序支持的 .vmbqt 工程")
    d = Document(); d.restore(value["document"])
    assets = value.get("assets", [])
    for p in assets:
        p["source"] = str((Path(path).resolve().parent / p["source"]).resolve())
    return d, assets, value.get("settings", {})
