"""Editor checks; intentionally not a replacement for the Autoware validator."""
from dataclasses import dataclass
import math
import numpy as np

from . import geometry as g
from .operations import bound_nodes, lane_center


@dataclass
class Issue:
    level: str
    ref: tuple[str, int]
    message: str


def validate(d):
    issues = []
    def add(level, kind, oid, text):
        issues.append(Issue(level, (kind, oid), text))
    for n in d.nodes.values():
        if not all(math.isfinite(v) for v in (n.x, n.y, n.z)):
            add("ERROR", "node", n.id, "坐标不是有限数")
    for w in d.ways.values():
        missing = [i for i in w.nodes if i not in d.nodes]
        if missing:
            add("ERROR", "way", w.id, f"缺失节点 {missing[:5]}")
            continue
        polygon = w.tags.get("area") == "yes"
        if len(set(w.nodes)) < (3 if polygon else 2):
            add("ERROR", "way", w.id, "有效节点数不足")
            continue
        p = d.points(w.id)
        if np.any(np.linalg.norm(np.diff(p, axis=0), axis=1) < 1e-7):
            add("WARNING", "way", w.id, "相邻节点重合")
        if len(p) <= 1500 and g.self_intersects(p, polygon):
            add("ERROR", "way", w.id, "折线或多边形存在自交")
        if polygon and abs(g.signed_area(p)) < .001:
            add("ERROR", "way", w.id, "多边形面积接近零")
    for r in d.relations.values():
        for m in r.members:
            if m.type not in ("node", "way", "relation") or d.get((m.type, m.ref)) is None:
                add("ERROR", "relation", r.id, f"缺失引用 {m.type}/{m.ref} role={m.role}")
        if r.tags.get("type") == "lanelet":
            try:
                for role in ("left", "right"):
                    if sum(m.role == role and m.type == "way" for m in r.members) != 1:
                        raise ValueError("必须各有一个 left/right 边界")
                a, b = bound_nodes(d, r.id)
                left = np.array([d.nodes[i].xyz for i in a]); right = np.array([d.nodes[i].xyz for i in b])
                outline = np.vstack([left, right[::-1]])
                if len(outline) < 1500 and g.self_intersects(outline, True):
                    add("ERROR", "relation", r.id, "左右边界交叉或方向错误")
                if g.signed_area(outline) >= 0:
                    add("ERROR", "relation", r.id, "车道边界左右位置或方向不正确")
                c = g.centerline(left, right, spacing=2)
                width = np.linalg.norm(g.sample(left, len(c)) - g.sample(right, len(c)), axis=1)
                if np.min(width) < .3:
                    add("ERROR", "relation", r.id, "局部车道宽度小于 0.3 米")
                for m in r.members:
                    if m.role == "centerline" and m.ref in d.ways:
                        if any(not g.contains(outline, p) for p in d.points(m.ref)[1:-1]):
                            add("WARNING", "relation", r.id, "中心线有点位于车道外")
                speed = float(r.tags.get("speed_limit", "nan"))
                if not math.isfinite(speed) or not 0 < speed <= 160:
                    add("WARNING", "relation", r.id, "speed_limit 缺失或不在 (0,160] km/h")
                if r.tags.get("subtype") == "road":
                    for k, value in (("one_way", "yes"), ("location", "urban")):
                        if r.tags.get(k) != value:
                            add("WARNING", "relation", r.id, f"请核对 {k}（通常为 {value}）")
            except (ValueError, KeyError, IndexError) as exc:
                add("ERROR", "relation", r.id, str(exc))
        elif r.tags.get("type") == "regulatory_element":
            subtype = r.tags.get("subtype", "")
            if not subtype:
                add("ERROR", "relation", r.id, "规则缺少 subtype")
            roles = {m.role for m in r.members}
            required = {"traffic_light": {"refers", "ref_line"},
                        "traffic_sign": {"refers", "ref_line"},
                        "detection_area": {"refers", "ref_line"},
                        "right_of_way": {"right_of_way", "yield"}}.get(subtype, set())
            if not required.issubset(roles):
                add("ERROR", "relation", r.id, f"规则缺少角色 {sorted(required-roles)}")
            if not d.parents(("relation", r.id)):
                add("WARNING", "relation", r.id, "规则未关联到任何对象")
    for s in d.stops.values():
        if s.lanelet_id not in d.relations:
            add("WARNING", "stop", s.id, "停靠点未关联到有效车道")
    # Include disconnected roads as a warning only: real maps have valid endpoints.
    lane_ends = {}
    for r in d.lanelets():
        try:
            a, b = bound_nodes(d, r.id)
            lane_ends[r.id] = ((a[0], b[0]), (a[-1], b[-1]))
        except (ValueError, KeyError):
            pass
    for lid, (start, end) in lane_ends.items():
        if len(lane_ends) > 1 and not any(i != lid and (s == end or e == start)
                                        for i, (s, e) in lane_ends.items()):
            add("WARNING", "relation", lid, "车道没有共享端点的前驱/后继（可能是独立道路）")
    return issues


def compare(base, other):
    result = []
    for kind in ("node", "way", "relation"):
        a, b = base.table(kind), other.table(kind)
        for oid in sorted(set(a) | set(b)):
            status = "新增" if oid not in a else "删除" if oid not in b else "修改" if a[oid] != b[oid] else None
            if status:
                result.append((status, (kind, oid)))
    return result
