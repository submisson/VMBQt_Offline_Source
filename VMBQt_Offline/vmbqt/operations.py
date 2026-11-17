"""Transactional callers live in the UI; operations here also work headlessly."""
from __future__ import annotations

import copy
import heapq
import math
import numpy as np

from . import geometry as g
from .model import Document, Member, Relation

LINE_TAGS = {"type": "line_thin", "subtype": "solid", "lane_change": "no"}
LANE_TAGS = {"type": "lanelet", "subtype": "road", "location": "urban",
             "participant:vehicle": "yes", "one_way": "yes", "speed_limit": "30"}


def bound_nodes(d: Document, lane_id):
    left, right = d.bounds(lane_id)
    if left not in d.ways or right not in d.ways:
        raise ValueError("车道缺少有效的左右边界")
    a, b = list(d.ways[left].nodes), list(d.ways[right].nodes)
    if min(len(a), len(b)) < 2:
        raise ValueError("车道边界至少有两个点")
    pa, pb = d.points(left), d.points(right)
    # Imported OSM can store the right boundary opposite to the left boundary.
    if np.linalg.norm(pa[0] - pb[-1]) + np.linalg.norm(pa[-1] - pb[0]) < \
            np.linalg.norm(pa[0] - pb[0]) + np.linalg.norm(pa[-1] - pb[-1]):
        b.reverse()
    return a, b


def lane_center(d, lane_id, spacing=1.):
    r = d.relations[lane_id]
    c = next((m.ref for m in r.members if m.type == "way" and m.role == "centerline"), None)
    if c in d.ways and len(d.ways[c].nodes) >= 2:
        return d.points(c)
    a, b = bound_nodes(d, lane_id)
    return g.centerline([d.nodes[i].xyz for i in a], [d.nodes[i].xyz for i in b], spacing)


def create_lane(d, center, width=3.5, speed=30., subtype="road"):
    if not .2 <= width <= 50:
        raise ValueError("车道宽度应在 0.2–50 米之间")
    p = np.asarray(center, dtype=float)
    if g.length(p, True) < .1:
        raise ValueError("车道中心线过短")
    left = d.add_way(g.offset(p, width / 2), LINE_TAGS)
    right = d.add_way(g.offset(p, -width / 2), LINE_TAGS)
    tags = dict(LANE_TAGS, speed_limit=f"{speed:g}", subtype=subtype)
    if subtype == "crosswalk":
        tags.update({"participant:vehicle": "no", "participant:pedestrian": "yes", "one_way": "no"})
    return d.add_relation([("way", left, "left"), ("way", right, "right")], tags)


def lane_from_bounds(d, left, right, speed=30.):
    if left == right or min(len(d.ways[left].nodes), len(d.ways[right].nodes)) < 2:
        raise ValueError("请选择两条不同的有效边界；先左后右")
    return d.add_relation([("way", left, "left"), ("way", right, "right")],
                          dict(LANE_TAGS, speed_limit=f"{speed:g}"))


def add_adjacent(d, lane_id, side="left", width=3.5):
    if width <= .2:
        raise ValueError("宽度必须大于 0.2 米")
    a, b = bound_nodes(d, lane_id)
    left, right = d.bounds(lane_id)
    source = a if side == "left" else b
    new = d.add_way(g.offset([d.nodes[i].xyz for i in source],
                             width if side == "left" else -width), LINE_TAGS)
    tags = dict(d.relations[lane_id].tags)
    return d.add_relation([("way", new if side == "left" else right, "left"),
                           ("way", left if side == "left" else new, "right")], tags)


def generate_centerline(d, lane_id, spacing=1.):
    r = d.relations[lane_id]
    a, b = bound_nodes(d, lane_id)
    points = g.centerline([d.nodes[i].xyz for i in a], [d.nodes[i].xyz for i in b], spacing)
    old = [m for m in r.members if m.role == "centerline" and m.type == "way"]
    r.members = [m for m in r.members if m.role != "centerline"]
    wid = d.add_way(points, {"type": "virtual"})
    r.members.append(Member("way", wid, "centerline"))
    for m in old:
        if not d.parents(("way", m.ref)) and m.ref in d.ways:
            del d.ways[m.ref]
    return wid


def _exclusive_boundaries(d, lane_ids):
    selected = {("relation", i) for i in lane_ids}
    for lid in lane_ids:
        for wid in d.bounds(lid):
            if any(p not in selected for p in d.parents(("way", wid))):
                raise ValueError("此操作涉及与其他对象共享的边界，请先选择全部相关车道或改用点编辑")


def change_width(d, lane_id, width):
    _exclusive_boundaries(d, [lane_id])
    if width < .2:
        raise ValueError("车道宽度不能小于 0.2 米")
    a, b = bound_nodes(d, lane_id)
    c = g.centerline([d.nodes[i].xyz for i in a], [d.nodes[i].xyz for i in b])
    for ids, sign in ((a, 1), (b, -1)):
        new = g.offset(g.sample(c, len(ids)), sign * width / 2)
        for nid, p in zip(ids, new):
            d.nodes[nid].x, d.nodes[nid].y, d.nodes[nid].z = map(float, p)
    if any(m.role == "centerline" for m in d.relations[lane_id].members):
        generate_centerline(d, lane_id)


def reverse_lanes(d, lane_ids):
    _exclusive_boundaries(d, lane_ids)
    done = set()
    for lid in lane_ids:
        r = d.relations[lid]
        for m in r.members:
            if m.role in ("left", "right", "centerline") and m.type == "way":
                if m.ref not in done:
                    d.ways[m.ref].nodes.reverse(); done.add(m.ref)
                if m.role == "left":
                    m.role = "right"
                elif m.role == "right":
                    m.role = "left"


def merge_nodes(d, keep, remove):
    if keep == remove:
        return
    if keep not in d.nodes or remove not in d.nodes:
        raise ValueError("合并节点不存在")
    for w in d.ways.values():
        w.nodes = [keep if i == remove else i for i in w.nodes]
        w.nodes = [n for j, n in enumerate(w.nodes) if not j or n != w.nodes[j - 1]]
    for r in d.relations.values():
        for m in r.members:
            if m.type == "node" and m.ref == remove:
                m.ref = keep
    del d.nodes[remove]


def connect_lanes(d, first, second, tolerance=2.):
    a, b = bound_nodes(d, first); c, e = bound_nodes(d, second)
    pairs = [(a[-1], c[0]), (b[-1], e[0])]
    for keep, remove in pairs:
        if np.linalg.norm(d.nodes[keep].xyz - d.nodes[remove].xyz) > tolerance:
            raise ValueError(f"端点距离超过 {tolerance:g} 米，请绘制连接车道")
    for keep, remove in pairs:
        merge_nodes(d, keep, remove)


def join_lanes(d, first, second):
    _exclusive_boundaries(d, [first, second])
    if d.parents(("relation", second)):
        raise ValueError("后一条车道仍被其他关系引用，无法直接合并")
    a, b = bound_nodes(d, first); c, e = bound_nodes(d, second)
    if a[-1] != c[0] or b[-1] != e[0]:
        raise ValueError("先连接端点：相连车道必须共享左右端点 ID")
    la, ra = d.bounds(first); lb, rb = d.bounds(second)
    d.ways[la].nodes = a + c[1:]; d.ways[ra].nodes = b + e[1:]
    for member in d.relations[second].members:
        if member.role == "regulatory_element" and member not in d.relations[first].members:
            d.relations[first].members.append(copy.deepcopy(member))
    d.relations[first].members = [m for m in d.relations[first].members if m.role != "centerline"]
    del d.relations[second]
    for wid in (lb, rb):
        if not d.parents(("way", wid)):
            del d.ways[wid]
    generate_centerline(d, first)
    return first


def split_lane(d, lid, fraction=.5):
    _exclusive_boundaries(d, [lid])
    if d.parents(("relation", lid)):
        raise ValueError("车道被规则引用，拆分前请先处理对应关系")
    if not .01 < fraction < .99:
        raise ValueError("拆分比例必须在 0.01–0.99 之间")
    a, b = bound_nodes(d, lid)
    result = []
    for wid, ids in zip(d.bounds(lid), (a, b)):
        p = np.array([d.nodes[i].xyz for i in ids])
        distances = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
        target = distances[-1] * fraction
        i = min(len(ids) - 2, max(0, int(np.searchsorted(distances, target)) - 1))
        t = (target - distances[i]) / max(distances[i + 1] - distances[i], 1e-12)
        if t < 1e-7:
            nid = ids[i]; left, right = ids[:i + 1], ids[i:]
        elif t > 1-1e-7:
            nid = ids[i+1]; left, right = ids[:i + 2], ids[i+1:]
        else:
            nid = d.add_node(p[i] + t * (p[i + 1] - p[i]))
            left, right = ids[:i + 1] + [nid], [nid] + ids[i+1:]
        d.ways[wid].nodes = left
        result.append(d.add_way(right, d.ways[wid].tags, ids=True))
    r = d.relations[lid]
    rules = [m for m in r.members if m.role == "regulatory_element"]
    r.members = [m for m in r.members if m.role in ("left", "right")]
    new = d.add_relation([("way", result[0], "left"), ("way", result[1], "right"), *rules], r.tags)
    generate_centerline(d, lid); generate_centerline(d, new)
    return new


def create_connector(d, first, second, spacing=1.):
    a, b = bound_nodes(d, first); c, e = bound_nodes(d, second)
    pa, pb = lane_center(d, first), lane_center(d, second)
    path = g.bezier_connector(pa[-1], pa[-1] - pa[-2], pb[0], pb[1] - pb[0], spacing)
    width = (np.linalg.norm(d.nodes[a[-1]].xyz - d.nodes[b[-1]].xyz) +
             np.linalg.norm(d.nodes[c[0]].xyz - d.nodes[e[0]].xyz)) / 2
    lid = create_lane(d, path, width, float(d.relations[first].tags.get("speed_limit", 30)))
    left, right = d.bounds(lid)
    for wid, start, end in ((left, a[-1], c[0]), (right, b[-1], e[0])):
        merge_nodes(d, start, d.ways[wid].nodes[0])
        merge_nodes(d, end, d.ways[wid].nodes[-1])
    ta, tb = (pa[-1] - pa[-2])[:2], (pb[1] - pb[0])[:2]
    turn = math.atan2(ta[0]*tb[1]-ta[1]*tb[0], np.dot(ta, tb))
    d.relations[lid].tags["turn_direction"] = "left" if turn > .25 else "right" if turn < -.25 else "straight"
    generate_centerline(d, lid, spacing)
    return lid


def insert_point(d, wid, xyz):
    w = d.ways[wid]
    _, index, t, foot = g.nearest_segment(d.points(wid), xyz)
    if t < 1e-6:
        return w.nodes[index]
    if t > 1-1e-6:
        return w.nodes[index+1]
    nid = d.add_node(foot)
    w.nodes.insert(index + 1, nid)
    return nid


def split_way(d, wid, nid):
    w = d.ways[wid]
    parents = d.parents(("way", wid))
    if any(d.relations[i].tags.get("type") != "multipolygon" for _, i in parents):
        raise ValueError("线被车道或规则引用，请使用拆分车道或先调整关系")
    i = w.nodes.index(nid)
    if i == 0 or i == len(w.nodes)-1:
        raise ValueError("不能在首尾点拆分")
    new = d.add_way(w.nodes[i:], w.tags, ids=True)
    w.nodes = w.nodes[:i+1]
    for _, rid in parents:
        r = d.relations[rid]
        r.members = [m2 for m in r.members for m2 in
                     ([m, Member("way", new, m.role)] if m.ref == wid and m.type == "way" else [m])]
    return new


def join_ways(d, first, second):
    if d.parents(("way", first)) or d.parents(("way", second)):
        raise ValueError("仅可合并未被引用的线，请先调整关系")
    a, b = d.ways[first].nodes, list(d.ways[second].nodes)
    if a[-1] != b[0]:
        if a[-1] == b[-1]:
            b.reverse()
        else:
            raise ValueError("两条线必须共享端点")
    a.extend(b[1:]); del d.ways[second]


def delete_objects(d, refs):
    selected = set(map(tuple, refs))
    for ref in selected:
        parents = [p for p in d.parents(ref) if p not in selected]
        if parents:
            raise ValueError(f"{ref} 仍被 {parents[:3]} 引用，请先删除引用或同时选中")
    for kind, oid in selected:
        d.table(kind).pop(oid, None)


def clean_orphans(d):
    # Standalone Points and Ways may be meaningful, so only remove unreferenced
    # nodes without semantic tags. This is an explicitly invoked operation.
    referenced = {i for w in d.ways.values() for i in w.nodes}
    referenced |= {m.ref for r in d.relations.values() for m in r.members if m.type == "node"}
    remove = [i for i, n in d.nodes.items() if i not in referenced and not
              set(n.tags).difference({"ele", "local_x", "local_y"})]
    for i in remove:
        del d.nodes[i]
    return len(remove)


def transform(d, refs, dx=0., dy=0., dz=0., angle=0., scale=1., mirror=None):
    ids = d.geometry_nodes(refs)
    stops = [d.stops[i] for k, i in refs if k == "stop"]
    objects = [d.nodes[i] for i in ids] + stops
    if not objects:
        raise ValueError("选择中没有可变换的坐标")
    p = np.array([[o.x, o.y, o.z] for o in objects])
    center = p.mean(axis=0)
    c, s = math.cos(angle), math.sin(angle)
    if not 0 < scale <= 1000:
        raise ValueError("比例应大于零且不超过 1000")
    q = (p[:, :2] - center[:2]) @ np.array([[c, s], [-s, c]]) * scale + center[:2]
    if mirror:
        a, b = np.array(mirror[0])[:2], np.array(mirror[1])[:2]
        u = b - a
        if np.linalg.norm(u) < 1e-8:
            raise ValueError("镜像轴长度为零")
        u /= np.linalg.norm(u)
        q = a + 2 * ((q-a) @ u)[:, None] * u - (q-a)
    q += [dx, dy]
    for obj, xy, old in zip(objects, q, p):
        obj.x, obj.y, obj.z = float(xy[0]), float(xy[1]), float(old[2]+dz)
        if hasattr(obj, "yaw"):
            obj.yaw += angle


def clone(d, refs):
    closure = d.descendants(refs, follow_related_lanes=False)
    mapping = {ref: d.new_id() for ref in sorted(closure)}
    for kind, oid in sorted(closure):
        obj = copy.deepcopy(d.get((kind, oid)))
        obj.id = mapping[(kind, oid)]
        if kind == "way":
            obj.nodes = [mapping[("node", i)] for i in obj.nodes]
        if kind == "relation":
            for m in obj.members:
                m.ref = mapping.get((m.type, m.ref), m.ref)
        if hasattr(obj, "tags"):
            remap_tag_ids(obj.tags, mapping)
        if kind == "stop":
            obj.lanelet_id = mapping.get(("relation", obj.lanelet_id), obj.lanelet_id)
        d.table(kind)[obj.id] = obj
    return [(kind, mapping[(kind, oid)]) for kind, oid in refs if (kind, oid) in mapping]


def remap_tag_ids(tags, mapping):
    for tag, kind in (("traffic_light_id", "way"), ("intersection_area", "way")):
        if tag in tags:
            try:
                tags[tag] = str(mapping.get((kind, int(tags[tag])), int(tags[tag])))
            except ValueError:
                pass


def merge_document(d, other):
    if d.projection.to_dict() != other.projection.to_dict():
        raise ValueError("合并地图必须使用相同投影配置；请先显式统一坐标系")
    mapping = {(kind, oid): d.new_id() for kind, objects in
               (("node", other.nodes), ("way", other.ways), ("relation", other.relations))
               for oid in objects}
    for kind, oid in mapping:
        obj = copy.deepcopy(other.get((kind, oid))); obj.id = mapping[(kind, oid)]
        if kind == "way":
            obj.nodes = [mapping.get(("node", i), i) for i in obj.nodes]
        elif kind == "relation":
            for m in obj.members:
                m.ref = mapping.get((m.type, m.ref), m.ref)
        remap_tag_ids(obj.tags, mapping)
        d.table(kind)[obj.id] = obj
    return mapping


def add_rule(d, lane_ids, subtype, members, extra_tags=None):
    if not lane_ids:
        raise ValueError("至少选择一条关联车道")
    for lid in lane_ids:
        if lid not in d.relations or d.relations[lid].tags.get("type") != "lanelet":
            raise ValueError("规则只能附加到 Lanelet")
    rid = d.add_relation(members, {"type": "regulatory_element", "subtype": subtype,
                                   **(extra_tags or {})})
    for lid in lane_ids:
        d.relations[lid].members.append(Member("relation", rid, "regulatory_element"))
    return rid


def stop_line_at(d, lid, point, subtype="traffic_sign"):
    center = lane_center(d, lid)
    _, index, _, foot = g.nearest_segment(center, point)
    tangent = center[index+1] - center[index]
    tangent /= max(np.linalg.norm(tangent[:2]), 1e-12)
    a, b = bound_nodes(d, lid)
    width = np.linalg.norm(d.nodes[a[-1]].xyz - d.nodes[b[-1]].xyz)
    normal = np.array([-tangent[1], tangent[0], 0.])
    wid = d.add_way([foot+normal*width/2, foot-normal*width/2], {"type": "stop_line"})
    if subtype == "traffic_sign":
        sign = d.add_way([foot+normal*(width/2+.5), foot+normal*(width/2+.5)+[0, 0, 2]],
                         {"type": "traffic_sign", "subtype": "stop_sign"})
        rid = add_rule(d, [lid], "traffic_sign", [("way", sign, "refers"),
                        ("way", wid, "ref_line")], {"sign_type": "stop_sign"})
    else:
        location = foot + tangent*2 + [0, 0, 5]
        traffic = d.add_way([location-normal*.6, location+normal*.6],
                            {"type": "traffic_light", "subtype": "red_yellow_green", "height": "0.4"})
        bulbs = [d.add_node(location+normal*v,
                            {"type": "light_bulb", "color": color})
                 for v, color in zip((-.4, 0, .4), ("red", "yellow", "green"))]
        bulb_way = d.add_way(bulbs, {"type": "light_bulbs", "traffic_light_id": str(traffic)}, ids=True)
        rid = add_rule(d, [lid], "traffic_light", [("way", traffic, "refers"),
                        ("way", bulb_way, "light_bulbs"), ("way", wid, "ref_line")])
    return rid


def route(d, first, last):
    lanes = {r.id: bound_nodes(d, r.id) for r in d.lanelets()
             if r.tags.get("participant:vehicle", "yes") != "no" and
             r.tags.get("no_drivable_lane") != "yes"}
    if first not in lanes or last not in lanes:
        raise ValueError("起终点必须是车辆可通行车道")
    starts = {}
    for lid, (a, b) in lanes.items():
        starts.setdefault((a[0], b[0]), []).append(lid)
    distances = {first: 0.}; previous = {}; queue = [(0., first)]
    while queue:
        cost, lid = heapq.heappop(queue)
        if cost > distances[lid]:
            continue
        if lid == last:
            result = [last]
            while result[-1] != first:
                result.append(previous[result[-1]])
            return list(reversed(result))
        a, b = lanes[lid]
        for nxt in starts.get((a[-1], b[-1]), []):
            new = cost + g.length(lane_center(d, nxt))
            if new < distances.get(nxt, float("inf")):
                distances[nxt] = new; previous[nxt] = lid
                heapq.heappush(queue, (new, nxt))
    raise ValueError("未找到共享左右端点 ID 的有向路径；本地预览不包含换道和交通规则推理")
