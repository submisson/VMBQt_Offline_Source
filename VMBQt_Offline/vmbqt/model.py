"""OSM primitive model. Identity is (OSM type, 64-bit-compatible integer id).

Ways and relations refer to primitives by identity, never by a copied coordinate.
All geometry operations therefore update every owner of a shared point.
"""
from __future__ import annotations

import copy
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Callable

import numpy as np

from .projection import ProjectionConfig, Projector
from .storage import atomic_text

Ref = tuple[str, int]


@dataclass
class Node:
    id: int
    x: float
    y: float
    z: float = 0.
    tags: dict[str, str] = field(default_factory=dict)
    attrs: dict[str, str] = field(default_factory=dict)
    extras: list[str] = field(default_factory=list)

    @property
    def xyz(self):
        return np.array([self.x, self.y, self.z], dtype=float)


@dataclass
class Way:
    id: int
    nodes: list[int]
    tags: dict[str, str] = field(default_factory=dict)
    attrs: dict[str, str] = field(default_factory=dict)
    extras: list[str] = field(default_factory=list)


@dataclass
class Member:
    type: str
    ref: int
    role: str = ""


@dataclass
class Relation:
    id: int
    members: list[Member]
    tags: dict[str, str] = field(default_factory=dict)
    attrs: dict[str, str] = field(default_factory=dict)
    extras: list[str] = field(default_factory=list)


@dataclass
class StopPoint:
    id: int
    x: float
    y: float
    z: float
    yaw: float = 0.
    name: str = "stop"
    lanelet_id: int = 0
    extra: dict[str, str] = field(default_factory=dict)


@dataclass
class TrajectoryPoint:
    id: int
    x: float
    y: float
    z: float
    yaw: float = 0.
    velocity: float = 0.
    lanelet_id: int = 0
    extra: dict[str, str] = field(default_factory=dict)


class Document:
    def __init__(self):
        self.nodes: dict[int, Node] = {}
        self.ways: dict[int, Way] = {}
        self.relations: dict[int, Relation] = {}
        self.stops: dict[int, StopPoint] = {}
        self.trajectory: list[TrajectoryPoint] = []
        self.projection = ProjectionConfig()
        self.name = "Untitled"
        self.root_attrs = {"version": "0.6", "generator": "VMBQt-Offline-0.1"}
        self.root_extras: list[str] = []
        self._next = 1

    def table(self, kind):
        return {"node": self.nodes, "way": self.ways, "relation": self.relations,
                "stop": self.stops}[kind]

    def get(self, ref: Ref):
        return self.table(ref[0]).get(ref[1])

    def new_id(self):
        value = self._next
        self._next += 1
        return value

    def add_node(self, xyz, tags=None):
        if len(xyz) != 3 or not np.isfinite(xyz).all():
            raise ValueError("点坐标必须是 3 个有限数值")
        n = Node(self.new_id(), *map(float, xyz), tags=tags or {})
        self.nodes[n.id] = n
        return n.id

    def add_way(self, points, tags=None, ids=False):
        nodes = list(points) if ids else [self.add_node(p) for p in points]
        w = Way(self.new_id(), nodes, dict(tags or {}))
        self.ways[w.id] = w
        return w.id

    def add_relation(self, members, tags=None):
        r = Relation(self.new_id(), [Member(*m) if not isinstance(m, Member)
                                    else copy.deepcopy(m) for m in members], dict(tags or {}))
        self.relations[r.id] = r
        return r.id

    def lanelets(self):
        return [r for r in self.relations.values() if r.tags.get("type") == "lanelet"]

    def bounds(self, relation_id):
        r = self.relations[relation_id]
        return tuple(next((m.ref for m in r.members if m.type == "way" and m.role == role),
                          None) for role in ("left", "right"))

    def points(self, way_id):
        return np.array([self.nodes[i].xyz for i in self.ways[way_id].nodes
                         if i in self.nodes], dtype=float).reshape((-1, 3))

    def descendants(self, refs, follow_related_lanes=True):
        result = set()
        roots = set(map(tuple, refs))
        pending = list(roots)
        while pending:
            ref = tuple(pending.pop())
            if ref in result:
                continue
            obj = self.get(ref)
            if obj is None:
                continue
            result.add(ref)
            if ref[0] == "way":
                pending.extend(("node", i) for i in obj.nodes)
            elif ref[0] == "relation":
                for m in obj.members:
                    if m.type not in ("node", "way", "relation"):
                        continue
                    # Rules reference other lanes; they do not own those lanes.
                    # Otherwise moving/copying a road with right_of_way could
                    # recursively move/copy an entire connected road network.
                    child = (m.type, m.ref)
                    if not follow_related_lanes and obj.tags.get("type") == "regulatory_element" and \
                            m.type == "relation" and m.ref in self.relations and \
                            self.relations[m.ref].tags.get("type") == "lanelet" and child not in roots:
                        continue
                    pending.append(child)
        return result

    def geometry_nodes(self, refs):
        return sorted(i for k, i in self.descendants(refs, follow_related_lanes=False) if k == "node")

    def parents(self, ref):
        result = []
        kind, oid = ref
        if kind == "node":
            result.extend(("way", w.id) for w in self.ways.values() if oid in w.nodes)
        result.extend(("relation", r.id) for r in self.relations.values()
                      if any(m.type == kind and m.ref == oid for m in r.members))
        return result

    def state(self):
        return {"schema": 1, "name": self.name,
                "projection": self.projection.to_dict(),
                "root_attrs": self.root_attrs, "root_extras": self.root_extras,
                "nodes": [asdict(n) for n in self.nodes.values()],
                "ways": [asdict(w) for w in self.ways.values()],
                "relations": [asdict(r) for r in self.relations.values()],
                "stops": [asdict(s) for s in self.stops.values()],
                "trajectory": [asdict(p) for p in self.trajectory], "next": self._next}

    def restore(self, value):
        if value.get("schema", 1) != 1:
            raise ValueError("不支持此工程文件版本")
        self.name = value.get("name", "Untitled")
        self.projection = ProjectionConfig(**value.get("projection", {}))
        Projector(self.projection)
        self.root_attrs = dict(value.get("root_attrs", {"version": "0.6"}))
        self.root_extras = list(value.get("root_extras", []))
        self.nodes = {int(n["id"]): Node(**n) for n in value.get("nodes", [])}
        self.ways = {int(w["id"]): Way(**w) for w in value.get("ways", [])}
        self.relations = {}
        for data in value.get("relations", []):
            r = dict(data)
            r["members"] = [Member(**m) for m in r["members"]]
            self.relations[int(r["id"])] = Relation(**r)
        self.stops = {int(s["id"]): StopPoint(**s) for s in value.get("stops", [])}
        self.trajectory = [TrajectoryPoint(**p) for p in value.get("trajectory", [])]
        self._next = max(int(value.get("next", 1)), max(
            [0, *self.nodes, *self.ways, *self.relations, *self.stops,
             *(p.id for p in self.trajectory)]) + 1)

    def snapshot(self):
        return json.dumps(self.state(), ensure_ascii=False, separators=(",", ":"),
                          allow_nan=False)

    def write_osm(self, path):
        projector = Projector(self.projection)
        root = ET.Element("osm", self.root_attrs)
        for extra in self.root_extras:
            root.append(ET.fromstring(extra))
        for kind, objects in (("node", self.nodes), ("way", self.ways),
                              ("relation", self.relations)):
            for obj in sorted(objects.values(), key=lambda o: o.id):
                attrs = dict(obj.attrs); attrs["id"] = str(obj.id)
                tags = dict(obj.tags)
                if kind == "node":
                    if not all(math.isfinite(v) for v in (obj.x, obj.y, obj.z)):
                        raise ValueError(f"点 {obj.id} 包含非有限坐标")
                    lat, lon = projector.reverse(obj.x, obj.y)
                    attrs.update(lat=f"{lat:.12f}", lon=f"{lon:.12f}")
                    tags.update(local_x=f"{obj.x:.9f}", local_y=f"{obj.y:.9f}",
                                ele=f"{obj.z:.9f}")
                element = ET.SubElement(root, kind, attrs)
                if kind == "way":
                    for nid in obj.nodes:
                        ET.SubElement(element, "nd", {"ref": str(nid)})
                if kind == "relation":
                    for member in obj.members:
                        ET.SubElement(element, "member", {"type": member.type,
                                      "ref": str(member.ref), "role": member.role})
                for key, value in sorted(tags.items()):
                    ET.SubElement(element, "tag", {"k": key, "v": str(value)})
                for extra in obj.extras:
                    element.append(ET.fromstring(extra))
        ET.indent(root, space="  ")
        atomic_text(path, '<?xml version="1.0" encoding="UTF-8"?>\n' +
                    ET.tostring(root, encoding="unicode") + "\n")

    @classmethod
    def read_osm(cls, path, projection=None):
        root = ET.parse(path).getroot()
        if root.tag != "osm":
            raise ValueError("文件根元素必须为 osm")
        d = cls()
        d.projection = copy.deepcopy(projection or ProjectionConfig())
        projector = Projector(d.projection)
        d.name = Path(path).stem
        d.root_attrs = dict(root.attrib)
        for e in root:
            if e.tag not in ("node", "way", "relation"):
                d.root_extras.append(ET.tostring(e, encoding="unicode"))
                continue
            oid = int(e.attrib["id"])
            tags = {}
            for t in e.findall("tag"):
                k, v = t.attrib["k"], t.attrib["v"]
                if k in tags:
                    raise ValueError(f"{e.tag} {oid} 出现重复标签 {k}")
                tags[k] = v
            attrs = dict(e.attrib)
            attrs.pop("id", None)
            extras = [ET.tostring(t, encoding="unicode") for t in e
                      if t.tag not in ("tag", "nd", "member")]
            if oid in d.table(e.tag):
                raise ValueError(f"重复的 {e.tag} ID {oid}")
            if e.tag == "node":
                if "local_x" in tags and "local_y" in tags:
                    x, y = float(tags["local_x"]), float(tags["local_y"])
                else:
                    x, y = projector.forward(float(e.attrib.get("lat", 0)),
                                              float(e.attrib.get("lon", 0)))
                z = float(tags.get("ele", 0))
                if not all(math.isfinite(v) for v in (x, y, z)):
                    raise ValueError(f"点 {oid} 坐标无效")
                d.nodes[oid] = Node(oid, x, y, z, tags, attrs, extras)
            elif e.tag == "way":
                d.ways[oid] = Way(oid, [int(n.attrib["ref"]) for n in e.findall("nd")],
                                  tags, attrs, extras)
            else:
                members = [Member(m.attrib["type"], int(m.attrib["ref"]),
                                  m.attrib.get("role", "")) for m in e.findall("member")]
                d.relations[oid] = Relation(oid, members, tags, attrs, extras)
        d._next = max([0, *d.nodes, *d.ways, *d.relations]) + 1
        return d


class History:
    """Whole-model transactions; point clouds are intentionally outside history.

    Bound by both a step count and a byte budget. A failed edit restores the model.
    """
    def __init__(self, document, limit=60, byte_limit=96 * 1024 * 1024):
        self.document = document
        self.limit = limit
        self.byte_limit = byte_limit
        self.undo_items = []
        self.redo_items = []
        self.saved = document.snapshot()

    @property
    def dirty(self):
        return self.document.snapshot() != self.saved

    def mark_saved(self):
        self.saved = self.document.snapshot()

    def edit(self, label: str, operation: Callable):
        before = self.document.snapshot()
        try:
            result = operation()
            if self.document.snapshot() == before:
                return result
            # A previous validator signature no longer describes the edited map.
            self.document.root_extras = [s for s in self.document.root_extras
                                         if ET.fromstring(s).tag != "validation"]
            after = self.document.snapshot()
        except Exception:
            self.document.restore(json.loads(before))
            raise
        self.undo_items.append((label, before, after))
        self.redo_items.clear()
        while len(self.undo_items) > self.limit or (len(self.undo_items) > 1 and
                sum(len(a) + len(b) for _, a, b in self.undo_items) > self.byte_limit):
            self.undo_items.pop(0)
        return result

    def undo(self):
        if self.undo_items:
            item = self.undo_items.pop()
            self.document.restore(json.loads(item[1]))
            self.redo_items.append(item)
            return item[0]

    def redo(self):
        if self.redo_items:
            item = self.redo_items.pop()
            self.document.restore(json.loads(item[2]))
            self.undo_items.append(item)
            return item[0]
