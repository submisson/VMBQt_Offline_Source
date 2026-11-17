"""Native Qt raster 2D/3D viewport, with one shared coordinate/picking model.

The cloud raster has a per-pixel depth test. Vector overlays remain visible over
the cloud to support map editing. This is not a GPU out-of-core point-cloud engine.
"""
from __future__ import annotations

import math
import numpy as np
from PySide6.QtCore import Qt, QPointF, QRectF, Signal
from PySide6.QtGui import (QColor, QPainter, QPen, QPolygonF, QImage,
                           QPainterPath, QFont)
from PySide6.QtWidgets import QWidget

from . import geometry as g
from .operations import lane_center, bound_nodes
from .pointcloud import height_at


COLORS = {"line_thin": "#d9e4ee", "line_thick": "#e0b87a", "virtual": "#818f9f",
          "stop_line": "#ff735e", "traffic_light": "#ffd86f", "light_bulbs": "#f5ca63",
          "traffic_sign": "#f9a968", "curbstone": "#c092c6", "guard_rail": "#b9b8ce",
          "road_border": "#ada3d9", "detection_area": "#d3a05f", "parking_space": "#ab9bfc",
          "parking_lot": "#ab9bfc", "no_stopping_area": "#e783a3",
          "no_parking_area": "#e783a3", "speed_bump": "#eed071"}


class Camera:
    def __init__(self):
        self.target = np.zeros(3)
        self.scale = 9.
        self.yaw = 0.
        self.elevation = math.radians(50)
        self.is_3d = False
        self.perspective = False
        self.width = 1000; self.height = 700

    def basis(self):
        y = self.yaw; e = self.elevation if self.is_3d else math.pi/2
        return (np.array([math.cos(y), math.sin(y), 0.]),
                np.array([-math.sin(y)*math.sin(e), math.cos(y)*math.sin(e), math.cos(e)]),
                np.array([math.sin(y)*math.cos(e), -math.cos(y)*math.cos(e), math.sin(e)]))

    @property
    def distance(self):
        return max(2., self.height / (2 * math.tan(math.radians(22.5)) * self.scale))

    def project(self, points):
        p = np.asarray(points, dtype=float).reshape((-1, 3)) - self.target
        right, up, toward = self.basis()
        x, y, z = p @ right, p @ up, p @ toward
        depth = self.distance - z
        factor = self.distance / np.maximum(depth, .01) if self.is_3d and self.perspective else 1.
        return np.column_stack([self.width/2 + x*self.scale*factor,
                                self.height/2 - y*self.scale*factor]), depth

    def unproject(self, screen, plane_z=0.):
        right, up, toward = self.basis()
        x, y = (screen[0]-self.width/2)/self.scale, -(screen[1]-self.height/2)/self.scale
        if self.is_3d and self.perspective:
            origin = self.target + toward*self.distance
            direction = right*x + up*y - toward*self.distance
        else:
            origin = self.target + right*x + up*y
            direction = -toward
        if abs(direction[2]) < 1e-7:
            raise ValueError("相机与编辑平面平行")
        t = (plane_z-origin[2])/direction[2]
        return origin + direction*t

    def fit(self, points):
        p = np.asarray(points, dtype=float).reshape((-1, 3))
        p = p[np.isfinite(p).all(axis=1)]
        if not len(p):
            return
        self.target = (p.min(axis=0)+p.max(axis=0))/2
        right, up, _ = self.basis()
        q = p-self.target
        span_x, span_y = max(5., np.ptp(q @ right)), max(5., np.ptp(q @ up))
        self.scale = min(self.width*.8/span_x, self.height*.8/span_y)
        self.scale = float(np.clip(self.scale, .001, 1000))


class Canvas(QWidget):
    selectionChanged = Signal(list)
    toolChanged = Signal(str)
    drawn = Signal(str, list)
    moved = Signal(list, object)
    cursorInfo = Signal(str)

    def __init__(self, document, parent=None):
        super().__init__(parent)
        self.document = document
        self.camera = Camera()
        self.clouds = []
        self.selection = []
        self.tool = "select"
        self.draft = []
        self.hover_world = np.zeros(3)
        self.hover_node = None
        self.edit_z = 0.
        self.snap_enabled = True
        self.cloud_snap = False
        self.snap_grid = 0.
        self.snap_px = 10.
        self.pick_kind = "all"
        self.layers = {k: True for k in ("node", "way", "lane", "area", "stop", "trajectory", "cloud", "image")}
        self.show_ids = False
        self.show_directions = True
        self.show_grid = True
        self.background = "#131d28"
        self.grid_color = "#283745"
        self.cloud_color = "intensity"
        self.cloud_opacity = .8
        self.map_opacity = .32
        self.cloud_size = 1
        self.max_display = 180000
        self.z_min = -1e12; self.z_max = 1e12
        self.intensity_max = 255.
        self.image = None; self.image_spec = None
        self.image_opacity = .65
        self.route_ids = []
        self._press = None; self._button = None; self._last = None
        self._drag_node = None; self._drag_original = None
        self._rubber = None
        self._skip_release = False
        self._node_ids = []; self._node_xyz = np.empty((0, 3))
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setMouseTracking(True)
        self.setMinimumSize(500, 380)

    def refresh(self):
        self._node_ids = list(self.document.nodes)
        self._node_xyz = np.array([n.xyz for n in self.document.nodes.values()]).reshape((-1, 3))
        self.selection = [r for r in self.selection if self.document.get(r) is not None]
        self.update()

    def set_tool(self, tool):
        self.tool = tool; self.draft.clear(); self.update()
        self.toolChanged.emit(tool)
        self.setCursor(Qt.CursorShape.ArrowCursor if tool == "select" else Qt.CursorShape.CrossCursor)
        self.setFocus()

    def select(self, refs, emit=True):
        self.selection = list(dict.fromkeys(map(tuple, refs)))
        if emit:
            self.selectionChanged.emit(self.selection)
        self.update()

    def focus_refs(self, refs=None):
        ids = self.document.geometry_nodes(refs or self.selection)
        p = [self.document.nodes[i].xyz for i in ids]
        p.extend([[self.document.stops[i].x, self.document.stops[i].y, self.document.stops[i].z]
                  for k, i in (refs or self.selection) if k == "stop"])
        if p:
            self.camera.fit(p); self.update()

    def fit_all(self):
        arrays = [self._node_xyz]
        arrays.extend(c.xyz[::max(1, len(c.xyz)//10000)] for c in self.clouds if c.visible)
        if self.document.trajectory:
            arrays.append(np.array([[p.x, p.y, p.z] for p in self.document.trajectory]))
        if any(len(a) for a in arrays):
            self.camera.fit(np.vstack(arrays))
        self.update()

    def resizeEvent(self, event):
        self.camera.width, self.camera.height = self.width(), self.height()
        super().resizeEvent(event)

    def _polygon(self, points):
        xy, depth = self.camera.project(points)
        return QPolygonF([QPointF(float(a), float(b)) for a, b in xy]), depth

    def _line(self, painter, points, color, width=1., dashed=False):
        if len(points) < 2:
            return
        poly, depth = self._polygon(points)
        pen = QPen(QColor(color), width)
        if dashed:
            pen.setStyle(Qt.PenStyle.DashLine)
        painter.setPen(pen); painter.setBrush(Qt.BrushStyle.NoBrush)
        if self.camera.perspective and self.camera.is_3d and np.any(depth < .05):
            # Clip segments crossing the near plane before projecting.
            points = np.asarray(points, dtype=float)
            for i in range(len(points)-1):
                a,b=points[i].copy(),points[i+1].copy();da,db=depth[i],depth[i+1]
                if da<.05 and db<.05:continue
                if da<.05:a+=(b-a)*((.05-da)/(db-da))
                elif db<.05:b+=(a-b)*((.05-db)/(da-db))
                line,_=self._polygon([a,b]);painter.drawPolyline(line)
        else:
            painter.drawPolyline(poly)

    def paintEvent(self, event):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(self.background))
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        if self.show_grid:
            self._paint_grid(p)
        if self.layers["image"] and self.image is not None and self.image_spec:
            s = self.image_spec
            x, y, z, w, h = (s[k] for k in ("x", "y", "z", "width", "height"))
            target, depths = self._polygon([[x,y+h,z], [x+w,y+h,z], [x+w,y,z], [x,y,z]])
            from PySide6.QtGui import QTransform
            transform = QTransform()
            source = QPolygonF([QPointF(0,0), QPointF(self.image.width(),0),
                                QPointF(self.image.width(),self.image.height()), QPointF(0,self.image.height())])
            if np.all(depths > .05) and QTransform.quadToQuad(source, target, transform):
                p.save(); p.setOpacity(self.image_opacity); p.setTransform(transform)
                p.drawImage(QPointF(0,0), self.image); p.restore()
        if self.layers["cloud"]:
            self._paint_cloud(p)
        self._paint_map(p)
        if self.layers["trajectory"] and self.document.trajectory:
            self._line(p, [[v.x,v.y,v.z] for v in self.document.trajectory], "#f1c167", 2, True)
        for lid in self.route_ids:
            try:
                self._line(p, lane_center(self.document, lid), "#ffcf5c", 4)
            except (ValueError, KeyError):
                pass
        self._paint_draft(p)
        if self._rubber:
            p.setPen(QPen(QColor("#79bff8"), 1)); p.setBrush(QColor(90,170,240,25))
            p.drawRect(self._rubber.normalized())
        self._paint_hud(p)
        p.end()

    def _paint_grid(self, p):
        step = 10 ** math.floor(math.log10(max(.1, 90/self.camera.scale)))
        if step*self.camera.scale < 40:
            step *= 2
        extent = max(self.width(), self.height())/self.camera.scale * 1.6
        cx, cy, z = self.camera.target
        lo_x, lo_y = math.floor((cx-extent)/step)*step, math.floor((cy-extent)/step)*step
        count = min(150, int(extent*2/step)+2)
        for i in range(count):
            x, y = lo_x+i*step, lo_y+i*step
            self._line(p, [[x,cy-extent,self.edit_z],[x,cy+extent,self.edit_z]], self.grid_color)
            self._line(p, [[cx-extent,y,self.edit_z],[cx+extent,y,self.edit_z]], self.grid_color)

    def _paint_cloud(self, painter):
        clouds = [c for c in self.clouds if c.visible]
        total = sum(len(c.xyz) for c in clouds)
        if not total:
            return
        stride = max(1, math.ceil(total/self.max_display))
        all_xy, all_depth, all_colors = [], [], []
        for c in clouds:
            indices = np.arange(0, len(c.xyz), stride)
            points = c.xyz[indices]
            mask = (points[:,2] >= self.z_min) & (points[:,2] <= self.z_max)
            indices, points = indices[mask], points[mask]
            if not len(points):
                continue
            xy, depth = self.camera.project(points)
            if self.cloud_color == "rgb" and c.rgb is not None:
                colors = c.rgb[indices]
            elif self.cloud_color == "height":
                t = (points[:,2]-np.min(c.xyz[:,2]))/max(np.ptp(c.xyz[:,2]), .01)
                colors = np.clip(np.column_stack([1.5-abs(4*t-3), 1.5-abs(4*t-2),
                                                  1.5-abs(4*t-1)]), 0, 1)*255
            else:
                t = np.clip(c.intensity[indices]/max(self.intensity_max, .001),0,1)
                if not np.any(t):
                    t = np.full_like(t,.45)
                colors = np.column_stack([60+190*t,80+170*t,95+155*t])
            all_xy.append(xy); all_depth.append(depth); all_colors.append(colors)
        if not all_xy:
            return
        xy = np.rint(np.vstack(all_xy)).astype(np.int64)
        depth = np.concatenate(all_depth); colors = np.vstack(all_colors).astype(np.uint8)
        valid = (xy[:,0]>=0)&(xy[:,1]>=0)&(xy[:,0]<self.width())&(xy[:,1]<self.height())&(depth>.05)
        xy, depth, colors = xy[valid], depth[valid], colors[valid]
        if not len(xy):
            return
        order = np.argsort(depth, kind="stable")
        keys = xy[order,1]*self.width()+xy[order,0]
        _, positions = np.unique(keys, return_index=True)
        ix = order[positions]
        rgba = np.zeros((self.height(),self.width(),4),dtype=np.uint8)
        for dx in range(self.cloud_size):
            for dy in range(self.cloud_size):
                x, y = xy[ix,0]+dx, xy[ix,1]+dy
                keep = (x<self.width())&(y<self.height())
                rgba[y[keep],x[keep],:3] = colors[ix[keep]]
                rgba[y[keep],x[keep],3] = int(255*self.cloud_opacity)
        image = QImage(rgba.data, self.width(), self.height(), self.width()*4,
                       QImage.Format.Format_RGBA8888)
        painter.drawImage(0,0,image)

    def _paint_map(self, p):
        d = self.document; selected = set(self.selection)
        fills = []
        for r in d.relations.values():
            if r.tags.get("type") == "lanelet" and self.layers["lane"]:
                try:
                    a, b = bound_nodes(d,r.id)
                    points = [d.nodes[i].xyz for i in a+b[::-1]]
                    poly, depth = self._polygon(points)
                    fills.append((float(np.mean(depth)),r,poly))
                except (ValueError,KeyError):
                    continue
        for _, r, poly in sorted(fills, key=lambda v: -v[0]):
            color = QColor("#4faadf" if r.tags.get("subtype")=="crosswalk" else "#34b797")
            if ("relation",r.id) in selected:
                color = QColor("#f3c567")
            color.setAlphaF(self.map_opacity)
            p.setBrush(color); p.setPen(Qt.PenStyle.NoPen); p.drawPolygon(poly)
            try:
                points = lane_center(d,r.id,2)
                self._line(p,points,"#69b3a8",1,True)
                if self.show_directions and len(points)>1:
                    i = max(0,(len(points)-2)//2)
                    screen, _ = self.camera.project(points[i:i+2])
                    a,b = screen
                    tangent = b-a; tangent /= max(np.linalg.norm(tangent),1e-12)
                    middle = (a+b)/2
                    normal = np.array([-tangent[1],tangent[0]])
                    arrow = QPolygonF([QPointF(*middle),QPointF(*(middle-tangent*9+normal*4)),
                                       QPointF(*(middle-tangent*9-normal*4))])
                    p.setPen(Qt.PenStyle.NoPen); p.setBrush(QColor("#8ed4c4"));p.drawPolygon(arrow)
                if self.show_ids or ("relation",r.id) in selected:
                    self._label(p,points[len(points)//2],f"L{r.id} · {r.tags.get('speed_limit','?')} km/h")
            except (ValueError,KeyError):
                pass
        if self.layers["area"]:
            for r in d.relations.values():
                if r.tags.get("type") != "multipolygon":
                    continue
                path = QPainterPath(); path.setFillRule(Qt.FillRule.OddEvenFill)
                for ids in self._area_rings(r):
                    poly, _ = self._polygon([d.nodes[i].xyz for i in ids if i in d.nodes])
                    path.addPolygon(poly)
                color = QColor("#a18cdb");color.setAlphaF(self.map_opacity)
                p.setBrush(color);p.setPen(QPen(QColor("#b59ceb"),1.5));p.drawPath(path)
        if self.layers["way"]:
            for w in d.ways.values():
                points = d.points(w.id)
                if len(points)<2:
                    continue
                color = "#ffd36c" if ("way",w.id) in selected else COLORS.get(w.tags.get("type"),"#bbceda")
                if w.tags.get("area")=="yes":
                    poly, _ = self._polygon(points)
                    fill = QColor(color);fill.setAlphaF(self.map_opacity)
                    p.setPen(QPen(QColor(color),1.5));p.setBrush(fill);p.drawPolygon(poly)
                else:
                    self._line(p,points,color,3 if ("way",w.id) in selected else 1.5,
                               w.tags.get("subtype")=="dashed")
                if self.camera.is_3d and "height" in w.tags:
                    try:
                        h=float(w.tags["height"])
                        if math.isfinite(h) and 0<h<1000:
                            top=points+np.array([0.,0.,h])
                            self._line(p,top,color,1.)
                            for base,upper in zip(points,top):self._line(p,[base,upper],color,1.)
                    except ValueError:pass
                if self.show_ids and len(d.ways)<400 or ("way",w.id) in selected:
                    self._label(p,points[len(points)//2],f"W{w.id}",color)
        if self.layers["node"]:
            selected_nodes = set(d.geometry_nodes(self.selection))
            if len(self._node_xyz):
                xy, depth = self.camera.project(self._node_xyz)
                for i, screen, dep in zip(self._node_ids,xy,depth):
                    if dep<.05 or not (0<=screen[0]<=self.width() and 0<=screen[1]<=self.height()):
                        continue
                    active = i in selected_nodes
                    if not active and (self.camera.scale<2 or len(xy)>7000):
                        continue
                    radius = 4.5 if ("node",i) in selected else 2.7 if active else 2.
                    p.setPen(Qt.PenStyle.NoPen)
                    p.setBrush(QColor("#ffce69" if active else "#e5eef2"))
                    p.drawEllipse(QPointF(*screen),radius,radius)
                    if ("node",i) in selected:
                        self._label(p,d.nodes[i].xyz,str(i),"#ffce69")
        if self.layers["stop"]:
            for s in d.stops.values():
                xy,_ = self.camera.project([[s.x,s.y,s.z]])
                q = QPointF(*xy[0]);p.setPen(QPen(QColor("#f494cf"),2));p.setBrush(QColor("#703b63"))
                p.drawEllipse(q,6,6)
                end = [s.x+math.cos(s.yaw)*2,s.y+math.sin(s.yaw)*2,s.z]
                self._line(p,[[s.x,s.y,s.z],end],"#f494cf",2)
                self._label(p,[s.x,s.y,s.z],s.name,"#f494cf")

    def _area_rings(self, relation):
        rings = []
        for role in ("outer","inner"):
            pending = [list(self.document.ways[m.ref].nodes) for m in relation.members
                       if m.type=="way" and m.role==role and m.ref in self.document.ways]
            while pending:
                chain = pending.pop(0)
                changed = True
                while changed and chain and chain[0]!=chain[-1]:
                    changed = False
                    for i, ids in enumerate(pending):
                        if ids and chain[-1] in (ids[0],ids[-1]):
                            if chain[-1]==ids[-1]: ids=ids[::-1]
                            chain.extend(ids[1:]);pending.pop(i);changed=True;break
                if len(chain)>=3:
                    rings.append(chain)
        return rings

    def _label(self,p,point,text,color="#d6e1eb"):
        xy,depth = self.camera.project([point])
        if depth[0]>.05:
            p.setPen(QColor(color));p.setFont(QFont("Sans Serif",9))
            p.drawText(QPointF(xy[0,0]+6,xy[0,1]-6),text)

    def _paint_draft(self,p):
        if self.draft:
            points = [v[0] for v in self.draft]+[self.hover_world]
            self._line(p,points,"#ffce6b",2,True)
            if self.tool=="measure":
                distance=g.length(points);planar=g.length(points,True)
                self._label(p,self.hover_world,f"XY {planar:.3f} m · 3D {distance:.3f} m","#ffce6b")
        if self.tool!="select":
            xy,_ = self.camera.project([self.hover_world])
            p.setPen(QPen(QColor("#ffce6b" if self.hover_node else "#80bcec"),1))
            q=QPointF(*xy[0]);p.drawEllipse(q,6,6)

    def _paint_hud(self,p):
        p.setPen(QColor("#94aabd"));p.setFont(QFont("Sans Serif",9))
        mode="3D · 透视" if self.camera.is_3d and self.camera.perspective else "3D · 正交" if self.camera.is_3d else "2D · 俯视"
        p.drawText(18,25,mode)
        p.drawText(18,self.height()-16,"左键选择/绘制  ·  中键平移  ·  右键旋转  ·  滚轮缩放  ·  Enter 完成  ·  Esc 取消")
        bar=100;distance=bar/self.camera.scale
        p.setPen(QPen(QColor("#bdcdd9"),2));p.drawLine(self.width()-140,26,self.width()-40,26)
        p.drawText(self.width()-140,45,f"{distance:.2f} m")

    def _node_pick(self, position):
        if not len(self._node_xyz):
            return None
        xy,depth=self.camera.project(self._node_xyz)
        dist=((xy-np.array(position))**2).sum(axis=1);dist[depth<=.05]=float("inf")
        index=int(np.argmin(dist))
        return self._node_ids[index] if dist[index]<=self.snap_px**2 else None

    def world_at(self, position):
        nid=self._node_pick(position) if self.snap_enabled else None
        if nid is not None:
            return self.document.nodes[nid].xyz,nid
        xyz=self.camera.unproject(position,self.edit_z)
        if self.snap_grid>0:
            xyz[:2]=np.round(xyz[:2]/self.snap_grid)*self.snap_grid
        if self.cloud_snap:
            z=height_at(self.clouds,xyz,.8)
            if z is not None: xyz[2]=z
        return xyz,None

    def pick(self, position):
        d=self.document
        if self.pick_kind in ("all","node") and self.layers["node"]:
            nid=self._node_pick(position)
            if nid is not None: return "node",nid
        if self.pick_kind in ("all","way") and self.layers["way"]:
            best=(8.,None)
            for w in d.ways.values():
                p=d.points(w.id)
                if len(p)<2: continue
                xy,depth=self.camera.project(p)
                if np.any(depth<=.05): continue
                pts=np.column_stack([xy,np.zeros(len(xy))])
                distance,*_=g.nearest_segment(pts,[*position,0])
                if distance<best[0]: best=(distance,("way",w.id))
            if best[1]: return best[1]
        if self.pick_kind in ("all","stop") and self.layers["stop"]:
            for s in d.stops.values():
                xy,_=self.camera.project([[s.x,s.y,s.z]])
                if np.linalg.norm(xy[0]-position)<10: return "stop",s.id
        if self.pick_kind in ("all","lane") and self.layers["lane"]:
            for r in reversed(d.lanelets()):
                try:
                    a,b=bound_nodes(d,r.id)
                    xy,depth=self.camera.project([d.nodes[i].xyz for i in a+b[::-1]])
                    if np.all(depth>.05) and g.contains(xy,position): return "relation",r.id
                except (KeyError,ValueError): pass
        if self.pick_kind in ("all","area") and self.layers["area"]:
            for r in reversed(list(d.relations.values())):
                if r.tags.get("type")!="multipolygon":continue
                inside=False
                for ids in self._area_rings(r):
                    xy,_=self.camera.project([d.nodes[i].xyz for i in ids if i in d.nodes])
                    if g.contains(xy,position):inside=not inside
                if inside:return "relation",r.id
        return None

    def mousePressEvent(self,event):
        self.setFocus();pos=(event.position().x(),event.position().y())
        self._press=np.array(pos);self._last=np.array(pos);self._button=event.button()
        if event.button()==Qt.MouseButton.LeftButton and self.tool=="select":
            ref=self.pick(pos)
            if event.modifiers()&Qt.KeyboardModifier.ShiftModifier:
                self._rubber=QRectF(QPointF(*pos),QPointF(*pos))
            elif ref and ref[0]=="node":
                self._drag_node=ref[1];self._drag_original=self.document.nodes[ref[1]].xyz.copy()

    def mouseMoveEvent(self,event):
        pos=np.array([event.position().x(),event.position().y()])
        try:self.hover_world,self.hover_node=self.world_at(pos)
        except ValueError:return
        self.cursorInfo.emit(f"X {self.hover_world[0]:.3f}   Y {self.hover_world[1]:.3f}   Z {self.hover_world[2]:.3f} m")
        if self._press is not None and self._last is not None:
            delta=pos-self._last
            if self._button==Qt.MouseButton.MiddleButton:
                a=self.camera.unproject(self._last,self.camera.target[2]);b=self.camera.unproject(pos,self.camera.target[2])
                self.camera.target+=a-b
            elif self._button==Qt.MouseButton.RightButton:
                self.camera.yaw+=delta[0]*.006
                if self.camera.is_3d:self.camera.elevation=float(np.clip(self.camera.elevation+delta[1]*.006,.15,1.55))
            elif self._rubber is not None:
                self._rubber=QRectF(QPointF(*self._press),QPointF(*pos))
            elif self._drag_node is not None and np.linalg.norm(pos-self._press)>3:
                original=self._drag_original
                a=self.camera.unproject(self._press,original[2]);b=self.camera.unproject(pos,original[2])
                n=self.document.nodes[self._drag_node];n.x,n.y,n.z=map(float,original+b-a)
                self.refresh()
            self._last=pos
        self.update()

    def mouseReleaseEvent(self,event):
        if self._skip_release:
            self._skip_release=False
            self._press=None;self._button=None;self._last=None
            return
        if self._press is None:return
        pos=np.array([event.position().x(),event.position().y()]);distance=np.linalg.norm(pos-self._press)
        if self._drag_node is not None:
            n=self.document.nodes[self._drag_node];new=n.xyz.copy();original=self._drag_original
            n.x,n.y,n.z=map(float,original)
            if distance>3:
                self.moved.emit([("node",n.id)],new-original)
        if self._button==Qt.MouseButton.LeftButton:
            shift=bool(event.modifiers()&Qt.KeyboardModifier.ShiftModifier)
            if self.tool=="select":
                if shift and self._rubber and distance>4:
                    rect=self._rubber.normalized();refs=list(self.selection)
                    xy,_=self.camera.project(self._node_xyz)
                    if self.pick_kind in ("all","node"):
                        refs.extend(("node",i) for i,p in zip(self._node_ids,xy) if rect.contains(QPointF(*p)))
                    else:
                        for kind,objects in (("way",self.document.ways),("relation",self.document.relations)):
                            for oid,obj in objects.items():
                                if kind=="relation" and self.pick_kind=="lane" and obj.tags.get("type")!="lanelet":continue
                                if kind=="way" and self.pick_kind!="way":continue
                                pts=[self.document.nodes[i].xyz for i in self.document.geometry_nodes([(kind,oid)])]
                                if pts:
                                    screen,_=self.camera.project(pts)
                                    if all(rect.contains(QPointF(*q)) for q in screen):refs.append((kind,oid))
                    self.select(refs)
                elif distance<4:
                    ref=self.pick(pos)
                    refs=list(self.selection) if shift else []
                    if ref:
                        if ref in refs:refs.remove(ref)
                        else:refs.append(ref)
                    self.select(refs)
            elif distance<4:
                xyz,nid=self.world_at(pos);self.draft.append((xyz.tolist(),nid))
                if self.tool in ("point","stop","stopline","trafficlight","insert","split"):
                    self.finish()
                elif self.tool in ("rectangle","circle","mirror") and len(self.draft)==2:
                    self.finish()
        self._press=None;self._button=None;self._last=None;self._drag_node=None;self._drag_original=None;self._rubber=None
        self.refresh()

    def mouseDoubleClickEvent(self,event):
        if event.button()==Qt.MouseButton.LeftButton and self.tool!="select":
            self._skip_release=True
            self.finish()

    def finish(self):
        if self.draft:
            draft=self.draft;self.draft=[]
            self.drawn.emit(self.tool,draft)
        self.update()

    def wheelEvent(self,event):
        pos=[event.position().x(),event.position().y()]
        before=self.camera.unproject(pos,self.camera.target[2])
        self.camera.scale=float(np.clip(self.camera.scale*math.exp(event.angleDelta().y()/700),.001,3000))
        after=self.camera.unproject(pos,self.camera.target[2]);self.camera.target+=before-after;self.update()

    def keyPressEvent(self,event):
        if event.key() in (Qt.Key.Key_Return,Qt.Key.Key_Enter):self.finish()
        elif event.key()==Qt.Key.Key_Escape:self.draft=[];self.set_tool("select")
        elif event.key()==Qt.Key.Key_Backspace and self.draft:self.draft.pop();self.update()
        elif event.key() in (Qt.Key.Key_Left,Qt.Key.Key_Right,Qt.Key.Key_Up,Qt.Key.Key_Down) and self.selection:
            step=.01 if event.modifiers()&Qt.KeyboardModifier.ShiftModifier else .1
            direction={Qt.Key.Key_Left:(-step,0,0),Qt.Key.Key_Right:(step,0,0),
                       Qt.Key.Key_Up:(0,step,0),Qt.Key.Key_Down:(0,-step,0)}[event.key()]
            self.moved.emit(self.selection,np.array(direction))
        else:super().keyPressEvent(event)
