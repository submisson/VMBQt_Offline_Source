#!/usr/bin/env python3
"""Deterministically create an artificial, non-geographic training map and PCD."""
from pathlib import Path
import sys
import math
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from vmbqt.model import Document, StopPoint, TrajectoryPoint
from vmbqt import operations as op
from vmbqt.exchange import save_project,write_csv


def create(destination):
    destination=Path(destination);destination.mkdir(parents=True,exist_ok=True)
    d=Document();d.name="Demo — Offline training map"
    d.root_extras=['<MapMetaInfo format_version="1" map_version="1" />']
    path=np.array([[0,0,0],[12,0,0],[24,0,.12],[36,1,.3],[46,5,.6]])
    first=op.create_lane(d,path,3.5,30)
    adjacent=op.add_adjacent(d,first,"left",3.5)
    d.relations[adjacent].tags["speed_limit"]="30"
    left,_=d.bounds(first);d.ways[left].tags.update(subtype="dashed",lane_change="yes")
    continuation=op.create_lane(d,[[46,5,.6],[55,9,.8],[63,16,1.2],[65,28,1.7]],3.5,20)
    op.connect_lanes(d,first,continuation,4)
    for lid in (first,adjacent,continuation):op.generate_centerline(d,lid,1.)
    op.stop_line_at(d,first,[33,.5,.25],"traffic_light")
    crosswalk=op.create_lane(d,[[30,-4,.2],[30,8,.2]],3.,10,"crosswalk")
    op.add_rule(d,[first,adjacent],"crosswalk",[("relation",crosswalk,"refers")])
    parking=d.add_way([[3,9,0],[19,9,0],[19,20,0],[3,20,0]],{"type":"parking_lot","area":"yes"})
    d.ways[parking].nodes.append(d.ways[parking].nodes[0])
    detection=d.add_way([[35,-1,.25],[41,-1,.3],[41,5,.4],[35,5,.3]],{"type":"detection_area","area":"yes"})
    stopline=d.add_way([[31,-1.5,.2],[31,1.5,.2]],{"type":"stop_line"})
    op.add_rule(d,[first],"detection_area",[("way",detection,"refers"),("way",stopline,"ref_line")])
    sid=d.new_id();d.stops[sid]=StopPoint(sid,15,0,0,0,"Stop A",first)
    points=op.lane_center(d,first)
    d.trajectory=[TrajectoryPoint(d.new_id(),*map(float,p),0.,5.,first) for p in points]
    d.write_osm(destination/"lanelet2_map.osm")
    d.projection.write_yaml(destination/"map_projector_info.yaml")
    write_csv(destination/"trajectory.csv",d.trajectory)
    write_csv(destination/"stop_points.csv",d.stops.values(),"stop")
    # Sparse road surface, high intensity markings, plus elevated roadside objects.
    rng=np.random.default_rng(7)
    x=rng.uniform(-8,75,40000);y=rng.uniform(-10,33,40000)
    z=.0003*np.maximum(x-20,0)**2+rng.normal(0,.035,len(x))
    intensity=rng.uniform(20,100,len(x))
    road=(np.abs(y)<1.9)&(x<35)
    intensity[road]=rng.uniform(80,145,road.sum())
    points_cloud=np.column_stack([x,y,z,intensity])
    for wid in [*d.bounds(first),*d.bounds(continuation),d.bounds(adjacent)[0]]:
        p=d.points(wid)
        for a,b in zip(p[:-1],p[1:]):
            t=np.linspace(0,1,max(2,int(np.linalg.norm(b-a)*40)))[:,None]
            xyz=a+(b-a)*t+rng.normal(0,.025,(len(t),3))
            points_cloud=np.vstack([points_cloud,np.column_stack([xyz,np.full(len(t),240)])])
    for center in [[13,23,0],[44,15,.3],[72,23,1.]]:
        xyz=rng.normal(size=(1300,3))*[1.8,1.8,2]+center
        xyz[:,2]=np.abs(xyz[:,2])+1
        points_cloud=np.vstack([points_cloud,np.column_stack([xyz,np.full(1300,90)])])
    pcd=destination/"pointcloud_map.pcd"
    header=("# Synthetic VMBQt training cloud. Not surveyed.\nVERSION .7\nFIELDS x y z intensity\n"
            f"SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(points_cloud)}\nHEIGHT 1\n"
            f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points_cloud)}\nDATA binary\n")
    with pcd.open("wb") as stream:stream.write(header.encode());stream.write(points_cloud.astype('<f4').tobytes())
    settings={"camera":{"target":[32,8,1],"scale":9,"yaw":0.,"elevation":.9,"is_3d":False,"perspective":False},
              "view":{"cloud_color":"intensity","intensity_max":255,"z_min":-1.,"z_max":10.},"lane_width":3.5,"lane_speed":30}
    save_project(destination/"demo.vmbqt",d,[{"kind":"cloud","source":str(pcd.resolve()),"visible":True}],settings)
    return d


if __name__=="__main__":
    target=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parent.parent/"examples"
    d=create(target)
    print(f"Created demo at {target}: {len(d.nodes)} nodes, {len(d.ways)} ways, {len(d.relations)} relations")
