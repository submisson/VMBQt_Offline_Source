from pathlib import Path
import struct
import numpy as np
import pytest

from vmbqt.model import Document,TrajectoryPoint,StopPoint
from vmbqt.exchange import read_csv,write_csv,save_project,load_project
from vmbqt.pointcloud import read_pcd,lzf_decompress,height_at


def header(n,mode,fields="x y z intensity",sizes="4 4 4 4",types="F F F F",counts="1 1 1 1"):
    return f"VERSION .7\nFIELDS {fields}\nSIZE {sizes}\nTYPE {types}\nCOUNT {counts}\nWIDTH {n}\nHEIGHT 1\nPOINTS {n}\nDATA {mode}\n".encode()


@pytest.mark.parametrize("mode",["ascii","binary","binary_compressed"])
def test_pcd_modes_and_field_layout(tmp_path,mode):
    p=tmp_path/f"{mode}.pcd"
    points=np.array([[1,2,3,10],[4,5,6,20],[7,8,9,30]],dtype='<f4')
    if mode=="ascii":body="\n".join(" ".join(map(str,row)) for row in points).encode()+b"\n"
    elif mode=="binary":body=points.tobytes()
    else:
        raw=points.T.copy().tobytes();compressed=b"".join(bytes([len(raw[i:i+32])-1])+raw[i:i+32] for i in range(0,len(raw),32))
        body=struct.pack('<II',len(compressed),len(raw))+compressed
    p.write_bytes(header(3,mode)+body);c=read_pcd(p)
    assert c.xyz==pytest.approx(points[:,:3]);assert c.intensity==pytest.approx(points[:,3])
    assert c.total==3 and height_at([c],[1.1,2.1],1)==3


def test_lzf_backreference_and_bounds():
    # "abc", then length=6 and offset=3 (overlapping reference is legal).
    assert lzf_decompress(bytes([2])+b"abc"+bytes([128,2]),9)==b"abcabcabc"
    with pytest.raises(ValueError):lzf_decompress(bytes([32,255]),3)
    with pytest.raises(ValueError):lzf_decompress(b"",100,50)


def test_pcd_rgb_integer_and_sample_limit(tmp_path):
    path=tmp_path/"rgb.pcd"
    dt=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
    a=np.zeros(100,dtype=dt);a['x']=np.arange(100);a['rgb']=0x123456
    path.write_bytes(header(100,'binary','x y z rgb','4 4 4 4','F F F U')+a.tobytes())
    c=read_pcd(path,max_points=9)
    assert len(c.xyz)<=9 and np.array_equal(c.rgb[0],[0x12,0x34,0x56])


def test_pcd_nan_filter_and_truncated_file(tmp_path):
    p=tmp_path/"nan.pcd";p.write_bytes(header(2,'ascii')+b"nan 1 2 3\n4 5 6 7\n")
    c=read_pcd(p);assert len(c.xyz)==1
    p.write_bytes(header(2,'binary')+b'123')
    with pytest.raises(ValueError):read_pcd(p)


def test_csv_preserves_unknown_columns(tmp_path):
    p=tmp_path/"t.csv";p.write_text('x,y,z,yaw,velocity,note\n1,2,3,0.5,4,hello\n')
    values=read_csv(p);assert values[0].extra=={'note':'hello'}
    out=tmp_path/'out.csv';write_csv(out,values)
    assert read_csv(out)[0]==values[0]
    s=tmp_path/'s.csv';write_csv(s,[StopPoint(5,1,2,3,.2,'Stop A',42)],'stop')
    assert read_csv(s,'stop')[0].lanelet_id==42


def test_project_relative_assets_and_roundtrip(tmp_path):
    d=Document();d.add_node([1,2,3]);d.stops[7]=StopPoint(7,1,2,3)
    d.trajectory=[TrajectoryPoint(9,1,2,3)]
    asset=tmp_path/'assets'/'a.pcd';asset.parent.mkdir();asset.touch()
    project=tmp_path/'map.vmbqt'
    save_project(project,d,[{'kind':'cloud','source':str(asset)}],{'lane_width':3.5})
    loaded,assets,settings=load_project(project)
    assert loaded.nodes==d.nodes and loaded.stops==d.stops and loaded.trajectory==d.trajectory
    assert assets[0]['source']==str(asset) and settings['lane_width']==3.5
