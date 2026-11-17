import numpy as np
from PySide6.QtCore import Qt,QPoint
from PySide6.QtTest import QTest
from PySide6.QtGui import QImage
from vmbqt import operations as op
from vmbqt.pointcloud import Cloud


def test_actual_click_draw_and_undo(window,qapp):
    c=window.canvas;c.set_tool('lane')
    QTest.mouseClick(c,Qt.MouseButton.LeftButton,pos=QPoint(200,200))
    QTest.mouseClick(c,Qt.MouseButton.LeftButton,pos=QPoint(430,210))
    QTest.keyClick(c,Qt.Key.Key_Return);qapp.processEvents()
    assert len(window.document.lanelets())==1
    window.undo();assert not window.document.lanelets()
    window.redo();assert len(window.document.lanelets())==1


def test_property_apply_shared_point_and_render(window,qapp,tmp_path):
    window.on_drawn('lane',[([0,0,0],None),([20,0,0],None)])
    d=window.document;a=d.lanelets()[0].id;b=op.add_adjacent(d,a)
    nid=d.ways[d.bounds(a)[0]].nodes[0]
    window.refresh();window.canvas.select([('node',nid)]);window.coords['z'].setValue(4)
    window.apply_inspector();assert d.nodes[nid].z==4
    assert d.points(d.bounds(b)[1])[0,2]==4
    window.canvas.fit_all();window.set_3d(True);window.canvas.camera.perspective=True
    qapp.processEvents();assert window.canvas.grab().save(str(tmp_path/'view.png'))
    image=QImage(str(tmp_path/'view.png'));assert image.width()>400


def test_depth_raster_cloud_and_image_background(window,qapp,tmp_path):
    c=window.canvas
    rng=np.random.default_rng(1);xyz=rng.normal(size=(1500,3))*5
    c.clouds=[Cloud('synthetic',xyz,np.full(len(xyz),128,dtype=float),None,len(xyz))]
    image=QImage(10,10,QImage.Format.Format_RGB32);image.fill(Qt.GlobalColor.darkGray)
    c.image=image;c.image_spec={'x':-10.,'y':-10.,'z':-3.,'width':20.,'height':20.}
    c.fit_all();c.camera.is_3d=True;c.camera.perspective=True
    qapp.processEvents();assert c.grab().save(str(tmp_path/'cloud.png'))


def test_camera_inverse_roundtrip_in_both_modes(window):
    cam=window.canvas.camera;point=np.array([4.,8.,3.])
    for is3d,perspective in [(False,False),(True,False),(True,True)]:
        cam.is_3d=is3d;cam.perspective=perspective;cam.yaw=.7
        screen,depth=cam.project([point]);restored=cam.unproject(screen[0],point[2])
        np.testing.assert_allclose(restored,point,atol=1e-10)


def test_gui_rules_check_and_serialization(window,tmp_path):
    window.on_drawn('lane',[([0,0,0],None),([20,0,0],None)])
    a=window.document.lanelets()[0].id
    window.canvas.select([('relation',a)])
    window.on_drawn('trafficlight',[([10,0,0],None)])
    assert any(r.tags.get('subtype')=='traffic_light' for r in window.document.relations.values())
    issues=window.run_checks();assert not [i for i in issues if i.level=='ERROR']
    window.document.write_osm(tmp_path/'map.osm')
