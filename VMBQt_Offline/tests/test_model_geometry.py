import json
import math
import numpy as np
import pytest

from vmbqt.model import Document,History,Member
from vmbqt.projection import ProjectionConfig,Projector
from vmbqt import operations as op,geometry as g
from vmbqt.validation import validate,compare


def lane(d,start=0):
    return op.create_lane(d,[[start,0,0],[start+10,0,1],[start+20,0,2]],3.5,30)


def test_shared_boundary_and_node_identity():
    d=Document();a=lane(d);b=op.add_adjacent(d,a,"left",3.5)
    assert d.bounds(a)[0]==d.bounds(b)[1]
    shared=d.ways[d.bounds(a)[0]].nodes[1]
    op.transform(d,[("node",shared)],0,0,2)
    assert d.points(d.bounds(b)[1])[1,2]==3
    assert not [i for i in validate(d) if i.level=="ERROR"]


def test_transaction_rollback_undo_redo():
    d=Document();h=History(d);initial=d.snapshot()
    a=h.edit("lane",lambda:lane(d));assert h.dirty
    h.undo();assert d.snapshot()==initial
    h.redo();assert a in d.relations
    snapshot=d.snapshot()
    def fail():
        d.add_node([2,2,2]);raise ValueError("abort")
    with pytest.raises(ValueError):h.edit("bad",fail)
    assert d.snapshot()==snapshot


def test_edit_invalidates_external_validation_signature():
    d=Document();d.root_extras=['<validation name="old" />','<MapMetaInfo map_version="2" />']
    h=History(d);h.edit("point",lambda:d.add_node([0,0,0]))
    assert len(d.root_extras)==1 and "MapMetaInfo" in d.root_extras[0]


def test_id_precision_and_unknown_osm_roundtrip(tmp_path):
    p=tmp_path/"map.osm"
    p.write_text('''<osm version="0.6" custom="yes"><MapMetaInfo map_version="xyz"/>
    <node id="9007199254740993" lat="0" lon="0" custom="keep"><tag k="local_x" v="1.25"/><tag k="local_y" v="2.5"/><tag k="ele" v="3"/><tag k="private:tag" v="a&amp;b"/><extension a="x"/></node>
    <node id="2" lat="0" lon="0"><tag k="local_x" v="2"/><tag k="local_y" v="3"/></node>
    <way id="2"><nd ref="9007199254740993"/><nd ref="2"/><tag k="type" v="virtual"/></way>
    <relation id="2"><member type="way" ref="2" role="custom_role"/><tag k="type" v="private_relation"/></relation></osm>''')
    d=Document.read_osm(p);out=tmp_path/"out.osm";d.write_osm(out);r=Document.read_osm(out)
    assert 9007199254740993 in r.nodes
    assert r.nodes[9007199254740993].tags["private:tag"]=="a&b"
    assert r.nodes[9007199254740993].attrs["custom"]=="keep"
    assert r.relations[2].members[0].role=="custom_role"
    assert r.nodes[9007199254740993].extras
    assert r.root_attrs["custom"]=="yes" and "MapMetaInfo" in r.root_extras[0]


def test_missing_references_are_reported_not_discarded(tmp_path):
    path=tmp_path/"bad.osm";path.write_text('<osm version="0.6"><way id="1"><nd ref="25"/><nd ref="26"/></way></osm>')
    d=Document.read_osm(path)
    assert d.ways[1].nodes==[25,26]
    assert any(i.level=="ERROR" and "缺失" in i.message for i in validate(d))


def test_geographic_osm_requires_explicit_projection(tmp_path):
    path=tmp_path/"geo.osm";path.write_text('<osm><node id="1" lat="35" lon="139"/></osm>')
    with pytest.raises(ValueError):Document.read_osm(path)


@pytest.mark.parametrize("kind",["LocalCartesianUTM","TransverseMercator"])
def test_projection_roundtrip(kind):
    p=Projector(ProjectionConfig(kind,35.6762,139.6503))
    xy=p.forward(35.677,139.651)
    lat,lon=p.reverse(*xy)
    assert lat==pytest.approx(35.677,abs=1e-10)
    assert lon==pytest.approx(139.651,abs=1e-10)
    assert p.forward(35.6762,139.6503)==pytest.approx((0,0),abs=1e-6)


def test_mgrs_tokyo_grid_and_bounds():
    p=Projector(ProjectionConfig("MGRS",mgrs_grid="54SUE"))
    xy=p.forward(35.6762,139.6503)
    assert 0<xy[0]<100000 and 0<xy[1]<100000
    assert p.reverse(*xy)==pytest.approx((35.6762,139.6503),abs=1e-9)
    assert p._mgrs_origin("54SUE")== (54,False,300000.,3900000.)
    with pytest.raises(ValueError):p.reverse(-20,10)
    with pytest.raises(ValueError):Projector(ProjectionConfig("MGRS",mgrs_grid="54SAA"))


def test_datum_is_not_silently_converted():
    with pytest.raises(ValueError):Projector(ProjectionConfig("MGRS",vertical_datum="EGM2008"))


def test_standard_utm_zone_exceptions():
    assert Projector.utm_zone(60,4)==32
    assert Projector.utm_zone(78,10)==33
    assert Projector.utm_zone(78,23)==35
    assert Projector.utm_zone(35,139)==54


def test_split_join_preserves_shared_endpoint_identity():
    d=Document();a=lane(d);b=op.split_lane(d,a,.5)
    aa,ab=op.bound_nodes(d,a);ba,bb=op.bound_nodes(d,b)
    assert aa[-1]==ba[0] and ab[-1]==bb[0]
    assert op.route(d,a,b)==[a,b]
    op.join_lanes(d,a,b)
    assert b not in d.relations
    assert not [i for i in validate(d) if i.level=="ERROR"]


def test_split_shared_boundary_is_atomic_on_failure():
    d=Document();a=lane(d);op.add_adjacent(d,a)
    h=History(d);before=d.snapshot()
    with pytest.raises(ValueError):h.edit("split",lambda:op.split_lane(d,a))
    assert d.snapshot()==before


def test_connect_uses_ids_and_distance_limit():
    d=Document();a=lane(d);b=lane(d,20)
    op.connect_lanes(d,a,b,3)
    assert op.route(d,a,b)==[a,b]
    c=lane(d,60)
    with pytest.raises(ValueError):op.connect_lanes(d,b,c,.1)


def test_connector_endpoints_and_direction():
    d=Document();a=op.create_lane(d,[[0,0,0],[10,0,0]])
    b=op.create_lane(d,[[20,10,1],[20,20,2]])
    c=op.create_connector(d,a,b,.5)
    assert op.route(d,a,b)==[a,c,b]
    assert d.relations[c].tags["turn_direction"]=="left"
    assert not [i for i in validate(d) if i.level=="ERROR"]


def test_delete_reference_protection():
    d=Document();a=lane(d);wid=d.bounds(a)[0]
    with pytest.raises(ValueError):op.delete_objects(d,[("way",wid)])
    op.delete_objects(d,[("relation",a),("way",wid)])
    assert wid not in d.ways


def test_insert_and_split_standalone_way():
    d=Document();w=d.add_way([[0,0,0],[10,0,10]],op.LINE_TAGS)
    nid=op.insert_point(d,w,[3,2,0]);assert d.nodes[nid].xyz==pytest.approx([3,0,3])
    new=op.split_way(d,w,nid)
    assert d.ways[w].nodes[-1]==d.ways[new].nodes[0]
    op.join_ways(d,w,new)
    assert len(d.ways[w].nodes)==3


def test_traffic_light_members_and_clone_id_remapping():
    d=Document();a=lane(d);rid=op.stop_line_at(d,a,[10,0,1],"traffic_light")
    r=d.relations[rid];roles={m.role:m.ref for m in r.members}
    assert set(roles)=={"refers","ref_line","light_bulbs"}
    assert d.ways[roles["light_bulbs"]].tags["traffic_light_id"]==str(roles["refers"])
    cloned=op.clone(d,[("relation",a)])[0][1]
    rr=next(m.ref for m in d.relations[cloned].members if m.role=="regulatory_element")
    roles2={m.role:m.ref for m in d.relations[rr].members}
    assert roles2["refers"]!=roles["refers"]
    assert d.ways[roles2["light_bulbs"]].tags["traffic_light_id"]==str(roles2["refers"])


def test_merge_remaps_each_primitive_namespace():
    d=Document();a=lane(d);other=Document();b=lane(other)
    mapping=op.merge_document(d,other)
    assert len(d.lanelets())==2
    assert mapping[("relation",b)]!=a
    assert not [i for i in validate(d) if i.level=="ERROR"]


def test_clone_cycles_terminate():
    d=Document();a=lane(d);b=lane(d,25)
    rid=op.add_rule(d,[a,b],"right_of_way",[("relation",a,"right_of_way"),("relation",b,"yield")])
    result=op.clone(d,[("relation",a)])
    assert result[0][1]!=a and len(d.lanelets())==3
    # The other lane is an external rule reference, not owned geometry.
    refs=d.descendants(result,follow_related_lanes=True)
    assert ("relation",b) in refs


def test_right_of_way_does_not_move_unselected_road():
    d=Document();a=lane(d);b=lane(d,25)
    op.add_rule(d,[a,b],"right_of_way",[("relation",a,"right_of_way"),("relation",b,"yield")])
    before=d.points(d.bounds(b)[0]).copy()
    op.transform(d,[("relation",a)],0,0,3)
    np.testing.assert_allclose(d.points(d.bounds(b)[0]),before)


def test_restore_preserves_allocated_id_cursor_after_deletion():
    d=Document();a=d.add_node([0,0,0]);b=d.add_node([1,1,1]);del d.nodes[b]
    h=History(d);h.edit("new",lambda:d.add_node([2,2,2]));h.undo()
    assert not h.dirty and d.new_id()==b+1


def test_geometry_offset_and_resample():
    p=np.array([[0,0,0],[10,0,10]])
    np.testing.assert_allclose(g.offset(p,2),[[0,2,0],[10,2,10]])
    assert g.sample(p,3)[1]==pytest.approx([5,0,5])
    with pytest.raises(ValueError):g.offset([[0,0,0],[0,0,1]],1)
    assert g.self_intersects([[0,0,0],[2,2,0],[0,2,0],[2,0,0]],True)


def test_diff_by_type_and_id():
    d=Document();lane(d);other=Document();other.restore(d.state())
    i=next(iter(other.nodes));other.nodes[i].z+=1
    assert compare(d,other)==[("修改",("node",i))]


@pytest.mark.parametrize("latitude,longitude",[(35.,139.),(60.,4.),(78.,10.)])
def test_true_lanelet2_reader(tmp_path,latitude,longitude):
    ll=pytest.importorskip("lanelet2",exc_type=ImportError)
    d=Document();d.projection=ProjectionConfig("LocalCartesianUTM",latitude,longitude)
    a=lane(d);op.add_adjacent(d,a);op.generate_centerline(d,a)
    p=tmp_path/"interop.osm";d.write_osm(p)
    projector=ll.projection.UtmProjector(ll.io.Origin(latitude,longitude))
    loaded,errors=ll.io.loadRobust(str(p),projector)
    assert not errors
    assert len(loaded.laneletLayer)==2
    assert loaded.laneletLayer[a].leftBound.id==d.bounds(a)[0]
    for n in d.nodes.values():
        p=loaded.pointLayer[n.id]
        assert [p.x,p.y,p.z]==pytest.approx([n.x,n.y,n.z],abs=1e-5)


def test_mgrs_with_independent_geotrans_oracle():
    mgrs=pytest.importorskip("mgrs",exc_type=ImportError)
    import re
    m=mgrs.MGRS()
    for latitude,longitude in [(35.6762,139.6503),(51.5074,-.1278),(40.7128,-74.006),
                               (-33.8688,151.2093),(39.9,116.4),(60,4),(78,10),(-.5,30.5)]:
        text=m.toMGRS(latitude,longitude,MGRSPrecision=5)
        match=re.fullmatch(r"(\d{1,2}[A-Z]{3})(\d{5})(\d{5})",text)
        assert match
        projector=Projector(ProjectionConfig("MGRS",mgrs_grid=match[1]))
        x,y=projector.forward(latitude,longitude)
        assert abs(x-int(match[2]))<1.01 and abs(y-int(match[3]))<1.01
