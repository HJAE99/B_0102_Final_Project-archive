"""Focused data/adapter tests; no ROS executor, operational DB or robot actions."""
import ast
import json
import math
import sys
from pathlib import Path

import pytest
from PIL import Image

APP = Path(__file__).resolve().parents[1]
ROOT = APP.parent
sys.path[:0] = [str(APP), str(ROOT / 'src/scv_nav'), str(ROOT / 'src/scv_monitor')]
from scv_nav.references import home_defaults, read_references
from scv_monitor.telemetry import Telemetry
from heatmap_data import merge_readings
from web_config import map_configuration, reference_map_files


def test_shared_calibration_and_units():
    assert home_defaults() == {'/robot3': (2.806,4.610,179.9), '/robot4': (2.863,.948,175.8)}
    m = dict(resolution_m=.050000000745, origin_x=-.15000000596, origin_y=-.25)
    cfg = map_configuration(m, ROOT / 'src/scv_nav/maps/testbed_v1.pgm')
    assert cfg['status'] == 'ok'
    assert cfg['references'][0]['yaw'] == pytest.approx(math.radians(179.9))
    assert [z['robot'] for z in cfg['zones']] == ['robot4','robot3']


def test_config_override_and_missing_file(tmp_path, monkeypatch):
    data = read_references(); data['robots']['robot3']['x'] = 1.25
    path = tmp_path / 'config.json'; path.write_text(json.dumps(data))
    monkeypatch.setenv('SCV_ROBOT_REFERENCE_CONFIG', str(path))
    assert home_defaults()['/robot3'][0] == 1.25
    monkeypatch.setenv('SCV_ROBOT_REFERENCE_CONFIG', str(tmp_path / 'missing'))
    assert map_configuration({}, 'missing')['status'] == 'unavailable'


def test_same_dimensions_with_different_pixels_do_not_match(tmp_path):
    image = Image.open(ROOT / 'src/scv_nav/maps/testbed_v1.pgm').convert('L')
    image.putpixel((30,30), 0 if image.getpixel((30,30)) else 255)
    path=tmp_path / 'different.png'; image.save(path)
    cfg=map_configuration(dict(resolution_m=.05,origin_x=-.15,origin_y=-.25),path)
    assert cfg['status']=='map_mismatch' and not cfg['references']


def test_reference_map_spacing_comes_from_zones():
    path, image, size, meta = reference_map_files()
    assert path.is_file() and image.is_file() and size == .5 and meta['resolution'] == .05


def test_valid_zero_pose_missing_battery_and_disconnect():
    now=[10.]; c=Telemetry(lambda: now[0])
    assert c.snapshot(['robot3'])['robots'][0]['state'] is None
    c.on_pose('robot3'); c.on_status('robot3',dict(x=0.,y=0.,yaw=0.,pose_ok=True))
    s=c.snapshot(['robot3'])['robots'][0]['state']
    assert s['pose_ok'] and s['x']==0 and s['battery_pct'] is None
    c.on_battery('robot3',.75); assert c.snapshot(['robot3'])['robots'][0]['state']['battery_pct']==75
    c.on_battery('robot3',float('nan')); c.on_battery('robot3',-1)
    now[0]+=6
    r=c.snapshot(['robot3'])['robots'][0]
    assert not r['connected'] and r['state']['x'] is None and r['state']['battery_pct'] is None
    assert r['state']['last_valid_pose']['x']==0


def test_invalid_pose_never_overwrites_last_valid():
    c=Telemetry();c.on_pose('r'); c.on_status('r',dict(x=1.,y=2.,yaw=0.,pose_ok=True))
    c.on_status('r',dict(x=0.,y=0.,yaw=0.,pose_ok=False))
    s=c.snapshot(['r'])['robots'][0]['state'];assert not s['pose_ok'] and s['last_valid_pose']['x']==1


def test_cache_bounds():
    c=Telemetry()
    for i in range(110):
        c.on_alert(dict(message=str(i)))
        c.on_inspection(dict(run_id=i,robot_id='r',facility_id='f'))
    s=c.snapshot([]);assert len(s['alerts'])==100 and len(s['inspections'])==20


def test_live_and_committed_readings_are_not_confused():
    s=[dict(cell_id=1,ap_id=2,n=0,misses=3,median=0,mean=None),
       dict(cell_id=2,ap_id=2,n=4,misses=1,median=-60,mean=-62)]
    r=merge_readings(s,[dict(cell_id=2,ap_id=2,n=10,median=-65,mean=-64,confidence='ok',state='normal',interpolated=False)])
    assert r[0]['median'] is None and r[0]['detection']=='not_detected'
    assert r[1]['median']==-65 and r[1]['misses']==1 and r[1]['statistic']=='committed_median'


def test_reader_has_no_motion_publishers_or_clients():
    tree=ast.parse((ROOT/'src/scv_monitor/scv_monitor/ros_reader.py').read_text())
    forbidden={'create_publisher','create_client','ActionClient','publish','undock','dock','navigateToPose','setInitialPose'}
    calls={n.func.attr if isinstance(n.func,ast.Attribute) else n.func.id if isinstance(n.func,ast.Name) else ''
           for n in ast.walk(tree) if isinstance(n,ast.Call)}
    assert not calls & forbidden


def test_navigation_paths_are_bounded_fresh_and_map_frame_only():
    now=[10.];c=Telemetry(lambda:now[0]);c.on_pose('r')
    c.on_status('r',dict(x=1.,y=2.,yaw=0.,pose_ok=True,run_id=5))
    points=[dict(x=float(i),y=0.) for i in range(2000)]
    c.on_path('r','odom',points);assert not c.snapshot([])['paths']
    c.on_path('r','map',points)
    path=c.snapshot([])['paths'][0]
    assert path['run_id']==5 and len(path['points'])<=501 and path['points'][-1]==points[-1]
    now[0]+=6;assert not c.snapshot([])['paths']


def test_live_photos_do_not_repeat_in_websocket_payload():
    c=Telemetry();photo='data:image/jpeg;base64,aGVsbG8='
    c.on_inspection(dict(run_id=7,robot_id='r',facility_id='f',photo=photo))
    i=c.snapshot([])['inspections'][0]
    assert 'photo' not in i and i['photo_url'].startswith('/api/telemetry/inspection-image?')
    assert c.inspection_photo(7,'r','f')==photo


def test_executor_keeps_full_override_priority_without_loading_defaults():
    from types import SimpleNamespace
    tree=ast.parse((ROOT/'src/scv_nav/scripts/mission_executor.py').read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    selection=next(n for n in main.body if isinstance(n,ast.If))
    code=compile(ast.fix_missing_locations(ast.Module(body=[selection],type_ignores=[])),'startup-selection','exec')
    def must_not_read(): raise AssertionError('An explicit pose must not depend on a default file')
    env=dict(math=math,xyyaw=[9.,8.,7.],home_defaults=must_not_read)
    exec(code,env);assert env['xyyaw']==[9.,8.,7.]
    env.update(xyyaw=[9.,math.nan,7.],home_defaults=home_defaults,nav=SimpleNamespace(get_namespace=lambda:'/robot3'))
    exec(code,env);assert env['xyyaw']==(2.806,4.610,179.9)
    env.update(xyyaw=[math.nan]*3,nav=SimpleNamespace(get_namespace=lambda:'/unregistered'))
    with pytest.raises(SystemExit):exec(code,env)
