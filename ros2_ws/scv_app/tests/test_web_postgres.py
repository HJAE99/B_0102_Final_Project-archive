"""Final API checks against a disposable test schema; app lifespan/ROS never start."""
import os
import sys
import uuid
from pathlib import Path

import pytest

APP=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(APP),str(APP.parent/'src/scv_monitor'),str(APP.parent/'src/scv_nav')]


@pytest.fixture(scope='module')
def web_db(tmp_path_factory):
    dsn=os.environ.get('SCV_TEST_DB_DSN')
    if not dsn: pytest.skip('SCV_TEST_DB_DSN not set')
    import psycopg
    from psycopg import sql
    from psycopg.conninfo import conninfo_to_dict,make_conninfo
    from psycopg.rows import dict_row
    from psycopg_pool import ConnectionPool
    from fastapi.testclient import TestClient
    import server_page as server
    if not conninfo_to_dict(dsn).get('dbname','').endswith('_test'):
        pytest.fail('Only an explicitly named *_test DB is accepted')
    schema='scv_web_test_'+uuid.uuid4().hex
    with psycopg.connect(dsn) as c:
        c.execute(sql.SQL('CREATE SCHEMA {}').format(sql.Identifier(schema)))
    isolated=make_conninfo(dsn,options=f'-c search_path={schema},public')
    media=tmp_path_factory.mktemp('web-media');(media/'maps').mkdir()
    from PIL import Image
    Image.open(APP.parent/'src/scv_nav/maps/testbed_v1.pgm').convert('L').save(media/'maps/m.png')
    pool=ConnectionPool(isolated,min_size=1,max_size=3,kwargs={'row_factory':dict_row})
    with pool.connection() as c:
        c.execute((APP.parent/'scv_db/sql/01_schema.sql').read_text())
        c.execute("INSERT INTO robots(robot_id) VALUES ('robot3'),('robot4')")
        mid=c.execute("INSERT INTO maps(name,robot_map_path,image_key,resolution_m,origin_x,origin_y,grid_size_m,is_active) VALUES ('web-test','/test/m.yaml','maps/m.png',.05,-.15,-.25,.5,true) RETURNING map_id").fetchone()['map_id']
        cid=c.execute('INSERT INTO grid_cells(map_id,ix,iy,center_x,center_y) VALUES (%s,1,1,.6,.5) RETURNING cell_id',(mid,)).fetchone()['cell_id']
        ap=c.execute("INSERT INTO aps(map_id,ssid,is_target) VALUES (%s,'test-ap',true) RETURNING ap_id",(mid,)).fetchone()['ap_id']
        run=c.execute("INSERT INTO runs(map_id,run_type,sector_robots) VALUES (%s,'patrol','{\"1\":\"robot4\"}') RETURNING run_id",(mid,)).fetchone()['run_id']
        for seq,v,ok,det in [(1,-80,True,True),(2,-60,True,True),(3,0,True,False),(4,-20,False,True)]:
            c.execute("INSERT INTO net_samples(robot_id,run_id,seq,stamp,x,y,yaw,pose_ok,ap_id,rssi_dbm,cell_id,detected) VALUES ('robot3',%s,%s,now(),.6,.5,0,%s,%s,%s,%s,%s)",(run,seq,ok,ap,v,cid,det))
    oldpool,oldmedia,oldowner,olduser=server.pool,server.MEDIA_DIR,server.RUN_OWNER,server.current_user
    server.pool,server.MEDIA_DIR,server.RUN_OWNER=pool,media,'ros'
    server.current_user=lambda req:dict(role='admin',username='admin',user_id=1)
    # Not a context manager: TestClient requests do not execute lifespan (or start subscriptions).
    client=TestClient(server.app)
    yield server,client,pool,mid,cid,ap,run
    client.close();pool.close()
    server.pool,server.MEDIA_DIR,server.RUN_OWNER,server.current_user=oldpool,oldmedia,oldowner,olduser
    # Keep the isolated schema for diagnosis; no operational rows are changed/deleted.


def test_active_map_matches_shared_calibration_with_pg_real_precision(web_db):
    _,client,_,mid,*_=web_db
    d=client.get('/api/maps/active').json()
    assert d['run_owner']=='ros' and d['map']['map_id']==mid
    assert d['map']['configuration']['status']=='ok'
    assert len(d['map']['configuration']['references'])==2
    assert client.get(f'/api/maps/{mid}').json()['map']['map_id']==mid


def test_real_db_median_does_not_include_undetected_or_bad_pose(web_db):
    _,client,_,_,_,_,run=web_db
    response=client.get(f'/api/runs/{run}');assert response.status_code==200,response.text
    r=response.json()['readings'][0]
    assert r['n']==2 and r['misses']==1 and r['median']==-70 and r['confidence']=='low'
    assert r['statistic']=='live_median'
    assert client.get('/api/runs').json()['current']==run


def test_committed_value_replaces_live_value_without_losing_misses(web_db):
    _,client,pool,_,cid,ap,run=web_db
    with pool.connection() as c:
        c.execute("INSERT INTO grid_stats(run_id,cell_id,ap_id,n,median,confidence,state) VALUES (%s,%s,%s,30,-65,'ok','normal')",(run,cid,ap))
    r=client.get(f'/api/runs/{run}').json()['readings'][0]
    assert r['median']==-65 and r['n']==30 and r['misses']==1 and r['statistic']=='committed_median'


def test_ros_agent_commands_are_still_blocked(web_db):
    _,client,*_=web_db
    assert client.post('/api/agent/samples',json={}).status_code==409


def test_reference_import_reads_local_files_without_browser_upload(web_db):
    _,client,pool,*_=web_db
    assert client.get('/api/reference-map').json()['grid_size_m']==.5
    assert client.post('/api/maps/import-reference',json={}).status_code==400
    r=client.post('/api/maps/import-reference',json={'robot_map_path':'/test/reference.yaml'})
    assert r.status_code==200,r.text
    with pool.connection() as c:
        m=c.execute('SELECT grid_size_m,resolution_m,is_active FROM maps WHERE map_id=%s',(r.json()['map_id'],)).fetchone()
    assert m['grid_size_m']==.5 and not m['is_active']


def test_live_photo_endpoint_is_separate_from_status_payload(web_db,monkeypatch):
    server,client,*_=web_db
    from scv_monitor.telemetry import Telemetry
    cache=Telemetry();cache.on_inspection(dict(run_id=7,robot_id='r',facility_id='f',photo='data:image/jpeg;base64,aGVsbG8='))
    monkeypatch.setattr(server,'TELEMETRY',cache)
    assert client.get('/api/telemetry/inspection-image?run_id=7&robot_id=r&facility_id=absent').status_code==404
    r=client.get('/api/telemetry/inspection-image?run_id=7&robot_id=r&facility_id=f')
    assert r.content==b'hello' and r.headers['cache-control']=='no-store'


def test_duplicate_import_failure_keeps_the_existing_image(web_db):
    server,client,pool,*_=web_db
    body={'robot_map_path':'/test/duplicate-reference.yaml'}
    first=client.post('/api/maps/import-reference',json=body)
    assert first.status_code==200,first.text
    with pool.connection() as c:
        row=c.execute('SELECT image_key FROM maps WHERE map_id=%s',(first.json()['map_id'],)).fetchone()
    original=server.MEDIA_DIR/row['image_key']
    assert original.exists()
    assert client.post('/api/maps/import-reference',json=body).status_code==409
    assert original.exists(),'Failed uploads must only remove their own temporary image'
