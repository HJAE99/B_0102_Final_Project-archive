#!/usr/bin/env python3
"""Isolated UI preview: synthetic samples, no DB pool, ROS, or hardware imports.

PYTHONPATH=<web dependencies> python3 preview_web.py  # http://127.0.0.1:8765
"""
import asyncio
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from PIL import Image
from io import BytesIO

from web_config import SRC, map_configuration

BASE = Path(__file__).resolve().parent
app = FastAPI(title='SCV sample-data preview')
app.mount('/static', StaticFiles(directory=BASE / 'static'), name='static')
image = Image.open(SRC / 'scv_nav/maps/testbed_v1.pgm').convert('L')
buffer = BytesIO(); image.save(buffer, format='PNG'); png = buffer.getvalue()
res, ox, oy, g = .05, -.15, -.25, .5
raw = np.asarray(image)
ys, xs = np.nonzero((raw > 205))
cells = sorted(set(zip(np.floor((xs+.5)*res/g).astype(int), np.floor((image.height-ys-.5)*res/g).astype(int))))
cells = [[i+1, int(x), int(y), 1 if oy+(y+.5)*g < 3.29 else 2] for i,(x,y) in enumerate(cells)]
MAP = dict(map_id=1, name='testbed_v1', image_url='/preview-map.png', image_key='preview-map.png',
           resolution_m=res, origin_x=ox, origin_y=oy, width=image.width, height=image.height,
           grid_size_m=g, cell_count=len(cells), cells=cells, running_run=21, dock_poses={},
           aps=[dict(ap_id=1,ssid='SCV_TEST_AP',is_target=True,x=1.6,y=3.4),
                dict(ap_id=2,ssid='SCV_SECOND_AP',is_target=False,x=.8,y=1.0)],
           facilities=[dict(facility_id='SHT-01',facility_type='shutter',expected_x=3.1,expected_y=3.,is_active=True),
                       dict(facility_id='EXT-DEMO',facility_type='extinguisher',expected_x=.35,expected_y=5.3,is_active=True)])
MAP['configuration'] = map_configuration(MAP, SRC / 'scv_nav/maps/testbed_v1.pgm')
READINGS = []
for cid, ix, iy, _ in cells:
    if (ix,iy) in [(1,1),(1,2),(2,1)]: continue
    for ap in (1,2):
        v = round(max(-90,min(-40, -44 - (iy-7)**2*.55 - (ix-3)**2*.8 - (ap-1)*8)),1)
        miss = ix == 1 and iy in (10,11)
        READINGS.append(dict(cell_id=cid,ap_id=ap,n=0 if miss else 12,misses=5 if miss else 0,
             median=None if miss else v,mean=None if miss else v-1,confidence='ok',
             state='unknown' if miss else 'coverage_suspect' if v < -72 else 'normal',interpolated=False,
             detection='not_detected' if miss else 'detected',statistic='committed_median'))


def stamp(): return datetime.now(timezone.utc).isoformat()


def run_detail():
    return dict(run=dict(run_id=21,map_id=1,map_name='testbed_v1',status='running',sector_robots={'1':'robot4','2':'robot3'},
                started_at=stamp(),ended_at=None,run_type='patrol'),aps=MAP['aps'],readings=READINGS,
                heat={},tasks=[],candidates=[],inspections=[],samples=2168,thr=-72,current=21,fetched_at=stamp())


@app.get('/server/{page}')
def page(page: str): return FileResponse(BASE / 'static/server/index.html')


@app.get('/preview-map.png')
def map_image(): return Response(png,media_type='image/png')


@app.get('/api/me')
def me(): return {'user':{'username':'미리보기', 'role':'admin'}}


@app.get('/api/maps/active')
@app.get('/api/maps/{map_id:int}')
def active_map(map_id: int=1): return {'map':MAP, 'run_owner':'ros'}


@app.get('/api/runs')
def runs(): return {'runs':[run_detail()['run']], 'current':21,'map':{'map_id':1,'name':MAP['name']}}


@app.get('/api/runs/{run_id}')
def detail(run_id: int): return run_detail()


@app.get('/api/runs/{run_id}/events')
def events(run_id: int): return {'events':[]}


@app.get('/api/master/summary')
def summary(): return {'cards':[]}


@app.websocket('/ws/ui')
async def socket(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            robots=[]
            for rid,x,y,yaw,battery,state,progress in [('robot3',1.1,4.8,-1.57,78,'작업 중',.63),('robot4',2.6,1.4,0,62,'복귀·도킹 중',.85)]:
                s=dict(x=x,y=y,yaw=yaw,pose_ok=True,battery_pct=battery,nav_state=state,progress=progress,
                       run_id=21,task_id='preview-'+rid,task_type=0 if rid=='robot3' else 3,ts=stamp(),received_at=stamp())
                robots.append(dict(robot_id=rid,connected=True,is_active=True,state=s))
            await ws.send_json(dict(type='robots',preview=True,ts=stamp(),run_owner='ros',run_id=21,robots=robots,
                events=[],telemetry=dict(bridge=dict(ready=True,error=None),inspections=[],alerts=[
                    dict(level='warn',robot_id='robot3',code='CANDIDATE',ref='demo',message='일부 영역에서 신호 저하가 관측되었습니다. 재측정 결과를 확인하세요.',received_at=stamp(),delayed=False)])))
            await asyncio.sleep(1)
    except Exception:
        pass


if __name__ == '__main__':
    uvicorn.run(app,host='127.0.0.1',port=8765)
