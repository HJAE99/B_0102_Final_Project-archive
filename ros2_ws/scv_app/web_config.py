"""Read shared calibration and zones without importing a ROS executor."""
import hashlib
import importlib.util
import math
import os
from pathlib import Path

import yaml
from PIL import Image

SRC = Path(__file__).resolve().parents[1] / 'src'
try:
    from scv_nav.references import read_references
except ImportError:
    spec = importlib.util.spec_from_file_location('scv_web_references', SRC / 'scv_nav/scv_nav/references.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    read_references = module.read_references


def map_configuration(m, image_path):
    """Fail closed on a different map: metadata AND original image pixels must match."""
    try:
        cfg = read_references()
        expected = cfg['map']
        with Image.open(image_path) as im:
            image = im.convert('L')
            matches = (image.size == (expected['width'], expected['height'])
                       and hashlib.sha256(image.tobytes()).hexdigest() == expected['pixels_sha256']
                       and math.isclose(float(m['resolution_m']), expected['resolution_m'], abs_tol=1e-6)
                       and math.isclose(float(m['origin_x']), expected['origin'][0], abs_tol=1e-6)
                       and math.isclose(float(m['origin_y']), expected['origin'][1], abs_tol=1e-6)
                       and expected['origin'][2] == 0)
        if not matches:
            return {'status': 'map_mismatch', 'references': [], 'zones': [],
                    'message': '시작 참고 설정과 지도 원본이 다릅니다. 참고점·구역 표시를 생략합니다.'}
        path = Path(os.environ.get('SCV_ZONES_CONFIG', SRC / 'scv_bringup/config/zones.yaml'))
        zones = yaml.safe_load(path.read_text(encoding='utf-8'))['zones']
        refs = [dict(robot_id=rid, x=float(p['x']), y=float(p['y']),
                     yaw=math.radians(float(p['yaw_deg'])), source='configured_default',
                     role='undock_reference') for rid, p in cfg['robots'].items()]
        return {'status': 'ok', 'source': 'robot_references.json / zones.yaml',
                'references': refs, 'zones': zones,
                'message': '기본 시작 참고값입니다. 실행 시 지정한 값·현재 위치와 다를 수 있습니다.'}
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        return {'status': 'unavailable', 'references': [], 'zones': [],
                'message': '공통 설정을 읽지 못했습니다: ' + type(exc).__name__}


def reference_map_files():
    """Use repository/installed canonical files; no client-supplied filesystem paths."""
    map_path = Path(os.environ.get('SCV_REFERENCE_MAP_YAML', SRC / 'scv_nav/maps/testbed_v1.yaml')).resolve()
    meta = yaml.safe_load(map_path.read_text(encoding='utf-8'))
    image_path = (map_path.parent / meta['image']).resolve()
    zone_path = Path(os.environ.get('SCV_ZONES_CONFIG', SRC / 'scv_bringup/config/zones.yaml'))
    zones = yaml.safe_load(zone_path.read_text(encoding='utf-8'))['zones']
    sizes = {float(z['spacing']) for z in zones}
    if len(sizes) != 1 or next(iter(sizes)) <= 0:
        raise ValueError('구역별 측정 간격이 다르거나 잘못되었습니다. 지도별 격자 크기를 직접 확인하세요.')
    return map_path, image_path, next(iter(sizes)), meta
