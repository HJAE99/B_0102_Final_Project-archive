"""Undock calibration defaults; reading this module never starts ROS or motion."""
import json
import math
import os
from pathlib import Path


def reference_path():
    override = os.environ.get('SCV_ROBOT_REFERENCE_CONFIG')
    if override:
        return Path(override)
    source = Path(__file__).resolve().parents[1] / 'config/robot_references.json'
    if source.exists():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('scv_nav')) / 'config/robot_references.json'


def read_references():
    data = json.loads(reference_path().read_text(encoding='utf-8'))
    if not data.get('robots') or not data.get('map'):
        raise ValueError('map/robots reference configuration is missing')
    for rid, p in data['robots'].items():
        if not rid or not all(math.isfinite(float(p[k])) for k in ('x', 'y', 'yaw_deg')):
            raise ValueError('invalid robot reference pose')
    return data


def home_defaults():
    return {'/' + rid: tuple(float(p[k]) for k in ('x', 'y', 'yaw_deg'))
            for rid, p in read_references()['robots'].items()}
