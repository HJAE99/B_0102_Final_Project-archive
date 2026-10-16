"""회피 구역 안 설비 cmd_vel 접근 (10/16, 기본 꺼짐) — ROS 없이 approach_twist 를 꺼내 차동 구동으로 가상 주행해 본다."""
import ast
import math
from pathlib import Path

scripts = Path(__file__).parents[1] / 'scripts' / 'mission_executor.py'
tree = ast.parse(scripts.read_text())
func = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'approach_twist')
consts = {}
for n in tree.body:   # CMD_LIN, CMD_ANG = 0.08, 0.4 · CMD_TOL = (...)
    if isinstance(n, ast.Assign):
        target = n.targets[0]
        if isinstance(target, ast.Tuple) and [e.id for e in target.elts] == ['CMD_LIN', 'CMD_ANG']:
            consts['CMD_LIN'], consts['CMD_ANG'] = ast.literal_eval(n.value)
        elif isinstance(target, ast.Name) and target.id == 'CMD_TOL':
            consts['CMD_TOL'] = ast.literal_eval(n.value)
namespace = {'math': math, **consts}
exec(compile(ast.fix_missing_locations(ast.Module(body=[func], type_ignores=[])), str(scripts), 'exec'), namespace)
approach_twist = namespace['approach_twist']


def drive(start, goal, dt=0.1, steps=600):
    """차동 구동 가상 주행: 명령을 dt 마다 적분. (도착 여부, 마지막 자세, 걸린 단계, 최대 선속도)"""
    x, y, yaw = start
    vmax = 0.0
    for k in range(steps):
        lin, ang, done = approach_twist((x, y, yaw), goal)
        if done:
            return True, (x, y, yaw), k, vmax
        vmax = max(vmax, lin)
        x += lin * math.cos(yaw) * dt
        y += lin * math.sin(yaw) * dt
        yaw = math.remainder(yaw + ang * dt, math.tau)
    return False, (x, y, yaw), steps, vmax


def test_reaches_goal_ahead():
    ok, (x, y, yaw), _, vmax = drive((0.0, 0.0, 0.0), (0.6, 0.0, 0.0))
    assert ok and math.dist((x, y), (0.6, 0.0)) <= consts['CMD_TOL'][0]
    assert vmax <= consts['CMD_LIN'] + 1e-9                       # 느리게만


def test_turns_first_when_goal_behind_and_ends_facing_goal_yaw():
    goal = (-0.5, 0.3, math.pi / 2)
    ok, (x, y, yaw), _, _ = drive((0.0, 0.0, 0.0), goal)
    assert ok and math.dist((x, y), goal[:2]) <= consts['CMD_TOL'][0]
    assert abs(math.remainder(yaw - goal[2], math.tau)) <= consts['CMD_TOL'][1]
    lin, ang, done = approach_twist((0.0, 0.0, 0.0), goal)
    assert lin == 0.0 and ang != 0.0 and not done                  # 크게 틀어졌으면 제자리 회전부터


def test_already_there_is_done():
    assert approach_twist((1.0, 1.0, 0.5), (1.0, 1.0, 0.5)) == (0.0, 0.0, True)


def test_off_by_default_both_sides():
    """기존 순찰 동작은 그대로 — 로봇 · 관제 둘 다 켜야만 동작"""
    assert "declare_parameter('keepout_cmd_vel', False)" in scripts.read_text()
    manager = Path(__file__).parents[2] / 'scv_manager' / 'scv_manager' / 'inspection_manager.py'
    assert 'declare_parameter("keepout_facility_cmd_vel", False)' in manager.read_text()


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('ok', name)
