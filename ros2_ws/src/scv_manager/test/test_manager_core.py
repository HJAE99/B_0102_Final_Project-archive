from scv_manager.manager_core import (
    ManagerCore,
    TaskState,
)


def test_task_success_flow():
    core = ManagerCore(["robot4", "robot3"])

    core.create_task(
        task_id="task-001",
        run_id=1,
        task_type=0,
        priority=1,
        target_ref="Z1",
    )

    task = core.tasks["task-001"]
    assert task.state == TaskState.PENDING

    core.assign_task("task-001", "robot4")
    assert task.state == TaskState.ASSIGNED
    assert task.robot_id == "robot4"

    core.start_task("task-001")
    assert task.state == TaskState.RUNNING

    core.update_task_progress("task-001", 0.5)
    assert task.progress == 0.5

    core.finish_task(
        "task-001",
        ok=True,
        reason="",
    )

    assert task.state == TaskState.SUCCEEDED
    assert task.progress == 1.0
    assert task.reason == ""


def test_task_failure_flow():
    core = ManagerCore(["robot4"])

    core.create_task(
        task_id="task-002",
        run_id=1,
        task_type=0,
        priority=1,
        target_ref="Z1",
    )

    core.assign_task("task-002", "robot4")
    core.start_task("task-002")
    core.finish_task(
        "task-002",
        ok=False,
        reason="BLOCKED",
    )

    task = core.tasks["task-002"]

    assert task.state == TaskState.FAILED
    assert task.reason == "BLOCKED"