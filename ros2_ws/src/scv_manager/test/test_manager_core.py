from scv_manager.manager_core import (
    ManagerCore,
    RunState,
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

    assert task.state == TaskState.BLOCKED
    assert task.reason == "BLOCKED"


def test_tasks_are_dispatched_in_creation_order_one_at_a_time():
    core = ManagerCore(["robot4", "robot3"])
    run_id = core.start_run()

    core.create_task(
        task_id="survey",
        run_id=run_id,
        task_type=0,
        priority=1,
        target_ref="Z1",
        preferred_robot_id="robot4",
    )
    core.create_task(
        task_id="facility",
        run_id=run_id,
        task_type=1,
        priority=1,
        target_ref="EXT-01",
        preferred_robot_id="robot3",
    )
    core.create_task(
        task_id="reinspection",
        run_id=run_id,
        task_type=2,
        priority=10,
        target_ref="cell-1",
        preferred_robot_id="robot4",
    )

    assert core.next_pending_task().task_id == "survey"
    core.assign_task("survey", "robot4")
    assert core.next_pending_task() is None

    core.start_task("survey")
    core.finish_task("survey", ok=True, reason="")
    assert core.next_pending_task().task_id == "facility"

    core.assign_task("facility", "robot3")
    core.start_task("facility")
    core.finish_task("facility", ok=True, reason="")
    assert core.next_pending_task().task_id == "reinspection"


def test_estop_and_resume_requeue_active_task():
    core = ManagerCore(["robot4"])
    run_id = core.start_run()
    core.create_task(
        task_id="survey",
        run_id=run_id,
        task_type=0,
        priority=1,
        target_ref="Z1",
        preferred_robot_id="robot4",
    )
    core.assign_task("survey", "robot4")
    core.start_task("survey")

    core.estop()
    assert core.run_state == RunState.ESTOPPED
    assert core.tasks["survey"].state == TaskState.CANCELED

    core.resume()
    assert core.run_state == RunState.RUNNING
    assert core.tasks["survey"].state == TaskState.PENDING
    assert core.tasks["survey"].robot_id is None


def test_fixed_role_task_is_not_reassigned_to_other_robot():
    core = ManagerCore(["robot4", "robot3"])
    run_id = core.start_run()
    core.create_task(
        task_id="survey",
        run_id=run_id,
        task_type=0,
        priority=1,
        target_ref="Z1",
        preferred_robot_id="robot4",
        allowed_robot_ids=("robot4",),
    )

    moved = core.reassign_pending_tasks("robot4", "robot3")
    assert moved == 0
    assert core.tasks["survey"].preferred_robot_id == "robot4"


def test_run_completes_only_when_all_tasks_are_terminal():
    core = ManagerCore(["robot4"])
    run_id = core.start_run()
    core.create_task(
        task_id="survey",
        run_id=run_id,
        task_type=0,
        priority=1,
        target_ref="Z1",
    )

    assert not core.all_tasks_terminal()
    core.finish_task("survey", ok=True, reason="")
    assert core.all_tasks_terminal()

    core.complete_run()
    assert core.run_state == RunState.COMPLETED
