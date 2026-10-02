from control_tasks import bootstrap


def test_task_entry_points_use_shared_environment(monkeypatch):
    calls = []
    monkeypatch.setattr(bootstrap, "run_in_venv", calls.append)
    bootstrap.catheter_target_offset()
    bootstrap.catheter_tip_trajectory()
    bootstrap.catheter_tip_trajectory_file()
    bootstrap.catheter_sparse_point_experiment()
    bootstrap.catheter_tip_path()
    bootstrap.catheter_tip_path_file()
    bootstrap.catheter_camera_overlay()
    assert calls == [
        "control_tasks.target_offset",
        "control_tasks.trajectory_action",
        "control_tasks.trajectory_file",
        "control_tasks.sparse_point_experiment",
        "control_tasks.path_action",
        "control_tasks.path_file",
        "control_tasks.camera_overlay",
    ]
