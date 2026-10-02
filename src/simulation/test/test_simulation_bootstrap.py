from simulation import bootstrap


def test_simulation_entry_points_use_shared_environment(monkeypatch):
    calls = []
    monkeypatch.setattr(bootstrap, "run_in_venv", calls.append)
    bootstrap.catheter_sim_device()
    bootstrap.catheter_sim_perception()
    bootstrap.catheter_sim_visualizer()
    bootstrap.catheter_sim_target()
    bootstrap.catheter_sim_scenario()
    assert calls == [
        "simulation.sim_device",
        "simulation.sim_perception",
        "simulation.sim_visualizer",
        "simulation.sim_target",
        "simulation.sim_scenario",
    ]
