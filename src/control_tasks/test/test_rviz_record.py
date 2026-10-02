import pytest

from control_tasks.rviz_record import _window_id


def test_rviz_recorder_finds_one_matching_window():
    tree = '''
       0x02a00007 "catheter_sim_rviz - RViz": ("rviz2" "Rviz")
       0x03000001 "Terminal": ("gnome-terminal" "Gnome-terminal")
    '''
    assert _window_id(tree, "RViz") == 0x02a00007


def test_rviz_recorder_rejects_ambiguous_windows():
    tree = '''
       0x02a00007 "first RViz": ("rviz2" "Rviz")
       0x02a00008 "second RViz": ("rviz2" "Rviz")
    '''
    with pytest.raises(RuntimeError, match="found 2 windows"):
        _window_id(tree, "RViz")
