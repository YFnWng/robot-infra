from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


package_name = "catheter_control"
config_data_files = []
for config_path in sorted(Path("config").rglob("*")):
    if not config_path.is_file():
        continue
    destination = Path("share") / package_name / config_path.parent
    config_data_files.append((str(destination), [str(config_path)]))


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        *config_data_files,
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chen-lab",
    maintainer_email="todo@todo.com",
    description="Closed-loop catheter model and control integration.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "catheter_mppi = catheter_control.bootstrap:catheter_mppi",
            "phase5_preflight = catheter_control.bootstrap:phase5_preflight",
            ("catheter_sim_device = "
             "catheter_control.bootstrap:catheter_sim_device"),
            ("catheter_sim_perception = "
             "catheter_control.bootstrap:catheter_sim_perception"),
            ("catheter_sim_visualizer = "
             "catheter_control.bootstrap:catheter_sim_visualizer"),
            ("catheter_sim_target = "
             "catheter_control.bootstrap:catheter_sim_target"),
            ("catheter_sim_scenario = "
             "catheter_control.bootstrap:catheter_sim_scenario"),
            ("catheter_target_offset = "
             "catheter_control.bootstrap:catheter_target_offset"),
            ("catheter_tip_trajectory = "
             "catheter_control.bootstrap:catheter_tip_trajectory"),
            ("catheter_tip_trajectory_file = "
             "catheter_control.bootstrap:catheter_tip_trajectory_file"),
            ("catheter_sparse_point_experiment = "
             "catheter_control.bootstrap:catheter_sparse_point_experiment"),
            ("catheter_tip_path = "
             "catheter_control.bootstrap:catheter_tip_path"),
            ("catheter_tip_path_file = "
             "catheter_control.bootstrap:catheter_tip_path_file"),
            ("catheter_camera_overlay = "
             "catheter_control.bootstrap:catheter_camera_overlay"),
            ("catheter_rviz_record = "
             "catheter_control.rviz_record:main"),
        ],
    },
)
