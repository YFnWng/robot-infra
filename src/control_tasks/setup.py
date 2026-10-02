from setuptools import find_packages, setup


package_name = "control_tasks"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="chen-lab",
    maintainer_email="todo@todo.com",
    description="Controller task clients and operator-facing runtime tools.",
    license="MIT",
    entry_points={
        "console_scripts": [
            ("catheter_target_offset = "
             "control_tasks.bootstrap:catheter_target_offset"),
            ("catheter_tip_trajectory = "
             "control_tasks.bootstrap:catheter_tip_trajectory"),
            ("catheter_tip_trajectory_file = "
             "control_tasks.bootstrap:catheter_tip_trajectory_file"),
            ("catheter_sparse_point_experiment = "
             "control_tasks.bootstrap:catheter_sparse_point_experiment"),
            ("catheter_tip_path = "
             "control_tasks.bootstrap:catheter_tip_path"),
            ("catheter_tip_path_file = "
             "control_tasks.bootstrap:catheter_tip_path_file"),
            ("catheter_camera_overlay = "
             "control_tasks.bootstrap:catheter_camera_overlay"),
            "catheter_rviz_record = control_tasks.rviz_record:main",
        ],
    },
)
