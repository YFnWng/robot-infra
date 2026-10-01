from glob import glob

from setuptools import find_packages, setup

package_name = "automation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config",
         ["config/live_coil_estimation.yaml", "config/catheter_limits.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chen-lab",
    maintainer_email="todo@todo.com",
    description="High-level automation nodes for catheter robot.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "state_estimator = automation.perception.state_estimator:main",
            "em_bridge = automation.perception.em_bridge:main",
            "collection = automation.experiments.collection:main",
            "causal_runtime_identity = "
            "automation.supervision.runtime_identity:main",
            "causal_session_check = automation.supervision.session_check:main",
            "causal_stationary_analysis = "
            "automation.supervision.stationary_analysis:main",
            "marker_tracking = automation.perception.marker_tracking:main",
            "marker_udp_receiver = "
            "automation.perception.marker_udp_receiver:main",
        ],
    },
)
