from setuptools import find_packages, setup

package_name = "automation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch",
         ["launch/estimator.launch.py", "launch/collection.launch.py"]),
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
            "state_estimator = automation.estimation.node:main",
            "em_bridge = automation.collection.em_bridge:main",
            "collection = automation.collection.node:main",
        ],
    },
)
