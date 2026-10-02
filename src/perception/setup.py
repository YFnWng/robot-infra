from setuptools import find_packages, setup


package_name = "perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/config",
            ["config/live_coil_estimation.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chen-lab",
    maintainer_email="todo@todo.com",
    description="Online marker tracking and shape-estimation adapters.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "state_estimator = perception.state_estimator:main",
            "em_bridge = perception.em_bridge:main",
            "marker_tracking = perception.marker_tracking:main",
            "marker_udp_receiver = perception.marker_udp_receiver:main",
        ],
    },
)
