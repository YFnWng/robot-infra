from setuptools import find_packages, setup


package_name = "experiments"

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
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="chen-lab",
    maintainer_email="todo@todo.com",
    description="Experiment schedules and guarded data collection.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "collection = experiments.collection:main",
            "session_recording = experiments.session_recording:main",
        ],
    },
)
