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
    package_data={"catheter_control": ["safety/fixtures/*.json"]},
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        *config_data_files,
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chen-lab",
    maintainer_email="todo@todo.com",
    description="Closed-loop controller core.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "catheter_mppi = catheter_control.bootstrap:catheter_mppi",
            "phase5_preflight = catheter_control.bootstrap:phase5_preflight",
            "phase5_conformance = catheter_control.bootstrap:phase5_conformance",
            "control_shadow_worker = catheter_control.bootstrap:control_shadow_worker",
        ],
    },
)
