from pathlib import Path

from setuptools import find_packages, setup


package_name = "simulation"
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
        (f"share/{package_name}", ["package.xml", "README.md"]),
        *config_data_files,
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chen-lab",
    maintainer_email="todo@todo.com",
    description=(
        "Isolated actuator, perception, scenario, and visualization "
        "simulation."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            ("catheter_sim_device = "
             "simulation.bootstrap:catheter_sim_device"),
            ("catheter_sim_perception = "
             "simulation.bootstrap:catheter_sim_perception"),
            ("catheter_sim_visualizer = "
             "simulation.bootstrap:catheter_sim_visualizer"),
            ("catheter_sim_target = "
             "simulation.bootstrap:catheter_sim_target"),
            ("catheter_sim_scenario = "
             "simulation.bootstrap:catheter_sim_scenario"),
        ],
    },
)
