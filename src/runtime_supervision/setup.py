from setuptools import find_packages, setup


package_name = "runtime_supervision"

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
    description="Runtime identity, session validation, and offline qualification.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "compute_profile = runtime_supervision.bootstrap:compute_profile",
            "replay_takeup_startup = runtime_supervision.bootstrap:replay_takeup_startup",
            "compare_compute = runtime_supervision.bootstrap:compare_compute",
            "qualify_live_compute = runtime_supervision.bootstrap:qualify_live_compute",
            "qualify_recovery = runtime_supervision.bootstrap:qualify_recovery",
            "engagement_g4 = runtime_supervision.bootstrap:engagement_g4",
            "engagement_shadow = runtime_supervision.bootstrap:engagement_shadow",
            "reduced_filter_shadow = runtime_supervision.bootstrap:reduced_filter_shadow",
            "reduced_filter_replay = runtime_supervision.bootstrap:reduced_filter_replay",
            "benchmark_accelerator = runtime_supervision.bootstrap:benchmark_accelerator",
            "causal_runtime_identity = runtime_supervision.bootstrap:runtime_identity",
            "causal_session_check = runtime_supervision.session_check:main",
            "causal_stationary_analysis = runtime_supervision.stationary_analysis:main",
        ],
    },
)
