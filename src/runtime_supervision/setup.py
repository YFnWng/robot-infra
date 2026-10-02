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
            "causal_runtime_identity = runtime_supervision.bootstrap:runtime_identity",
            "causal_session_check = runtime_supervision.session_check:main",
            "causal_stationary_analysis = runtime_supervision.stationary_analysis:main",
        ],
    },
)
