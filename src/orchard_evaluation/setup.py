from setuptools import find_packages, setup

package_name = "orchard_evaluation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="liaw",
    maintainer_email="liaw@example.com",
    description="Experiment metric scripts for orchard project.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "metrics_aggregator = orchard_evaluation.metrics_aggregator:main",
            "batch_runner = orchard_evaluation.batch_runner:main",
            "report_generator = orchard_evaluation.report_generator:main",
        ],
    },
)
