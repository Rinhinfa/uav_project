from setuptools import find_packages, setup

package_name = "orchard_sim"

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
    description="Orchard simulation helper nodes.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "world_generator = orchard_sim.world_generator:main",
            "fleet_state_publisher = orchard_sim.fleet_state_publisher:main",
            "spawn_plan_publisher = orchard_sim.spawn_plan_publisher:main",
            "gz_path_follower = orchard_sim.gz_path_follower:main",
            "viz_publisher = orchard_sim.viz_publisher:main",
        ],
    },
)
