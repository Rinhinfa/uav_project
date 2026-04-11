from setuptools import find_packages, setup

package_name = "orchard_task_allocation"

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
    description="Task allocation and global waypoint generation nodes.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task_allocator = orchard_task_allocation.task_allocator:main",
            "dynamic_scheduler = orchard_task_allocation.dynamic_scheduler:main",
            "event_injector = orchard_task_allocation.event_injector:main",
            "event_profile_player = orchard_task_allocation.event_profile_player:main",
        ],
    },
)
