from setuptools import find_packages, setup

package_name = "ros2_shadow_demos"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/nav2_shadow_demo.launch.py"]),
        ("share/" + package_name + "/rviz", ["rviz/nav2_shadow.rviz"]),
        ("share/" + package_name + "/params", ["params/nav2_shadow_planners.yaml"]),
        ("share/" + package_name + "/config", [
            "config/nav2_shadow.yaml",
            "config/demo_twist.yaml",
            "config/demo_isolated.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Puja",
    maintainer_email="catplotlib@gmail.com",
    description="Runnable demonstrations of ros2_shadow.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "shadow_demo_pair = ros2_shadow_demos.demo:main",
            "shadow_demo_candidate = ros2_shadow_demos.demo:candidate_main",
            "shadow_demo_production = ros2_shadow_demos.demo:production_main",
            "nav2_probe = ros2_shadow_demos.nav2_probe:main",
        ],
    },
)
