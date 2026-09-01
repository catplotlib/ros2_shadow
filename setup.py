from setuptools import find_packages, setup

package_name = "ros2_shadow"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="Puja",
    maintainer_email="catplotlib@gmail.com",
    description="Shadow-mode comparison for ROS 2 nodes.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "shadow = ros2_shadow.cli:main",
            "shadow_demo_candidate = ros2_shadow.demo:candidate_main",
            "shadow_demo_production = ros2_shadow.demo:production_main",
            "shadow_demo_pair = ros2_shadow.demo:main",
        ],
    },
)
