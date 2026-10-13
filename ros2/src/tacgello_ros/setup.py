from setuptools import find_packages, setup
import glob

package_name = "tacgello_ros"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="TacGELLO Authors",
    maintainer_email="user@example.com",
    description=(
        "TacGELLO: tactile contact and joint-current load feedback for a "
        "3D-printed STS3215 UR3 teleoperation leader."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bluetooth_gripper_bridge = tacgello_ros.bluetooth_gripper_bridge:main",
            "leader_publisher = tacgello_ros.leader_publisher:main",
            "lerobot_demo_recorder = tacgello_ros.lerobot_demo_recorder:main",
            "tracking_report_recorder = tacgello_ros.tracking_report_recorder:main",
        ],
    },
)
