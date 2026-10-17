from glob import glob
from setuptools import setup


package_name = "bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=["bringup_ros"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/bringup"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.mviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Stuart G Johnson",
    maintainer_email="stuart.g.johnson@gmail.com",
    description="System ROS 2 launch files for the farm simulator viewers and nodes.",
    license="MIT",
)
