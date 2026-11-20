from setuptools import setup

setup(
    name="farm_mission_processor", version="0.1.0",
    packages=["farm_mission_processor"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/farm_mission_processor"]),
        ("share/farm_mission_processor", ["package.xml"]),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Stuart G Johnson", maintainer_email="stuart.g.johnson@gmail.com",
    description="Farm mission planning and Nav2 execution", license="MIT",
    entry_points={"console_scripts": ["farm_mission_processor = farm_mission_processor.node:main"]},
)
