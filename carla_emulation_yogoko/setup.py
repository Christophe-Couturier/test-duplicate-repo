from setuptools import setup

package_name = "carla_emulation_yogoko"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/yogoko_publisher_multi.launch.py",
                "launch/pvt_translator.launch.py",
                "launch/pvt_translator_multi.launch.py",
                "launch/spatem_translator.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yogoko",
    maintainer_email="support@yogoko.fr",
    description="ROS2 packages use to emulate SPATEM and CAM messages to YGhost",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pvt_translator = {}.carla_pvt_translator_node:main".format(package_name),
            "spatem_translator = {}.carla_spatem_translator_node:main".format(
                package_name
            ),
        ],
    },
)
