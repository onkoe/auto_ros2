from setuptools import find_packages, setup

package_name = "soro_simulator_driver"
data_files = []
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)
data_files.append(
    ("share/" + package_name + "/launch", ["launch/launch_sim.py"])
)
data_files.append(
    ("share/" + package_name + "/worlds", ["worlds/base_world.wbt"])
)
data_files.append(
    ("share/" + package_name + "/resource", ["resource/remi.urdf"])
)
data_files.append(("share/" + package_name, ["package.xml"]))


_ = setup(
    name=package_name,
    version="0.0.0",
    python_requires=">=3.13",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    zip_safe=True,
    maintainer="brendan",
    maintainer_email="bford@axmilius.com",
    description="TODO",
    license="TODO",
    entry_points={
        "console_scripts": [
            "driver = driver.main:main",
        ],
    },
)
