# -*- coding: utf-8 -*-
from setuptools import setup, find_packages

with open("README.md", "r") as f:
    long_description = f.read()

setup(
    name="common_robot_interface",
    version="0.5.3",
    description="Common Robot Interface",
    license="GPLv3",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Nathan Lepora, John Lloyd",
    author_email="n.lepora@bristol.ac.uk",
    url="https://github.com/robot-dexterity/common-robot-interface",
    packages=find_packages(),
	package_data={
        'cri.ur': ['rtde_config.xml'],
        "cri.dobot.magician": ['DobotDll.dll',"msvcp120.dll","msvcr120.dll","Qt5Core.dll","Qt5Network.dll","Qt5SerialPort.dll"]
    },
    install_requires=["numpy", "transforms3d"],
    extras_require={"sim": ["tactile_sim"]},
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
    ],
)
