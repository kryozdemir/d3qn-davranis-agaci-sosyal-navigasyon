from setuptools import setup, find_packages
from glob import glob
import os

paket_adi = "d3qn_sosyal_navigasyon"

setup(
    name=paket_adi,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + paket_adi]),
        ("share/" + paket_adi, ["package.xml"]),
        (os.path.join("share", paket_adi, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", paket_adi, "config"), glob("config/*.yaml")),
        (os.path.join("share", paket_adi, "config", "senaryo_ayarlari"), glob("config/senaryo_ayarlari/*.yaml")),
        (os.path.join("share", paket_adi, "config", "davranis_agaci_xml"), glob("config/davranis_agaci_xml/*.xml")),
        (os.path.join("share", paket_adi, "worlds"), glob("worlds/*.world")),
        (os.path.join("share", paket_adi, "dokumanlar"), glob("dokumanlar/*.md")),
        (os.path.join("share", paket_adi, "scripts"), glob("scripts/*.sh") + glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Koray Özdemir",
    maintainer_email="korayozdemir34@gmail.com",
    description="D3QN tabanlı sosyal robot navigasyonu (ROS2 + Gazebo) - eğitim ve test altyapısı.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "egit = d3qn_sosyal_navigasyon.src.egit:main",
            "test = d3qn_sosyal_navigasyon.src.test:main",
            "ana_node = d3qn_sosyal_navigasyon.src.ros2_entegrasyon.ana_node:main",
            "insan_tespit_node = d3qn_sosyal_navigasyon.src.ros2_entegrasyon.yolov5_insan_tespit_node:main",
        ],
    },
)
