from setuptools import find_packages, setup


package_name = "vision_pkg"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/resource", ["resource/best.pt"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mu-09",
    maintainer_email="mu09@example.com",
    description="YOLO vision node for fire-extinguisher gauge detection.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "test = vision_pkg.test:main",
        ],
    },
)
