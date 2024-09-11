import os
from glob import glob

from setuptools import setup

package_name = "coListener"

with open('requirements.txt') as f:
    requirements = f.read().splitlines()

with open('version') as f:
    version = f.read()
    #print(f"current version: {version}")


setup(
    name=package_name,
    version=version,
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["requirements.txt"]),
        (os.path.join('lib', 'python3.8', 'site-packages', package_name), ['version']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=requirements,
    zip_safe=True,
    maintainer="fei",
    maintainer_email="fei.gao@coscene.io",
    description="error code listener",
    license="Apache-2.0 license",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["coListener = coListener.main:main"],
    },
)
