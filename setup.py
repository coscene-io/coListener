from setuptools import setup, find_packages

setup(
    name='coListener',
    version='0.0.0',
    packages=find_packages("src"),
    package_dir={'': 'src'},
    install_requires=['rospy'],
)
