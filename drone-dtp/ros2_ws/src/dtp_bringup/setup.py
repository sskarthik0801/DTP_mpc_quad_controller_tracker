import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'dtp_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Both directories must be installed, not just launch/: dtp_system.launch.py locates
        # the other launch files through get_package_share_directory, and every layer resolves
        # config/dtp_params.yaml the same way. A launch file that is not installed fails with
        # "file not found" at launch, which is loud; a params file that is not installed fails
        # by silently leaving every node on its own defaults, which is not.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krishna',
    maintainer_email='srajule@purdue.edu',
    description='Launch composition for the DTP human-follow stack: sensors, SUPER, follower.',
    license='Proprietary',
    tests_require=['pytest'],
    # No console_scripts. dtp_bringup ships launch files and one importable module
    # (dtp_bringup.preflight, imported BY the launch files -- which is why this is an
    # ament_python package with a real python module rather than a launch-only ament_cmake one).
    entry_points={
        'console_scripts': [],
    },
)
