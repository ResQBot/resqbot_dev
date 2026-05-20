from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'lotti_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch/'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paul',
    maintainer_email='73643665+PaulKupka@users.noreply.github.com',
    description='Xbox-based teleoperation node for the Lotti3 drive, flippers, and arm servo control.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = lotti_teleop.lotti_teleop:main',
        ],
    },
)
