import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lotti_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- ADD THIS LINE TO REGISTER YOUR LAUNCH FILES ---
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='FFmpeg low-latency camera streaming package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ffmpeg_camera_node = lotti_vision.ffmpeg_camera_node:main',
            'camera_dashboard = lotti_vision.camera_dashboard:main',
        ],
    },
)