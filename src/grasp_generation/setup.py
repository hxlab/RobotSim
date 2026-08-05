from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'grasp_generation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            "grasp_generation=grasp_generation.grasp_generation:main"
        ],
    },
)