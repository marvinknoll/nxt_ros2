import os
from glob import glob
from setuptools import setup

package_name = 'nxt_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         glob(os.path.join('resource', package_name))),

        (os.path.join('share', package_name),
         glob(os.path.join('package.xml'))),

        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),

        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marvin',
    maintainer_email='marvin.knoll@protonmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nxt = nxt_ros2.nxt:main'
        ],
    },
)
