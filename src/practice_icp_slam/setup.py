import os
from setuptools import setup

package_name = 'practice_icp_slam'
submodule_name = 'utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, os.path.join(package_name, submodule_name)],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'practice_icp_slam = practice_icp_slam.practice_icp_slam:main'
        ],
    },
)
