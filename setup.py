from setuptools import setup

import glob
import os

package_name = 'eight_cdpr'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'model/meshes'), glob.glob('urdf/model/*.dae')),
        (os.path.join('share', package_name, 'model'), glob.glob('urdf/eight_cdpr_model.urdf')),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/eight_cdpr_launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob.glob('rviz/config.rviz'))
        #(os.path.join('share', package_name, 'condig'), glob.glob('config/params.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emad',
    maintainer_email='emad.aldoghry@rwth-aachen.de',
    description='Eight cable parallel driven robot',
    license='RWTH',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['eight_cdpr = eight_cdpr.main:main'
        ],
    },
)