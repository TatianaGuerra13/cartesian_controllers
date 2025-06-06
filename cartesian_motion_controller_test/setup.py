from setuptools import setup
import os
from glob import glob

package_name = 'cartesian_motion_controller_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name + '.test'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nblab',
    maintainer_email='tuo@email.com',
    description='Pacchetto per il controllo cartesiano in velocità con publisher di test',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_publisher = cartesian_motion_controller_test.test_publisher:main',
        ],
    },
)
