from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'prueba_robotics4_0'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src', include=['prueba_robotics4_0', 'prueba_robotics4_0.*']),
    package_dir={'': 'src'},  # This tells Python to treat 'src' as the root package directory
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'test'), glob('test/**/*.py')),  # Recursively include all test files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sofia',
    maintainer_email='sofia.castano@iao.edu.co',
    description='ROS 2 package with publisher, subscriber, and more.',
    license='TODO: License declaration',
    tests_require=['pytest', 'pytest-cov'],  # Add pytest-cov for coverage reports
    entry_points={
        'console_scripts': [
            'publicador = prueba_robotics4_0.publicador:main',
            'suscriptor = prueba_robotics4_0.suscriptor:main',
            'turtle_mover = prueba_robotics4_0.turtle_mover:main',
            'color_detection = prueba_robotics4_0.color_detection_node:main',
            'object_detection = prueba_robotics4_0.object_detection_node:main',
        ],
    },
)
