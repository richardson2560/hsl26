from setuptools import setup
import os
from glob import glob

package_name = 'hsl_safety'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HSL26 Team',
    maintainer_email='team@starline.ru',
    description='Safety supervisor for HSL26',
    license='Proprietary',
    tests_require=['pytest'],
)
