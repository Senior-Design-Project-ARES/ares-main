from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'testing_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (f'share/{package_name}/test_map', glob('test_map/*')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ares',
    maintainer_email='yukang.kong.uni@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            f'publish_fake_data = {package_name}.publish_fake_data:main',
            f'publish_fake_data_coordinate = {package_name}.publish_fake_data_coordinate:main',
            f'call_searching_server = {package_name}.call_searching_server:main',
            f'plot_generated_graph = {package_name}.plot_generated_graph:main',
            f'rover_simulation = {package_name}.rover_simulator:main',
            f'lidar_simulation = {package_name}.lidar_simulator:main',
            f'visualization = {package_name}.visualization:main',
            f'export_data = {package_name}.export_data:main',
        ],
    },
)
