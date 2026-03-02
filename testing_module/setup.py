from setuptools import find_packages, setup
from glob import glob

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
            f'publish_fake_data_node = {package_name}.publish_fake_data:main',
            f'publish_fake_data_coordinate_node = {package_name}.publish_fake_data_coordinate:main',
            f'call_searching_server_node = {package_name}.call_searching_server:main',
            f'plot_generated_graph_node = {package_name}.plot_generated_graph:main',
        ],
    },
)
