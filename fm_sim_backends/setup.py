from glob import glob

from setuptools import find_packages, setup

package_name = 'fm_sim_backends'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py'),
        ),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # Isaac Sim only: the stage runner, run with Isaac Sim's own Python.
        ('share/' + package_name + '/isaac', glob('isaac/*.py')),
        # Catalog entries (see fm_sim_bringup.catalog): index markers + their yaml.
        (
            'share/ament_index/resource_index/fm_sim.backends',
            glob('resource/fm_sim.backends/*'),
        ),
        ('share/' + package_name + '/catalog/backends', glob('catalog/backends/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='First Motive',
    maintainer_email='nish@ubundi.co.za',
    description='Simulation backend hosts: per-engine launch files (mujoco, gazebo, isaac)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_info = fm_sim_backends.camera_info:main',
        ],
    },
)
