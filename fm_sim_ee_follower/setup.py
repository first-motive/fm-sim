from glob import glob

from setuptools import find_packages, setup

package_name = 'fm_sim_ee_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/profiles', glob('profiles/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='First Motive',
    maintainer_email='nish@ubundi.co.za',
    description='Damped-least-squares IK node that follows a commanded tool pose',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'ee_follower = fm_sim_ee_follower.follower_node:main',
        ],
    },
)
