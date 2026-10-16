from glob import glob

from setuptools import find_packages, setup

package_name = 'fm_sim_bringup'
share = 'share/' + package_name
index = 'share/ament_index/resource_index/'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (index + 'packages', ['resource/' + package_name]),
        (share, ['package.xml']),
        (share + '/launch', glob('launch/*.launch.py')),
        (share + '/schema', glob('schema/*.json')),
        (share + '/config/openarm', glob('config/openarm/*.yaml')),
        # Catalog entries: one index marker per entry, pointing at its yaml.
        (index + 'fm_sim.robots', glob('resource/fm_sim.robots/*')),
        (index + 'fm_sim.scenarios', glob('resource/fm_sim.scenarios/*')),
        (index + 'fm_sim.controllers', glob('resource/fm_sim.controllers/*')),
        (share + '/catalog/robots', glob('catalog/robots/*.yaml')),
        (share + '/catalog/scenarios', glob('catalog/scenarios/*.yaml')),
        (share + '/catalog/controllers', glob('catalog/controllers/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='First Motive',
    maintainer_email='nish@ubundi.co.za',
    description='Sim run launch and the ament-index catalog of backends, robots, '
    'scenarios and controllers',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'catalog = fm_sim_bringup.catalog:main',
        ],
    },
)
