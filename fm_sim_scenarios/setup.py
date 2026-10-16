from setuptools import find_packages, setup

package_name = 'fm_sim_scenarios'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='First Motive',
    maintainer_email='nish@ubundi.co.za',
    description='Backend-neutral scenarios: seeded spawns, the success predicate, '
    'per-backend world generators and the scenario node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scenario_node = fm_sim_scenarios.scenario_node:main',
        ],
    },
)
