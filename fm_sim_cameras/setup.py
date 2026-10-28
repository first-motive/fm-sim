from setuptools import find_packages, setup

package_name = 'fm_sim_cameras'

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
    description='Simulated cameras: the fisheye warp of a pinhole render, published '
    'JPEG-compressed with the real intrinsics',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_camera = fm_sim_cameras.camera_node:main',
        ],
    },
)
