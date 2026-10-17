"""The camera relays every rendering backend starts, and the cameras file they read.

A backend renders each camera raw on ``/cam_<role>/image_raw``; these relays give
the contract's topics: image_transport's republisher compresses it onto
``/cam_<role>/image_raw/compressed`` (JPEG), and one ``camera_info`` node answers
every image with the camera's intrinsics on ``/cam_<role>/camera_info`` (the
CameraInfo from ``fm_sim_cameras.camera_info``). The Gazebo and Isaac backends
both start them; each applies its own rules to the loaded cameras on top.
"""

import json

from fm_sim_cameras import cameras_file as shared_cameras_file
from launch_ros.actions import Node

# The relays discover only on this host, as the agent's run containers do
# (agent/src/fm_sim_agent/runner.py DEFAULT_DDS_RANGE), so no LAN peer can feed
# them fake frames.
HOST_LOCAL_ENV = {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}


def load_cameras(cameras_file):
    """Return a robot's cameras file as ``{role: camera}``; ``{}`` for no file.

    The shared loader and check (``fm_sim_cameras.cameras_file.load``); a backend
    refuses what it cannot render on top.
    """
    if not cameras_file:
        return {}
    return shared_cameras_file.load(cameras_file)


def camera_relays(cameras):
    """Return the republisher per camera and the one camera_info node; none for none.

    ``cameras`` is ``{role: camera}`` as ``load_cameras`` returns it;
    the camera_info node publishes exactly its intrinsics and distortion.
    """
    if not cameras:
        return []
    relays = [
        Node(
            package='image_transport',
            executable='republish',
            name=f'cam_{role}_compress',
            parameters=[{'in_transport': 'raw', 'out_transport': 'compressed'}],
            # The compressed publisher's own name is remapped: on Jazzy a remap of
            # the base name 'out' leaves it on /out/compressed (checked live).
            remappings=[
                ('in', f'/cam_{role}/image_raw'),
                ('out/compressed', f'/cam_{role}/image_raw/compressed'),
            ],
            additional_env=HOST_LOCAL_ENV,
            output='screen',
        )
        for role in sorted(cameras)
    ]
    relays.append(Node(
        package='fm_sim_backends',
        executable='camera_info',
        parameters=[{'cameras': json.dumps(cameras, sort_keys=True)}],
        additional_env=HOST_LOCAL_ENV,
        output='screen',
    ))
    return relays
