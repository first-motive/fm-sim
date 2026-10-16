"""Read the names the follower needs from a shipped robot profile.

A robot profile names one robot build's joints, controllers and frames (contract §4). The
follower reads only ``base_frame`` and, for the arm it drives, ``joints``, ``arm_controller``
and ``ee_frame``; other keys are left to the profile's owners.

anvil_robot_profiles, fm_skills_task and fm_teleop_policy each carry their own profile
loader, and this is one more. It reads a subset on purpose and stays local until one shared
profile package replaces them all (https://github.com/first-motive/fm-robot/issues/49).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re

import yaml

PACKAGE = 'fm_sim_ee_follower'
DEFAULT_PROFILE = 'fm_openarm_v2'
# A ROS joint, frame or controller name. A profile name must take this shape too, so it can
# never step outside the profiles directory.
_NAME = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')
_SOURCE_PROFILES = Path(__file__).resolve().parent.parent / 'profiles'


@dataclass(frozen=True)
class ArmNames:
    """The profile names the follower uses for one arm."""

    base_frame: str
    joints: tuple[str, ...]
    arm_controller: str
    ee_frame: str

    @property
    def command_topic(self) -> str:
        """Topic the forward position controller reads joint commands from."""
        return f'/{self.arm_controller}/commands'


def profiles_dir() -> Path:
    """Return the shipped profiles: the installed share, else the source tree."""
    # Imported here, not at module level, so plain pytest loads profiles from the source
    # tree on a machine without ROS.
    try:
        from ament_index_python.packages import (
            get_package_share_directory,
            PackageNotFoundError,
        )
    except ImportError:
        return _SOURCE_PROFILES
    try:
        return Path(get_package_share_directory(PACKAGE)) / 'profiles'
    except PackageNotFoundError:
        return _SOURCE_PROFILES


def shipped_profiles() -> list[str]:
    """Return the names of the shipped profiles."""
    return sorted(p.stem for p in profiles_dir().glob('*.yaml'))


def load_arm(name: str, arm: str) -> ArmNames:
    """Load one arm's names from a shipped profile.

    Raises:
        FileNotFoundError: If no shipped profile has that name.
        ValueError: If the name is not an identifier, the arm is missing, or a key is
            missing or invalid.
    """
    if not _NAME.fullmatch(name):
        raise ValueError(f'robot profile {name!r} is not a profile name')
    path = profiles_dir() / f'{name}.yaml'
    if not path.is_file():
        raise FileNotFoundError(f'robot profile {name!r} not found; shipped: {shipped_profiles()}')
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    return parse_arm(data, arm, source=str(path))


def parse_arm(data: object, arm: str, source: str = '<dict>') -> ArmNames:
    """Validate a profile mapping and return one arm's names.

    Raises:
        ValueError: On a missing arm or key or a name that is not a valid ROS name.
    """
    if not isinstance(data, dict):
        raise ValueError(f'{source}: a profile must be a mapping')
    arms = data.get('arms')
    if not isinstance(arms, dict) or not isinstance(arms.get(arm), dict):
        raise ValueError(f'{source}: no arms.{arm} in profile')
    names = arms[arm]
    joints = names.get('joints')
    if not isinstance(joints, list) or not joints:
        raise ValueError(f'{source}: arms.{arm}.joints must be a non-empty list')
    return ArmNames(
        base_frame=_checked(data.get('base_frame'), 'base_frame', source),
        joints=tuple(_checked(j, f'arms.{arm}.joints', source) for j in joints),
        arm_controller=_checked(names.get('arm_controller'), f'arms.{arm}.arm_controller', source),
        ee_frame=_checked(names.get('ee_frame'), f'arms.{arm}.ee_frame', source),
    )


def _checked(name: object, key: str, source: str) -> str:
    if not isinstance(name, str) or not _NAME.fullmatch(name):
        raise ValueError(f'{source}: {key} {name!r} is not a valid ROS name')
    return name
