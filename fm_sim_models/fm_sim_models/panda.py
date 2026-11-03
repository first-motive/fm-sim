"""Franka Panda MJCF from the vendored mujoco_menagerie, renamed to fm-robot's joints.

fm-robot's panda.sim.urdf.xacro uses franka_description's names, which call the
Panda `fer`: fer_joint1..7 and fer_finger_joint1..2. Menagerie's
franka_emika_panda/panda.xml names the same joints joint1..7 and
finger_joint1..2, and MujocoSystemInterface matches the ros2_control joints to the
MJCF by name, so this module renames them.

It also rewrites the actuators into the form MujocoSystemInterface commands by
position. Menagerie drives every joint with a <general> actuator (gain kp, bias
-kp, -kv), which the interface treats as a custom actuator; each becomes the
equivalent <position kp kv>. The hand is driven through a tendon on a 0..255
scale; it becomes a position actuator on fer_finger_joint1 in metres, with the
tendon's own gains, and Menagerie's equality keeps fer_finger_joint2 following it.

Every body is gravity compensated, as the real arm's controller compensates it, so
the servos hold their targets with no steady sag for MoveIt Servo to compound.

``mjcf_path`` re-derives the model on every call and writes it once per content
hash (``fm_sim_models.cache``).
"""

import os
import xml.etree.ElementTree as ET

from fm_sim_models import cache

_SOURCE = ('franka_emika_panda', 'panda.xml')
ARM_JOINTS = tuple(f'joint{i}' for i in range(1, 8))
FINGER_JOINTS = ('finger_joint1', 'finger_joint2')
JOINT_PREFIX = 'fer_'
# Menagerie's tendon actuator: force = 0.0157 * ctrl - 100 * length - 10 * velocity,
# with ctrl 0..255 standing for 0..0.04 m. As a joint position actuator that is
# kp 100, kv 10 over the finger's own 0..0.04 m travel, capped at its 100 N.
FINGER_KP = 100.0
FINGER_KV = 10.0
FINGER_RANGE_M = (0.0, 0.04)
FINGER_FORCE_N = 100.0


def _fail(what):
    raise RuntimeError(f'mujoco_menagerie panda.xml does not match the fm-robot patch: {what}')


def _source(menagerie):
    return os.path.join(menagerie, *_SOURCE)


def _rename_joints(root):
    """Prefix every arm and finger joint, and every reference to one, with fer_."""
    names = {name: JOINT_PREFIX + name for name in ARM_JOINTS + FINGER_JOINTS}
    found = set()
    for joint in root.iter('joint'):
        for key in ('name', 'joint', 'joint1', 'joint2'):
            value = joint.get(key)
            if value in names:
                joint.set(key, names[value])
                if key == 'name':
                    found.add(value)
    missing = sorted(set(names) - found)
    if missing:
        _fail(f'no joint named {missing}')
    for actuator in root.iterfind('actuator/*'):
        if actuator.get('joint') in names:
            actuator.set('joint', names[actuator.get('joint')])


def _arm_position_actuators(actuators, defaults):
    """Return the <position> equivalent of each arm joint's <general> actuator.

    A <general> with gain kp and bias (0, -kp, -kv) is a position servo; settings
    the element leaves out come from the `panda` default class (``defaults``).
    """
    arm = []
    tendons = [a for a in actuators if a.get('tendon') is not None]
    if len(tendons) != 1:
        _fail(f'{len(tendons)} tendon actuators, expected the one hand actuator')
    for general in actuators:
        if general.tag != 'general':
            _fail(f'unexpected <{general.tag}> actuator')
        settings = {**defaults, **general.attrib}
        if settings.get('joint', '').removeprefix(JOINT_PREFIX) not in ARM_JOINTS:
            if settings.get('tendon') is None:
                _fail(f'actuator {settings.get("name")!r} drives neither an arm joint nor the hand')
            continue
        arm.append(ET.Element('position', {
            'name': settings['name'], 'joint': settings['joint'],
            'kp': repr(float(settings['gainprm'].split()[0])),
            'kv': repr(-float(settings['biasprm'].split()[2])),
            'ctrlrange': settings['ctrlrange'], 'forcerange': settings['forcerange'],
        }))
    if len(arm) != len(ARM_JOINTS):
        _fail(f'{len(arm)} arm actuators, expected {len(ARM_JOINTS)}')
    return arm


def _hand_position_actuator():
    """Return the hand servo: fer_finger_joint1 in metres, with the tendon's gains."""
    lower, upper = FINGER_RANGE_M
    return ET.Element('position', {
        'name': 'hand', 'joint': JOINT_PREFIX + FINGER_JOINTS[0],
        'kp': f'{FINGER_KP:g}', 'kv': f'{FINGER_KV:g}',
        'ctrlrange': f'{lower:g} {upper:g}',
        'forcerange': f'{-FINGER_FORCE_N:g} {FINGER_FORCE_N:g}',
    })


def _position_actuators(root):
    """Replace Menagerie's actuators with position servos: the arm's, then the hand's."""
    actuators = root.find('actuator')
    if actuators is None:
        _fail('no <actuator>')
    general_defaults = root.find("default/default[@class='panda']/general")
    defaults = dict(general_defaults.attrib) if general_defaults is not None else {}
    servos = _arm_position_actuators(list(actuators), defaults) + [_hand_position_actuator()]
    for actuator in list(actuators):
        actuators.remove(actuator)
    actuators.extend(servos)


def _compensate_gravity(root):
    """Cancel gravity on every body, as the Panda's own controller does.

    The position servos have no integral term, so gravity holds each joint a little
    off its target (fer_joint4 sags about 0.006 rad at the ready pose). MoveIt Servo
    builds every command from the measured state, so that sag is taken in as the new
    target each cycle and compounds into drift on joints a jog never named (#139).
    """
    for body in root.iter('body'):
        body.set('gravcomp', '1')


def _drop_keyframes(root):
    """Remove the keyframes; their ctrl vectors match the old 8-actuator layout only."""
    keyframe = root.find('keyframe')
    if keyframe is not None:
        root.remove(keyframe)


def _absolute_meshdir(root, source):
    """Make meshdir absolute, so the renamed copy resolves its meshes from anywhere.

    Only meshdir is resolved, so a second <compiler>, an assetdir or texturedir, or
    an <include> is refused.
    """
    compilers = root.findall('compiler')
    if len(compilers) != 1:
        _fail(f'{len(compilers)} <compiler> elements, expected one')
    compiler = compilers[0]
    for key in ('assetdir', 'texturedir'):
        if key in compiler.attrib:
            _fail(f'<compiler {key}> is set; only meshdir is resolved')
    if root.find('.//include') is not None:
        _fail('an <include>, whose relative paths would not resolve from the copy')
    meshdir = compiler.get('meshdir')
    if meshdir is None:
        _fail('no <compiler meshdir>')
    compiler.set('meshdir', os.path.join(os.path.dirname(os.path.abspath(source)), meshdir))


def mjcf_text(menagerie):
    """Return Menagerie's panda.xml under ``menagerie`` renamed and re-actuated for fm-robot."""
    source = _source(menagerie)
    root = ET.parse(source).getroot()
    _absolute_meshdir(root, source)
    _rename_joints(root)
    _position_actuators(root)
    _compensate_gravity(root)
    _drop_keyframes(root)
    ET.indent(root)
    return ET.tostring(root, encoding='unicode') + '\n'


def mjcf_path(menagerie):
    """Write the renamed Panda MJCF for the vendored ``menagerie`` and return its path."""
    source = _source(menagerie)
    if not os.path.isfile(source):
        raise RuntimeError(
            f'mujoco_menagerie is not vendored: {source} is missing (vcs import fm-sim.repos)'
        )
    return cache.write(mjcf_text(menagerie), 'panda')
