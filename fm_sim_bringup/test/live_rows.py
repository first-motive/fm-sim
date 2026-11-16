"""Which live run.launch.py rows this host can run, for test_run_launch."""

from fm_sim_bringup import catalog

# The MJCF keys fm_sim_models compiles at run time with the mujoco Python binding,
# which the sim image installs from requirements-sim.txt and a bare ROS image lacks.
# run.launch.py builds the robot's MJCF on every backend (cameras and the world are
# placed from it), so such a robot needs the binding whatever the backend.
MJCF_COMPILED = frozenset({'openarm_anvil', 'axol'})


def skip_reason(robot, backend, has_binding):
    """Return why the row cannot run on this host, or None when it can."""
    mjcf = catalog.load('robots', robot)['mjcf']
    if mjcf in MJCF_COMPILED and not has_binding:
        return f'{robot} compiles its MJCF with the mujoco binding, absent here'
    return None
