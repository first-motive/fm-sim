"""The simulated depth sensor: noise growing with z squared, dropout, range and the seed."""

from fm_sim_cameras import depth, depth_sensor
import numpy as np
import pytest

# A flat wall large enough for the noise statistics, with no edge inside it.
SHAPE = (400, 400)


def _wall(metres):
    return np.full(SHAPE, metres, np.float32)


def _interior(image):
    """The wall away from its border, which the edge dropout takes (no data past it)."""
    margin = depth_sensor.EDGE_DILATE_PX + 1
    return image[margin:-margin, margin:-margin]


@pytest.mark.parametrize('metres', [0.5, 1.0, 2.0, 3.0])
def test_noise_grows_with_the_square_of_the_depth(metres):
    rng = depth_sensor.frame_rng(1, 'chest', 0)
    measured = _interior(depth_sensor.measure(_wall(metres), rng))
    valid = measured[measured > 0]
    expected = depth_sensor.AXIAL_NOISE_PER_M * metres**2
    # About 150 000 samples: the sample sigma is within 2 % of the model's.
    assert valid.std() == pytest.approx(expected, rel=0.02)
    assert valid.mean() == pytest.approx(metres, abs=3 * expected / np.sqrt(valid.size))


def test_noise_at_two_metres_is_the_d400_order():
    sigma = depth_sensor.AXIAL_NOISE_PER_M * 2.0**2
    # The D400 datasheet: under 2 % at 2 m.
    assert 0.005 < sigma / 2.0 < 0.02


def test_random_dropout_is_the_invalid_rate():
    measured = _interior(depth_sensor.measure(_wall(1.0), depth_sensor.frame_rng(2, 'waist', 0)))
    assert np.mean(measured == 0) == pytest.approx(depth_sensor.INVALID_RATE, rel=0.15)


def test_pixels_near_a_depth_edge_are_dropped():
    # A near box on the left half, the far wall on the right: a 1 m jump at column 200.
    truth = _wall(2.0)
    truth[:, :200] = 1.0
    measured = depth_sensor.measure(truth, depth_sensor.frame_rng(3, 'chest', 0))
    # Columns 199 and 200 jump; the band reaches EDGE_DILATE_PX past each.
    band = slice(199 - depth_sensor.EDGE_DILATE_PX, 200 + depth_sensor.EDGE_DILATE_PX + 1)
    assert np.all(measured[:, band] == 0)
    # Away from the edge, depth survives (but for the random dropout).
    assert np.mean(_interior(measured)[:, 50:150] > 0) > 0.98
    assert np.mean(_interior(measured)[:, 250:350] > 0) > 0.98


def test_a_small_step_is_not_an_edge():
    truth = _wall(1.0)
    truth[:, 200:] += depth_sensor.EDGE_JUMP_M / 2
    assert not depth_sensor.edges(truth).any()


def test_no_data_and_out_of_range_read_zero():
    truth = _wall(1.0)
    truth[:100] = 0.0  # no data: corners, nothing hit
    truth[100:200] = depth_sensor.MIN_RANGE_M / 2
    truth[200:300] = depth_sensor.MAX_RANGE_M + 0.5
    measured = depth_sensor.measure(truth, depth_sensor.frame_rng(4, 'chest', 0))
    assert np.all(measured[:300] == 0)
    assert np.mean(measured[310:] > 0) > 0.98


def test_the_sensor_depth_encodes_like_the_true_depth():
    measured = depth_sensor.measure(_wall(1.5), depth_sensor.frame_rng(5, 'chest', 0))
    decoded = depth.decode(depth.encode(depth.to_millimetres(measured)))
    assert decoded.dtype == np.uint16
    assert np.array_equal(decoded == 0, measured == 0)
    assert np.abs(decoded[measured > 0] / 1000.0 - measured[measured > 0]).max() <= 0.0005 + 1e-6


def test_one_seed_role_and_stamp_reproduce_a_frame():
    truth = _wall(1.2)
    first = depth_sensor.measure(truth, depth_sensor.frame_rng(7, 'wrist_left', 100))
    again = depth_sensor.measure(truth, depth_sensor.frame_rng(7, 'wrist_left', 100))
    assert np.array_equal(first, again)
    for other in (
        depth_sensor.frame_rng(8, 'wrist_left', 100),
        depth_sensor.frame_rng(7, 'wrist_right', 100),
        depth_sensor.frame_rng(7, 'wrist_left', 101),
    ):
        assert not np.array_equal(first, depth_sensor.measure(truth, other))


@pytest.mark.parametrize('seed', [-1, depth_sensor.SEED_MAX + 1, 1.5, True])
def test_frame_rng_refuses_a_bad_seed(seed):
    with pytest.raises(ValueError, match='seed must be an integer'):
        depth_sensor.frame_rng(seed, 'chest', 0)
