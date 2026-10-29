"""Tests for bench_verdict: python3 -m unittest discover -s scripts/native -p 'test_*.py'"""

import os
import tempfile
import unittest

from bench_verdict import GO, NO_GO, NO_VERDICT, load, verdict

CHEST = '/cam_chest/image_raw/compressed'
WRIST = '/cam_wrist_right/image_raw/compressed'


def result(chest, wrist, rtf=1.0):
    return {'ready_s': 2.0, 'rtf': rtf, 'camera_hz': {CHEST: chest, WRIST: wrist}}


class VerdictTest(unittest.TestCase):

    def test_go_when_every_camera_doubles_and_real_time_holds(self):
        self.assertEqual(verdict(result(5, 4), result(10, 8))[0], GO)

    def test_no_go_below_twice_the_container(self):
        self.assertEqual(verdict(result(5, 5), result(9.5, 9.5))[0], NO_GO)

    def test_no_go_when_one_camera_lags_even_if_the_mean_doubles(self):
        self.assertEqual(verdict(result(5, 5), result(30, 6))[0], NO_GO)

    def test_no_go_below_real_time(self):
        self.assertEqual(verdict(result(5, 5), result(10, 10, rtf=0.989))[0], NO_GO)

    def test_no_verdict_when_a_container_camera_is_silent(self):
        self.assertEqual(verdict(result(0, 5), result(0, 10))[0], NO_VERDICT)

    def test_no_verdict_when_a_native_camera_is_silent(self):
        self.assertEqual(verdict(result(5, 5), result(10, 0))[0], NO_VERDICT)

    def test_no_verdict_on_an_error(self):
        self.assertEqual(verdict({'error': 'not ready'}, result(10, 10))[0], NO_VERDICT)
        self.assertEqual(verdict(result(5, 5), {'error': 'no output'})[0], NO_VERDICT)

    def test_no_verdict_when_the_cameras_differ(self):
        native = {'ready_s': 2.0, 'rtf': 1.0, 'camera_hz': {CHEST: 10}}
        self.assertEqual(verdict(result(5, 5), native)[0], NO_VERDICT)

    def test_no_verdict_without_cameras(self):
        empty = {'ready_s': 2.0, 'rtf': 1.0, 'camera_hz': {}}
        self.assertEqual(verdict(empty, empty)[0], NO_VERDICT)


class LoadTest(unittest.TestCase):

    def load_text(self, text):
        with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as f:
            f.write(text)
        try:
            return load(f.name)
        finally:
            os.unlink(f.name)

    def test_reads_the_last_line(self):
        self.assertEqual(self.load_text('noise\n{"rtf": 1.0}\n'), {'rtf': 1.0})

    def test_empty_output_is_an_error(self):
        self.assertIn('error', self.load_text(''))

    def test_a_traceback_is_an_error_not_a_crash(self):
        self.assertIn('error', self.load_text('Traceback ...\nIndexError: list index out of range\n'))

    def test_a_missing_file_is_an_error(self):
        self.assertIn('error', load('/nonexistent/fm-sim-bench.json'))

    def test_no_go_has_its_own_exit_code(self):
        self.assertNotIn(NO_GO, (0, 1, NO_VERDICT))


if __name__ == '__main__':
    unittest.main()
