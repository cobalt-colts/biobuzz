"""Run with Python, numpy and opencv-python-headless: python -m unittest discover -s tests."""
import importlib.util
import math
from pathlib import Path
import re
import unittest

import cv2
import numpy as np

UTIL = Path(__file__).resolve().parents[1] / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/util'
spec = importlib.util.spec_from_file_location('snapscript', UTIL / 'snapscript.py')
script = importlib.util.module_from_spec(spec)
spec.loader.exec_module(script)
CONSTANTS = (UTIL / 'Constants.java').read_text()


def constant(name):
    return float(re.search(r'\b' + name + r'\s*=\s*([\d.]+)', CONSTANTS)[1])


class SnapScriptContractTest(unittest.TestCase):
    def check_payload(self, output):
        self.assertEqual(len(output), 8)
        self.assertTrue(all(type(value) is float and math.isfinite(value) for value in output))
        self.assertEqual(output[int(constant('PYTHON_PROTOCOL_INDEX'))], constant('PYTHON_PROTOCOL_ID'))

    def test_empty_frames(self):
        for frame in (None, np.zeros((0, 0, 3), np.uint8), np.zeros((480, 640, 3), np.uint8)):
            contour, _, output = script.runPipeline(frame, [])
            self.check_payload(output)
            self.assertEqual(output[int(constant('CLUSTER_BLOB_COUNT_INDEX'))], 0)
            self.assertEqual(contour.size, 0)

    def test_multiple_blobs_and_clusters(self):
        for width, height in ((640, 480), (1280, 960)):
            hsv = np.zeros((height, width, 3), np.uint8)
            # Two close blobs and one separate cluster, all within configured HSV limits.
            for x, y in ((0.25, 0.5), (0.35, 0.5), (0.85, 0.2)):
                cv2.circle(hsv, (int(x * width), int(y * height)), 18, (25, 240, 220), -1)
            contour, _, output = script.runPipeline(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), [])
            self.check_payload(output)
            self.assertEqual(output[0], 3)
            self.assertEqual(output[int(constant('CLUSTER_BLOB_COUNT_INDEX'))], 2)
            self.assertGreater(cv2.contourArea(contour), 0)
            self.assertAlmostEqual(output[int(constant('NORMALIZED_CLUSTER_X_INDEX'))], 0.30, delta=0.01)
            self.assertAlmostEqual(output[int(constant('NORMALIZED_CLUSTER_Y_INDEX'))], 0.50, delta=0.01)


if __name__ == '__main__':
    unittest.main()
