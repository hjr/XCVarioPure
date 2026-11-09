"""Legacy log conventions: use independent attitude evidence, preserve valid mappings, JS/Python parity.

Run with: python3 tools/airmass_sim/conversion_test.py
"""
import json
import math
from pathlib import Path
import subprocess
import unittest

from sens2rec import scan_log


def log_fixture(*, gravity=False, kelvin=False, temperature=None, rates=True, flat=False, samples=220):
    lines = ["!XCV,FMT,SENS,7,1,-1,-1,1,-1,-1"]
    roll = math.radians(-5)
    for i in range(samples):
        t = i * 0.2
        pitch = 0 if flat else math.radians(8 + 2 * math.sin(t / 2))
        pitch_rate = 0 if flat or not rates else math.cos(t / 2)
        force = [math.sin(pitch), -math.sin(roll)*math.cos(pitch), -math.cos(roll)*math.cos(pitch)]
        acc = [-x for x in force] if gravity else [force[0], -force[1], -force[2]]
        gyro = [0, pitch_rate * math.cos(roll), -pitch_rate * math.sin(roll)]
        if not gravity:
            gyro = [gyro[0], -gyro[1], -gyro[2]]
        oat = temperature if temperature is not None else (285.59 if kelvin else 12.44)
        lines.append("$SENS," + ",".join(map(str, [f"{t:.3f}", 0, 1000, 1000, 0, oat] + acc + gyro)))
        lines.append("$PXCV,0,1,0,1,1,12.44,1013,1000,0," + f"{math.degrees(roll)},{math.degrees(pitch)}")
    return "\n".join(lines) + "\n"


class ConventionTests(unittest.TestCase):
    def inspect(self, text):
        py = scan_log(text.splitlines())
        script = """
const fs = require('fs'), {convert} = require('./web/convert.js');
const c = convert(fs.readFileSync(0, 'utf8'));
console.log(JSON.stringify({axes: !!c.axesCorrected, kelvin: c.temperatureKelvinSamples,
  estimated: c.temperatureEstimatedSamples}));
"""
        js = json.loads(subprocess.check_output(
            ["node", "-e", script], input=text.encode(), cwd=Path(__file__).parent))
        self.assertEqual(js["axes"], bool(py.get("axes_corrected")))
        self.assertEqual(js["kelvin"] > 0, py["temperature_kelvin"])
        return py, js

    def test_inconsistent_metadata_is_repaired(self):
        py, js = self.inspect(log_fixture(gravity=True, kelvin=True))
        self.assertEqual(py["fmt"], (7, [-1, -1, -1, 1, 1, 1]))
        self.assertEqual(js, dict(axes=True, kelvin=220, estimated=0))

    def test_valid_legacy_mapping_is_preserved_even_with_kelvin(self):
        py, js = self.inspect(log_fixture(kelvin=True))
        self.assertEqual(py["fmt"], (7, [1, -1, -1, 1, -1, -1]))
        self.assertFalse(js["axes"])
        self.assertEqual(js["kelvin"], 220)

    def test_celsius_unchanged(self):
        _, js = self.inspect(log_fixture())
        self.assertEqual(js, dict(axes=False, kelvin=0, estimated=0))

    def test_no_axis_declaration_is_invented(self):
        text = log_fixture(gravity=True).split("\n", 1)[1]
        py, js = self.inspect(text)
        self.assertIsNone(py["fmt"])
        self.assertFalse(js["axes"])

    def test_bad_oat_keeps_fallback(self):
        for oat in (84.9, 310):
            with self.subTest(oat=oat):
                _, js = self.inspect(log_fixture(temperature=oat))
                self.assertEqual(js["kelvin"], 0)
                self.assertEqual(js["estimated"], 220)

    def test_insufficient_axis_evidence_preserves_declaration(self):
        for args in (dict(samples=10), dict(rates=False), dict(flat=True)):
            with self.subTest(args=args):
                _, js = self.inspect(log_fixture(gravity=True, **args))
                self.assertFalse(js["axes"])


if __name__ == "__main__":
    unittest.main()
