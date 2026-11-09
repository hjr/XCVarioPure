"""Legacy log conventions: use independent attitude evidence, preserve valid mappings, JS/Python parity.

Run with: python3 tools/airmass_sim/conversion_test.py
"""
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from sens2rec import nmea_fields, scan_log


def log_fixture(*, gravity=False, kelvin=False, temperature=None, rates=True, flat=False, samples=220, gyro_bias_z=0):
    lines = ["!XCV,FMT,SENS,7,1,-1,-1,1,-1,-1"]
    roll = math.radians(-5)
    for i in range(samples):
        t = i * 0.2
        pitch = 0 if flat else math.radians(8 + 2 * math.sin(t / 2))
        pitch_rate = 0 if flat or not rates else math.cos(t / 2)
        force = [math.sin(pitch), -math.sin(roll)*math.cos(pitch), -math.cos(roll)*math.cos(pitch)]
        acc = [-x for x in force] if gravity else [force[0], -force[1], -force[2]]
        gyro = [0, pitch_rate * math.cos(roll), -pitch_rate * math.sin(roll)]
        gyro[2] += gyro_bias_z
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
console.log(JSON.stringify({axes: !!(c.axesCorrected || c.axesInferred), kelvin: c.temperatureKelvinSamples,
  estimated: c.temperatureEstimatedSamples}));
"""
        js = json.loads(subprocess.check_output(
            ["node", "-e", script], input=text.encode(), cwd=Path(__file__).parent))
        self.assertEqual(js["axes"], bool(py.get("axes_corrected") or py.get("axes_inferred")))
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

    def test_missing_metadata_can_be_inferred_with_independent_evidence(self):
        text = log_fixture(gravity=True, gyro_bias_z=9).split("\n", 1)[1]
        py, js = self.inspect(text)
        self.assertEqual(py["fmt"], (7, [-1, -1, -1, 1, 1, 1]))
        self.assertTrue(py["axes_inferred"])
        self.assertTrue(js["axes"])

    def test_missing_metadata_is_not_guessed_without_evidence(self):
        for args in (dict(gravity=False), dict(gravity=True, samples=10), dict(gravity=True, rates=False), dict(gravity=True, flat=True)):
            with self.subTest(args=args):
                text = log_fixture(**args).split("\n", 1)[1]
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


class FlightDetectionTests(unittest.TestCase):
    def test_flight_detection_does_not_require_ground_speed(self):
        for gnss in (None, 3):
            with self.subTest(ground_speed_knots=gnss), tempfile.TemporaryDirectory() as tmp:
                lines = ["!XCV,CMN,1032,300,100,0,0"]
                for i in range(240):
                    if gnss is not None:
                        lines.append(f"$GPRMC,120000.00,A,4800.000,N,01100.000,E,{gnss},90.0,091126,,,A")
                    lines.append("$PXCV,0,1,0,1,1,15,1013,1000,0,0,0")
                    lines.append(f"$SENS,{300 + i*.1:.3f},0,900,900,500,15,0,0,-1,0,0,0")
                text = "\n".join(lines) + "\n"
                source, output = Path(tmp) / "flight.nmea", Path(tmp) / "replay.nmea"
                source.write_text(text)
                subprocess.run([sys.executable, str(Path(__file__).with_name("sens2rec.py")), str(source), str(output), "--glider", "log"],
                               check=True, capture_output=True)
                script = """
const fs = require('fs'), {convert} = require('./web/convert.js');
const c = convert(fs.readFileSync(0, 'utf8'), {glider: 'log'});
console.log(JSON.stringify({frames: c.frames, starts: c.starts, rec: c.rec}));
"""
                js = json.loads(subprocess.check_output(["node", "-e", script], input=text.encode(), cwd=Path(__file__).parent))
                self.assertEqual(js["starts"], 1)
                self.assertEqual(js["frames"], 141)
                py_rec = output.read_text().splitlines()
                self.assertEqual(len(py_rec), len(js["rec"]))
                for py_line, js_line in zip(py_rec, js["rec"]):
                    py_fields, js_fields = nmea_fields(py_line), nmea_fields(js_line)
                    self.assertEqual(py_fields[0], js_fields[0])
                    # Signed zero has the same meaning, even though its NMEA text/checksum differs.
                    self.assertEqual([float(v) if v else None for v in py_fields[1:]],
                                     [float(v) if v else None for v in js_fields[1:]])


if __name__ == "__main__":
    unittest.main()
