"""Standard-library tests; optionally also validate using RobotPy's official DataLogReader."""
import csv
import importlib.util
import io
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import convert_logs_2026 as converter


def read_records(path):
    """Independent reader uses variable widths (unlike the converter's fixed-width writer)."""
    data = io.BytesIO(path.read_bytes())
    assert data.read(6) == b"WPILOG"
    version, extra_size = struct.unpack("<HI", data.read(6))
    assert version == 0x0100
    data.read(extra_size)
    entries, result = {}, []
    while header := data.read(1):
        flags = header[0]
        entry = int.from_bytes(data.read((flags & 3) + 1), "little")
        size = int.from_bytes(data.read(((flags >> 2) & 3) + 1), "little")
        timestamp = int.from_bytes(data.read(((flags >> 4) & 7) + 1), "little")
        payload = data.read(size)
        assert len(payload) == size
        if entry == 0:
            assert payload[0] == 0
            control = io.BytesIO(payload[1:])
            entry_id = struct.unpack("<I", control.read(4))[0]
            strings = []
            for _ in range(3):
                length = struct.unpack("<I", control.read(4))[0]
                strings.append(control.read(length).decode())
            entries[entry_id] = strings[:2]
        else:
            name, kind = entries[entry]
            result.append((name, kind, timestamp, payload))
    return result


class ConvertTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.output = self.root / "run.wpilog"

    def tearDown(self):
        self.directory.cleanup()

    def csv(self, name="match.csv", session="one", epoch=100000, detail=False):
        fields = ["schema_version", "session_id", "phase", "match_t_ms", "epoch_ms", "event", "note",
                  "robot_x_in", "robot_y_in", "robot_heading_deg", "turret_robot_relative_deg",
                  "sequence_id", "target_x_in", "target_y_in", "ball_count"]
        if detail:
            fields.append("controller_power_before_boost")
        path = self.root / name
        with path.open("w",newline="",encoding="utf-8") as stream:
            writer = csv.DictWriter(stream,fields)
            writer.writeheader()
            for time, event, sequence in ((0,"SESSION_START",0),(20,"SHOT_START",1),(40,"SESSION_END",1)):
                row = dict(zip(fields,[1,session,"AUTO",time,epoch+time,event,'note, "quoted"\nline',
                                       72,72,0,180,sequence,58,60,4]))
                if detail:
                    row["controller_power_before_boost"] = .48
                writer.writerow(row)
        return path

    def test_modern_geometry_and_units(self):
        path = self.csv()
        self.assertEqual(converter.convert([path],self.output),3)
        records = read_records(self.output)
        self.assertTrue(any(r[:2]==("/.schema/struct:Pose2d","structschema") for r in records))
        pose = next(r for r in records if r[0]=="/Replay/RobotPose")
        self.assertEqual(pose[1],"struct:Pose2d")
        x,y,h = struct.unpack("<ddd",pose[3])
        self.assertAlmostEqual(x,0); self.assertAlmostEqual(y,0); self.assertAlmostEqual(h,converter.math.pi/2)
        shots = [r for r in records if r[0]=="/Replay/CommandedVolleyLocations"]
        self.assertEqual(len(shots),1)
        self.assertEqual(shots[0][2],20000)
        self.assertEqual(len(shots[0][3]),24)
        self.assertTrue(any(r[0]=="/Match/note" and b'"quoted"\n' in r[3] for r in records))
        if importlib.util.find_spec("wpiutil"):
            from wpiutil.log import DataLogReader
            reader = DataLogReader(str(self.output))
            self.assertTrue(reader.isValid())
            official = list(reader)
            self.assertTrue(len(official)>len(records))

    def test_detail_merge_and_types(self):
        match, detail = self.csv(), self.csv("shot_detail.csv",detail=True)
        self.assertEqual(converter.convert([match,detail],self.output),6)
        records = read_records(self.output)
        self.assertTrue(any(r[0]=="/ShotDetail/controller_power_before_boost" for r in records))
        self.assertEqual(len([r for r in records if r[0]=="/Replay/CommandedVolleyLocations"]),1)

    def test_cross_run_requires_wall_timeline(self):
        a,b=self.csv(),self.csv("teleop.csv",session="two",epoch=110000)
        with self.assertRaises(ValueError): converter.convert([a,b],self.output)
        converter.convert([a,b],self.output,timeline="wall")
        records=read_records(self.output)
        self.assertEqual(len([r for r in records if r[0]=="/Replay/CommandedVolleyLocations"]),2)
        self.assertTrue(any(r[2]==10000000 for r in records))

    def test_detail_only_replay_and_raw_pedro_frame(self):
        path=self.csv("shot_detail.csv",detail=True)
        converter.convert([path],self.output,frame="pedro")
        pose=next(r for r in read_records(self.output) if r[0]=="/Replay/RobotPose")
        self.assertEqual(struct.unpack("<ddd",pose[3]),(72*.0254,72*.0254,0))

    def test_incomplete_tail_recovery(self):
        path=self.csv()
        with path.open("a") as stream: stream.write("1,one,AUTO,100")
        self.assertEqual(converter.convert([path],self.output),3)

    def test_no_time_reordering(self):
        path=self.csv()
        with path.open("a",newline="") as stream:
            stream.write('1,one,AUTO,5,100005,SAMPLE,note,72,72,0,180,1,58,60,4\n')
        with self.assertRaises(ValueError): converter.convert([path],self.output)


if __name__=="__main__": unittest.main()
