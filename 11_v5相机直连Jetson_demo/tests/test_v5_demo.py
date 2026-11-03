from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
import sys
sys.path.insert(0, str(ROOT))

from v5_robot.config import ConfigError, load_config, validate_config
from v5_robot.devices import build_devices
from v5_robot.mission import MissionRunner
from v5_robot.models import Pose2D, Stage
from v5_robot.front_geometry import bearing_from_pixel, normalized_bbox_to_pixels, robust_depth_m
from v5_robot.controller_api import event_payload
from v5_robot.motion_commands import blend_l, move_blend
from v5_robot.bottle_plan import (BottlePlanError, build_bottle_pick_plan,
                                  camera_point_to_arm_base, pixel_depth_to_camera,
                                  pixel_ray_to_arm_base_plane)


class V5DemoTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(ROOT / "config" / "v5_config.json")
        validate_config(self.cfg, "sim")

    def test_complete_simulated_mission(self):
        with tempfile.TemporaryDirectory() as folder:
            devices = build_devices(self.cfg, "sim")
            result = MissionRunner(self.cfg, devices, folder).run(Pose2D(2.0, 0.8, 0.0), "bottle")
            self.assertTrue(result.success)
            self.assertEqual(result.final_stage, Stage.COMPLETE)
            self.assertTrue(devices.world.placed)
            events = [json.loads(line)["event"] for line in Path(result.log_file).read_text(encoding="utf-8").splitlines()]
            self.assertIn("front_detection", events)
            self.assertIn("wrist_detection", events)
            self.assertIn("place_complete", events)

    def test_fault_enters_safe_abort(self):
        with tempfile.TemporaryDirectory() as folder:
            devices = build_devices(self.cfg, "sim", fail_at="wrist_detect")
            result = MissionRunner(self.cfg, devices, folder).run(Pose2D(2.0, 0.8, 0.0), "bottle")
            self.assertFalse(result.success)
            self.assertEqual(result.final_stage, Stage.SAFE_ABORT)

    def test_real_mode_rejects_unconfirmed_calibration(self):
        with self.assertRaises(ConfigError):
            validate_config(self.cfg, "real")

    def test_front_camera_geometry_and_depth_filter(self):
        box = normalized_bbox_to_pixels(0.5, 0.5, 0.2, 0.4, 640, 400)
        self.assertEqual(box, (256, 120, 384, 280, 320, 200))
        self.assertAlmostEqual(bearing_from_pixel(320, 500, 320), 0.0)
        self.assertAlmostEqual(robust_depth_m([0, 975, 980, 10000], 0.001), 0.9775)

    def test_controller_event_uses_sub_commands(self):
        value = event_payload("MoveTwoFingersGripper", distance=70, interval=25)
        self.assertEqual(value["user_action"], "Click")
        self.assertEqual(value["sub_commands"],
                         [{"selected": True, "distance": 70, "interval": 25}])
        self.assertNotIn("common_parame", value)

    def test_v5_motion_uses_blendl_inside_moveblend_only(self):
        target = [0.4, -0.1, 0.3, 0.0, 1.0, 0.0, 0.0]
        commands = move_blend([target, target], "v25")
        self.assertEqual(commands[0], "{MoveBlend --type=first_insert}")
        self.assertEqual(commands[-1], "{MoveBlend --type=start}")
        self.assertEqual(sum("type=insert_line" in value for value in commands), 2)
        self.assertIn("--speed=v25", blend_l(target, "v25"))
        self.assertNotIn("MoveAbsJ", "\n".join(commands))

    def test_v4_eye_in_hand_transform(self):
        p = camera_point_to_arm_base(
            (0.1, 0.2, 0.3),
            (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0),
            [[1, 0, 0, 0.01], [0, 1, 0, 0.02], [0, 0, 1, 0.03], [0, 0, 0, 1]],
        )
        self.assertEqual(tuple(round(v, 6) for v in p), (1.11, 2.22, 3.33))
        self.assertEqual(pixel_depth_to_camera(320, 240, 1.0,
                         [[500, 0, 320], [0, 500, 240], [0, 0, 1]]), (0.0, 0.0, 1.0))

    def test_fixed_height_rgb_ray_intersection(self):
        p = pixel_ray_to_arm_base_plane(
            320, 240, 0.5,
            [[500, 0, 320], [0, 500, 240], [0, 0, 1]],
            [0, 0, 0, 0, 0, 0, 1],
            [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.1], [0, 0, 0, 1]],
        )
        self.assertEqual(tuple(round(v, 6) for v in p), (0.0, 0.0, 0.5))

    def test_v4_bottle_plan_requires_new_taught_poses(self):
        cfg = json.loads((ROOT / "config" / "bottle_demo_v5.json").read_text(encoding="utf-8"))
        # Keep this guard test independent of the deployment's real teaching
        # progress: explicitly mark one required point as unconfirmed.
        cfg["poses"]["home"]["confirmed"] = False
        with self.assertRaises(BottlePlanError):
            build_bottle_pick_plan(cfg, (0.4, 0.0, 0.2))

    def test_v4_bottle_plan_uses_v3_style_reference_compensation(self):
        pose = {"confirmed": True, "robot_model": "taiKeU058T_0",
                "robottarget": [0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0]}
        cfg = {"robot_model": "taiKeU058T_0", "grasp_z_extra_m": -0.01,
               "lift_height_m": 0.15,
               "poses": {name: dict(pose) for name in
                         ("observe", "above_ref", "grasp_ref", "place_above", "place", "home")}}
        cfg["poses"]["grasp_ref"]["ref_detection_base_m"] = [0.5, 0.0, 0.2]
        plan = build_bottle_pick_plan(cfg, (0.52, -0.01, 0.21))
        self.assertEqual(tuple(round(v, 3) for v in plan.compensation_delta_m), (0.02, -0.01, 0.01))
        self.assertEqual(tuple(round(v, 3) for v in plan.grasp.to_list()[:3]), (0.42, -0.01, 0.3))


if __name__ == "__main__":
    unittest.main()
