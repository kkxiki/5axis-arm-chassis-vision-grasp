import math
import unittest

from zhenshi_driver_py.commands import (
    build_gripper,
    build_joint_jog,
    build_move_joint,
    build_move_pose_sequence,
    normalize_quaternion,
    scope_command,
)


class CommandTests(unittest.TestCase):
    def test_scope_first_model(self):
        self.assertEqual(
            scope_command("Enable", 0, 2),
            "{Enable||NotRunExecute}",
        )

    def test_scope_second_model(self):
        self.assertEqual(
            scope_command("Enable", 1, 2),
            "{NotRunExecute||Enable}",
        )

    def test_move_joint_requires_real_joint_count(self):
        with self.assertRaisesRegex(ValueError, "expected 7"):
            build_move_joint(0, 2, [0.0] * 10, 7)

    def test_move_joint_pads_controller_slots_and_sets_slow_speed(self):
        command = build_move_joint(1, 2, [0.0] * 7, 7, 10, 0.01)
        self.assertTrue(command.startswith("{NotRunExecute||MoveAbsJ"))
        self.assertIn(
            "jointtarget_value={0,0,0,0,0,0,0,0,0,0}",
            command,
        )
        self.assertIn("--speed_value=0.01 --zone=fine", command)

    def test_joint_jog_changes_only_selected_real_joint(self):
        command = build_joint_jog(
            0, 2, [0.0] * 7, 6, math.radians(0.5), 7
        )
        self.assertIn(
            "jointtarget_value={0,0,0,0,0,0,0.00872664625997,0,0,0}",
            command,
        )

    def test_joint_jog_rejects_step_over_half_degree(self):
        with self.assertRaisesRegex(ValueError, "safety|within"):
            build_joint_jog(
                0, 2, [0.0] * 7, 6, math.radians(0.51), 7
            )

    def test_quaternion_is_normalized(self):
        value = normalize_quaternion([0.0, 0.0, 0.0, 1.01])
        self.assertTrue(math.isclose(value[3], 1.0))

    def test_pose_rejects_outside_workspace(self):
        with self.assertRaisesRegex(ValueError, "outside"):
            build_move_pose_sequence(
                0,
                2,
                [2.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
                [0.1, 0.1],
                [0.0, -0.7, -0.3],
                [1.0, 0.7, 0.5],
            )

    def test_pose_builds_six_scoped_commands(self):
        commands = build_move_pose_sequence(
            1,
            2,
            [0.5, 0.0, 0.1],
            [0.0, 0.0, 0.0, 1.0],
            [0.1, 0.1],
            [0.0, -0.7, -0.3],
            [1.0, 0.7, 0.5],
        )
        self.assertEqual(len(commands), 6)
        self.assertTrue(
            all(
                command.startswith("{NotRunExecute||")
                for command in commands
            )
        )

    def test_gripper_states(self):
        for state in (0, 1, 2):
            with self.subTest(state=state):
                self.assertIn(
                    f"state={state}",
                    build_gripper(0, 2, state),
                )

    def test_gripper_rejects_unknown_state(self):
        with self.assertRaises(ValueError):
            build_gripper(0, 2, 3)


if __name__ == "__main__":
    unittest.main()
