import unittest

from zhenshi_driver_py.backends import DryRunBackend


class DryRunBackendTests(unittest.TestCase):
    def test_dry_run_records_commands(self):
        backend = DryRunBackend(model_count=2)
        result = backend.send_command("{Enable||NotRunExecute}", 5000)
        self.assertTrue(result.success)
        self.assertEqual(
            backend.history,
            ["{Enable||NotRunExecute}"],
        )

    def test_dry_run_state_is_explicit(self):
        backend = DryRunBackend(model_count=2)
        state = backend.read_state(1)
        self.assertTrue(state.data_valid)
        self.assertFalse(state.connected)
        self.assertEqual(state.source, "dry_run")
        self.assertEqual(state.model_index, 1)
        self.assertEqual(len(state.jointtarget), 7)


if __name__ == "__main__":
    unittest.main()
