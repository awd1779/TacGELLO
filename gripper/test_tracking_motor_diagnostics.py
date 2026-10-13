import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


class TrackingDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).with_name('bluetooth_anyskin_gripper_shell.py')
        tree = ast.parse(source.read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                   and n.name == 'RosPositionGripperController')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                      and n.name == '_read_tracking_position')
        scope = dict(json=json, time=SimpleNamespace(monotonic_ns=lambda: 10**9))
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), scope)
        self.read = scope['_read_tracking_position']
        self.o = SimpleNamespace(args=SimpleNamespace(regrasp_diagnostics='true'),
            tracker=SimpleNamespace(contact=False, target=(1750, 2250), desired=.5),
            client=Mock(), node=Mock(), String=lambda: SimpleNamespace(),
            timing_publisher=Mock())
        self.o.client.read_diagnostics.return_value = dict(
            id=1, ok=True, actual_raw=1753, goal_raw=1750, load_raw=32,
            torque_enable_raw=1, mode_raw=0)

    def test_same_timeout_single_transaction_and_no_motion(self):
        self.assertEqual(self.read(self.o, 1, .05), 1753)
        self.o.client.read_diagnostics.assert_called_once_with(1, timeout=.05)
        self.assertEqual(len(self.o.client.mock_calls), 1)
        record = json.loads(self.o.timing_publisher.publish.call_args.args[0].data)
        self.assertEqual(record['phase'], 'tracking')
        self.assertEqual(record['commanded_target_raw'], [1750, 2250])
        self.assertEqual(record['torque_enable_raw'], 1)

    def test_missing_reply_is_explicit_without_retry_or_disable(self):
        self.o.client.read_diagnostics.return_value = None
        self.assertIsNone(self.read(self.o, 1, .05))
        self.assertEqual(len(self.o.client.mock_calls), 1)
        self.assertFalse(self.o.tracking_motor_records[1]['diagnostic_available'])
        self.assertFalse(getattr(self.o, 'diagnostics_unavailable', False))

    def test_contact_does_not_overwrite_precontact_snapshot(self):
        self.read(self.o, 1, .05)
        saved = dict(self.o.tracking_motor_records[1])
        self.o.tracker.contact = True
        self.o.client.read_diagnostics.return_value['actual_raw'] = 1754
        self.read(self.o, 1, .05)
        self.assertEqual(self.o.tracking_motor_records[1], saved)
        record = json.loads(self.o.timing_publisher.publish.call_args.args[0].data)
        self.assertEqual(record['phase'], 'contact_paused')

    def test_disabled_uses_original_read(self):
        self.o.args.regrasp_diagnostics = 'false'
        self.o.client.read_position.return_value = 1753
        self.assertEqual(self.read(self.o, 1, .05), 1753)
        self.o.client.read_position.assert_called_once_with(1, timeout=.05)
        self.o.client.read_diagnostics.assert_not_called()

    def test_log_throttled_but_every_poll_published(self):
        self.read(self.o, 1, .05)
        self.read(self.o, 1, .05)
        self.assertEqual(self.o.node.get_logger.return_value.info.call_count, 1)
        self.assertEqual(self.o.timing_publisher.publish.call_count, 2)


if __name__ == '__main__':
    unittest.main()
