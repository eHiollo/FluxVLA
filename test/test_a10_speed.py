"""Controller speed transactions against loopback hardware; no robot endpoints."""
import unittest

from test_a10_operator import FakeController, status_reply


class SpeedOverrideTest(unittest.TestCase):
    def controller(self, controller, mode='Manual'):
        values = dict(percent=40, mode=mode, moving=False, ignore=False)
        def get():
            reply = status_reply(active=controller.active, mode=controller.mode)
            ctx = reply['ret_context']
            ctx['robot_msg'].update(op_mode=values['mode'], motion='Moving' if values['moving'] else 'Stopped')
            ctx['motion_msg']['jog_vel'] = values['percent']
            ctx['program_msg'] = dict(vel=values['percent'])
            return reply
        controller.command_replies['get'] = get
        for command in ('set_jog_vel', 'set_pgm_vel'):
            def update():
                if not values['ignore']: values['percent'] = 25
                return dict(ret_code=0, ret_context={}, ret_msg='')
            controller.command_replies[f'{command} --vel_percent=25'] = update
        return values

    def test_direct_manual_and_auto_commands_and_readback(self):
        for mode, command in [('Manual', 'set_jog_vel'), ('Auto', 'set_pgm_vel')]:
            with self.subTest(mode=mode), FakeController() as controller:
                self.controller(controller, mode)
                robot = controller.operator()
                self.assertEqual(robot.get_speed_override(), dict(mode=mode, percent=40))
                self.assertEqual(robot.set_speed_override(25, expected_mode=mode, expected_percent=40),
                                 dict(mode=mode, percent=25))
                self.assertEqual(controller.commands.count(f'{command} --vel_percent=25'), 1)
                self.assertFalse(any(c.startswith('follower_') for c in controller.commands))

    def test_refuse_moving_active_mode_or_percentage_changes(self):
        for case in ('moving', 'active', 'mode', 'percent'):
            with self.subTest(case=case), FakeController() as controller:
                values = self.controller(controller)
                robot = controller.operator()
                if case == 'moving': values['moving'] = True
                if case == 'active': controller.active = True
                if case == 'mode': values['mode'] = 'Auto'
                if case == 'percent': values['percent'] = 30
                with self.assertRaises(RuntimeError):
                    robot.set_speed_override(25, expected_mode='Manual', expected_percent=40)
                self.assertFalse(any(c.startswith('set_') for c in controller.commands))

    def test_readback_timeout_sends_only_once(self):
        with FakeController() as controller:
            values = self.controller(controller)
            values['ignore'] = True
            robot = controller.operator(timeout_s=.1)
            with self.assertRaises(TimeoutError):
                robot.set_speed_override(25, expected_mode='Manual')
            self.assertEqual(controller.commands.count('set_jog_vel --vel_percent=25'), 1)

    def test_invalid_percent_or_mode_no_commands(self):
        with FakeController() as controller:
            self.controller(controller)
            robot = controller.operator()
            for value in (True, -1, 101, float('nan')):
                with self.subTest(value=value), self.assertRaises(ValueError):
                    robot.set_speed_override(value, expected_mode='Manual')
            with self.assertRaises(ValueError): robot.set_speed_override(25, expected_mode='Bad')
            self.assertEqual(controller.commands, [])


if __name__ == '__main__':
    unittest.main()
