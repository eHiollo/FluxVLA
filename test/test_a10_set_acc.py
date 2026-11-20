"""Motor acceleration command tests; in-memory and loopback only."""
import argparse
import copy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from scripts.a10_control import A10Console, parse_command, CommandError
from fluxvla.engines.operators.kaanh_commands.set_acc import set_acceleration


def motors():
    return [dict(slave_id=i+2,motion_type='rotatory',direction=-1 if i in (2,8) else 1,
                 max_acc=400.,min_acc=-400.,max_vel=150. if i%6<3 else 180.,
                 min_vel=-150. if i%6<3 else -180.,max_pos=175.,min_pos=-175.,
                 mode='位置',resolution=524288) for i in range(12)]


class Robot:
    allow_motion=True
    shared_control={'role':'console'}
    command_timeout_s=.02
    is_connected=True
    is_multi_arm=True
    def __init__(self):
        self.data=motors();self.program='Running';self.activate='Enabled'
        self.motion='Running';self.active=False;self.user='Manufacturer';self.mode='Manual'
        self.calls=[];self.depth=0;self.fail=None;self.corrupt=False;self.cancelled=False
    def get_state(self):
        return SimpleNamespace(has_error=False,follower_active=self.active,raw=dict(ret_context=dict(
            robot_msg=dict(activate=self.activate,motion=self.motion,op_mode=self.mode),
            program_msg=dict(state=self.program),user_management_msg=dict(user=self.user))))
    def read_motor_config(self):return copy.deepcopy(self.data)
    def begin_shared_console_operation(self):self.depth+=1
    def end_shared_console_operation(self):self.depth-=1
    def disable(self):self.calls.append('manual_ds');self.activate='Disabled'
    def send_controller_command(self,command):
        if self.cancelled:raise InterruptedError('cancelled')
        self.calls.append(command)
        if command==self.fail:raise TimeoutError('unknown result')
        if command=='pgmctrl --stop':self.motion='Stopped';self.program='Stopped'
        elif command=='pgmctrl --unload':
            if self.fail=='unload_rejected':return {'ret_code':-1}
            if self.fail!='unload_no_effect':self.program='Unloaded'
        elif command=='stop_follower':self.active=False;self.motion='Stopped'
        else:
            assert command.startswith('write_motor --id=')
            assert self.activate=='Disabled' and self.program=='Unloaded' and not self.active
            i=int(command.split('--id=')[1].split()[0]);row=json.loads(command.split('--data=')[1])
            self.data[i]=row
            if self.fail=='write_timeout':raise TimeoutError('ACK lost after apply')
            if self.corrupt:self.data[i]['direction']=-self.data[i]['direction']
        return {'ret_code':0,'ret_context':'','ret_msg':''}


class SetAccTests(unittest.TestCase):
    def run_command(self,robot,value=200,**kw):
        return set_acceleration(robot,value,output=lambda _:None,backup_root=self.tmp.name,**kw)
    def setUp(self):self.tmp=TemporaryDirectory()
    def tearDown(self):self.tmp.cleanup()
    def test_prepares_unloaded_disabled_and_verifies_all_fields(self):
        r=Robot();before=copy.deepcopy(r.data);result=self.run_command(r)
        self.assertTrue(result['success']);self.assertEqual(result['completed_motor_ids'],list(range(12)))
        self.assertEqual(r.calls[:3],['pgmctrl --stop','pgmctrl --unload','manual_ds'])
        for a,b in zip(r.data,before):
            self.assertEqual(a,dict(b,max_acc=200.,min_acc=-200.))
        self.assertEqual((r.program,r.activate,r.depth),('Unloaded','Disabled',0))
        self.assertFalse(any(c.startswith(('manual_en','follower_aj')) for c in r.calls))
        backup=Path(result['backup_dir'])
        self.assertEqual(json.loads((backup/'before.json').read_text()),before)
        self.assertTrue((backup/'verification.json').exists())
    def test_single_axis_preserves_other_axes_and_already_prepared_state(self):
        r=Robot();r.program='Unloaded';r.motion='Stopped';r.activate='Disabled';before=copy.deepcopy(r.data)
        result=self.run_command(r,700,motor_id=8)
        self.assertEqual(len(r.calls),1);self.assertEqual(result['completed_motor_ids'],[8])
        for i,row in enumerate(r.data):self.assertEqual(row,dict(before[i],max_acc=700.,min_acc=-700.) if i==8 else before[i])
    def test_read_only_wrong_role_permissions_and_mode_do_not_control_robot(self):
        for key,value in [('allow_motion',False),('shared_control',{'role':'inference'}),('user','Operator'),('mode','Auto')]:
            r=Robot();setattr(r,key,value)
            with self.subTest(key=key),self.assertRaises((PermissionError,RuntimeError)):self.run_command(r)
            self.assertEqual(r.calls,[]);self.assertEqual(r.depth,0)
    def test_unload_rejected_or_not_confirmed_never_writes(self):
        for failure in ('unload_rejected','unload_no_effect'):
            r=Robot();r.fail=failure
            with self.subTest(failure=failure),self.assertRaises((RuntimeError,TimeoutError)):self.run_command(r)
            self.assertFalse(any(c.startswith('write_motor') for c in r.calls));self.assertEqual(r.depth,0)
    def test_unknown_write_result_not_retried_or_rolled_back(self):
        r=Robot();r.fail='write_timeout'
        with self.assertRaises(TimeoutError):self.run_command(r)
        self.assertEqual(sum(c.startswith('write_motor') for c in r.calls),1)
        self.assertEqual(r.data[0]['max_acc'],200.);self.assertEqual(r.data[1]['max_acc'],400.)
        error=json.loads(next(Path(self.tmp.name).glob('*/error.json')).read_text())
        self.assertEqual(error['completed_motor_ids'],[]);self.assertEqual(r.depth,0)
    def test_non_acceleration_change_stops_remaining_writes(self):
        r=Robot();r.corrupt=True
        with self.assertRaisesRegex(RuntimeError,'direction'):self.run_command(r)
        self.assertEqual(sum(c.startswith('write_motor') for c in r.calls),1)
        self.assertEqual(r.depth,0)
    def test_active_follower_stopped_before_program_preparation(self):
        r=Robot();r.active=True;self.run_command(r,motor_id=0)
        self.assertEqual(r.calls[0],'stop_follower')
    def test_cancelled_command_never_writes(self):
        r=Robot();r.cancelled=True
        with self.assertRaises(InterruptedError):self.run_command(r)
        self.assertEqual(r.calls,[]);self.assertEqual(r.depth,0)
    def test_parser_rejects_bad_acceleration_and_ids(self):
        for tokens in [['set_acc','0'],['set_acc','-1'],['set_acc','nan'],['set_acc','inf'],
                       ['set_acc','400','--id','12'],['set_acc','400','--all','--id','0']]:
            with self.subTest(tokens=tokens),self.assertRaises(CommandError):parse_command(tokens)
        for tokens,expected in [(['set_acc','400'],None),(['set_acc','400','--id=0'],0),(['set_acc','400','--all'],None)]:
            a=parse_command(tokens);self.assertEqual(a.command,'set_acc');self.assertEqual(a.motor_id,expected)
    def test_local_console_routes_to_command_without_raw_passthrough(self):
        r=Robot();output=[]
        with patch('fluxvla.engines.operators.kaanh_commands.set_acc.set_acceleration',return_value={'success':True}) as apply:
            A10Console(r,output.append).execute(parse_command(['set_acc','400','--id=0']))
        apply.assert_called_once_with(r,400.,motor_id=0,output=output.append)
        self.assertEqual(r.calls,[]);self.assertEqual(r.depth,0)
    def test_real_operator_ws_full_json_writes_and_readback(self):
        from test_a10_operator import FakeController,DUAL_ARMS,dual_motion,status_reply
        data=motors();state={'program':'Running','motion':'Running'}
        with FakeController() as c:
            def status():
                reply=status_reply(active=False,mode='None');ctx=reply['ret_context']
                ctx['motion_msg'].update(dual_motion())
                ctx['robot_msg'].update(activate=c.activation,motion=state['motion'],op_mode='Manual')
                ctx['program_msg']={'state':state['program'],'vel':50}
                ctx['user_management_msg']={'user':'Manufacturer'}
                return reply
            def stop():state.update(program='Stopped',motion='Stopped');return {'ret_code':0,'ret_context':''}
            def unload():state['program']='Unloaded';return {'ret_code':0,'ret_context':''}
            want=dict(data[0],max_acc=300.,min_acc=-300.)
            command='write_motor --id=0 --data='+json.dumps(want,ensure_ascii=False,separators=(',',':'))
            def write():
                self.assertEqual((state['program'],c.activation),('Unloaded','Disabled'))
                data[0]=want.copy();return {'ret_code':0,'ret_context':''}
            c.command_replies.update({'get':status,'pgmctrl --stop':stop,'pgmctrl --unload':unload,
                'read_motor --all':lambda:dict(ret_code=0,ret_context=json.dumps(data)),command:write})
            robot=c.operator(arms=DUAL_ARMS,cartesian_target_order=['left','right'])
            result=self.run_command(robot,300,motor_id=0)
            self.assertTrue(result['success']);self.assertEqual(c.commands.count(command),1)
            self.assertEqual(data[0]['max_acc'],300.)
            self.assertNotIn('manual_en',c.commands)

if __name__=='__main__':unittest.main()
