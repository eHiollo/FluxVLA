"""Execute both experiments against a packet-driven simulated controller."""
import contextlib
import io
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from test_a10_operator import FakeController,status_reply
from fluxvla.engines.communication.a10_shared_server import SharedControl,SharedControlServer
try:
    from .run import main
except ImportError:
    from run import main


class LoopbackTest(unittest.TestCase):
    def test_complete_step_and_chunk_recording_and_plots(self):
        with FakeController() as controller,tempfile.TemporaryDirectory() as directory:
            q=[0.,90.,-90.,180.,0.,45.]
            lock,done=threading.Lock(),threading.Event()
            failures=[]
            def get():
                reply=status_reply(active=controller.active,mode=controller.mode)
                with lock:
                    reply['ret_context']['motion_msg']['actual_pos']=[q.copy()]
                    reply['ret_context']['motion_msg']['motor_pos']=[q.copy()]
                return reply
            controller.command_replies['get']=get
            def plant():
                try:
                    while not done.is_set():
                        try:packet=controller.joint_udp.recv(65535)
                        except socket.timeout:continue
                        target=json.loads(packet[40:].decode().split('=',1)[1])
                        with lock:
                            for i,x in enumerate(target):q[i]+=max(-.5,min(.5,x-q[i]))
                except BaseException as exc:failures.append(exc)
            worker=threading.Thread(target=plant);worker.start()
            port=controller.server.socket.getsockname()[1]
            robot=SharedControl('127.0.0.1',port,port,controller.udp.getsockname()[1],controller.joint_udp.getsockname()[1])
            robot.connect()
            server=SharedControlServer(('127.0.0.1',0),robot)
            thread=threading.Thread(target=server.serve_forever,kwargs=dict(poll_interval=.01))
            thread.start()
            try:
                output=io.StringIO()
                with contextlib.redirect_stdout(output):
                    code=main(['--host','127.0.0.1','--ws-port',str(port),'--status-ws-port',str(port),
                        '--udp-port',str(robot.udp_port),'--joint-udp-port',str(robot.joint_udp_port),
                        '--shared-port',str(server.server_address[1]),'--log-dir',directory,
                        '--delta-deg','1','--repeat','2','--chunk-steps','6','--chunks','2','--settle-s','.08','--tail-s','.1'])
                self.assertEqual(code,0,output.getvalue())
                self.assertFalse(failures)
                self.assertFalse(controller.active)
                path=next(Path(directory).glob('*_follower_tracking'))
                result=json.loads((path/'run.json').read_text())
                self.assertTrue(result['completed']);self.assertTrue(result['stop_confirmed'])
                report=json.loads((path/'follower_report/metrics.json').read_text())
                self.assertTrue(report['measurement_complete'])
                self.assertTrue(all(p['completed'] for p in report['phases']))
                self.assertEqual({p['name'] for p in report['phases']},{'baseline','step','step_return','chunk','chunk_return'})
                with (path/'events.jsonl').open() as stream:rows=[json.loads(line) for line in stream]
                sends=[r for r in rows if r['event']=='send' and r['phase']=='chunk']
                self.assertEqual(len(sends),24)
                for a,b in zip(sends[::2],sends[1::2]):
                    self.assertEqual(a['target_deg'],b['target_deg'])
                    self.assertGreater(b['send_started_s']-a['send_started_s'],.005)
                for name in ('step.png','chunk.png','overview.png','aligned.csv'):
                    self.assertTrue((path/'follower_report'/name).is_file())
                self.assertTrue(any(r['event']=='state' and 'raw' in r for r in rows))
            finally:
                server.shutdown();thread.join(2);server.server_close();robot.close()
                done.set();worker.join(2)


if __name__=='__main__':unittest.main()
