#!/usr/bin/env python3
"""A10 双夹爪浏览器调试界面（唯一 UI 入口，无需 DISPLAY）。

启动：python3 scripts/a10_gripper_ui.py
默认监听 192.168.110.119:8765，在电脑浏览器打开终端输出的完整链接。
网页、HTTP 服务和调试后端均在此文件，设备校验复用 a10_devices.py。
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import socket
import threading
from urllib.parse import parse_qs, urlsplit

import a10_devices as devices

DEFAULT_HOST = '192.168.110.119'
DEFAULT_PORT = 8765

HTML_PAGE = r"""<!doctype html>
<html lang="zh-CN">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>A10 夹爪调试</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#f3f5f8;color:#1d2939;font:15px system-ui,sans-serif}main{max-width:960px;margin:48px auto;padding:0 22px}h1{font-size:30px;margin-bottom:8px}.muted{color:#667085;line-height:1.7}.cards{display:grid;grid-template-columns:1fr 1fr;gap:20px;margin:26px 0}.card{background:white;border:1px solid #dce1e9;border-radius:16px;padding:24px}h2{margin:0 0 10px;font-size:21px}.row{display:flex;gap:10px;margin:18px 0}button,input{font:inherit;border:1px solid #cbd3de;border-radius:8px;padding:10px;min-width:0}button{cursor:pointer;background:white;flex:1}button.primary{background:#245edd;color:white;border-color:#245edd}button:disabled{opacity:.5;cursor:wait}input{width:52%;background:#f9fafb}.feedback{min-height:48px;line-height:1.7;font-size:14px;white-space:pre-wrap}#status{font-weight:600;margin:20px 0 12px}#log{background:#172231;color:#dbe7f5;border-radius:12px;padding:18px;min-height:150px;max-height:300px;overflow:auto;font:13px/1.8 ui-monospace,monospace;white-space:pre-wrap;overflow-wrap:anywhere}.warn{font-size:13px;margin-top:18px}.error{color:#c43229}.toolbar{background:white;border:1px solid #dce1e9;border-radius:16px;padding:20px;margin-top:24px}.speed-row{display:flex;align-items:center;gap:14px;flex-wrap:wrap}.speed-row input[type=range]{width:220px;padding:0}.speed-row input[type=number]{width:78px}.presets{display:flex;gap:8px}.presets button{padding:6px 12px}.toolbar .row{margin-bottom:0}@media(max-width:640px){main{margin:26px auto}.cards{grid-template-columns:1fr}}
</style>
<main>
<h1>夹爪调试</h1>
<div class="muted">左右独立操作 · 启动不发送动作 · raw 为原始位置单位</div>
<section class="toolbar">
<div class="speed-row"><label for="speed">运动速度（%）</label><input id="speed-range" aria-label="运动速度滑块" type="range" min="1" max="50" value="5" disabled><input id="speed" type="number" min="1" max="50" step="1" value="5" disabled><div class="presets" id="presets"></div></div>
<div class="muted">范围 1–50%，调节后在下次动作生效；左右单独操作和双爪操作共用此速度。</div>
<label class="muted" style="display:block;margin-top:12px"><input id="drop-warning" type="checkbox" style="width:auto" disabled> 掉落报警 B=0x1000 仅提示，允许继续动作（其他报警仍阻止运动）</label>
<div class="row"><button class="primary" id="both-open" disabled>双爪同时张开</button><button id="both-close" disabled>双爪同时关闭</button></div>
</section>
<div class="cards" id="cards"></div>
<div id="status" role="status">正在连接本地服务…</div>
<div id="log" role="log" aria-live="polite">等待操作。</div>
<div class="muted warn">关闭页面或服务不会停止夹爪或释放力矩。等待超时后，最后目标仍可能生效。</div>
</main>
<script>
const token='__DEBUG_TOKEN__';
let seen=0, pending=false, built=false;
const names={left:'左夹爪',right:'右夹爪',both:'双爪'};
const labels={open:'张开',close:'关闭',status:'查询状态',position:'移动到 raw'};
const outcomes={position_reached:'确认到位',torque_reached:'遇阻，未确认到位',timeout:'等待超时，未确认到位'};
const status=document.getElementById('status');
const speedInput=document.getElementById('speed');
const speedRange=document.getElementById('speed-range');
speedRange.oninput=()=>speedInput.value=speedRange.value;
speedInput.oninput=()=>{if(speedInput.checkValidity())speedRange.value=speedInput.value;};
for(const value of [5,10,20,35,50]){
  const button=document.createElement('button');
  button.textContent=`${value}%`;
  button.disabled=true;
  button.onclick=()=>{speedInput.value=value;speedRange.value=value;};
  document.getElementById('presets').append(button);
}
document.getElementById('both-open').onclick=()=>send('both','open');
document.getElementById('both-close').onclick=()=>send('both','close');
function log(text){
  const el=document.getElementById('log');
  if(el.textContent==='等待操作。')el.textContent='';
  el.textContent+=`[${new Date().toLocaleTimeString()}] ${text}\n`;
  el.scrollTop=el.scrollHeight;
}
function disable(value){document.querySelectorAll('button,input').forEach(el=>el.disabled=value);}
async function api(path,body){
  const response=await fetch(path,{method:body?'POST':'GET',headers:{'X-Debug-Token':token,'Content-Type':'application/json'},body:body?JSON.stringify(body):undefined});
  const data=await response.json();
  if(!response.ok)throw Error(data.error||`HTTP ${response.status}`);
  return data;
}
function build(grippers){
  speedInput.value=speedRange.value=grippers.left.speed;
  for(const side of ['left','right']){
    const cfg=grippers[side],c=cfg.check;
    const card=document.createElement('section');card.className='card';
    card.innerHTML=`<h2>${names[side]}</h2><div class="muted" id="meta-${side}"></div><div class="row"><button class="primary" data-action="open">张开</button><button data-action="close">关闭</button><button data-action="status">查询状态</button></div><div class="row"><input type="number" step="1" aria-label="${names[side]}目标 raw" id="target-${side}"><button data-action="position">移动到 raw</button></div><div class="feedback" id="feedback-${side}">尚未查询</div>`;
    card.querySelector(`#meta-${side}`).textContent=`范围 ${c.min_raw}–${c.max_raw} raw · 力矩 ${cfg.torque}%`;
    const input=card.querySelector('input');
    input.min=c.min_raw;input.max=c.max_raw;input.value=Math.floor((c.min_raw+c.max_raw)/2);
    card.querySelectorAll('button').forEach(button=>button.onclick=()=>send(side,button.dataset.action));
    document.getElementById('cards').append(card);
  }
  built=true;
}
async function send(side,action){
  if(pending)return;
  let target, speed;
  if(action!=='status'){
    speed=Number(speedInput.value);
    if(speedInput.value.trim()===''||!Number.isInteger(speed)||speed<1||speed>50){
      log('未发送：速度须为 1–50 的整数百分比。');return;
    }
  }
  if(action==='position'){
    const el=document.getElementById(`target-${side}`);target=Number(el.value);
    if(el.value.trim()===''||!Number.isInteger(target)||!el.checkValidity()){
      log('未发送：请输入配置范围内的整数 raw 位置。');return;
    }
  }
  pending=true;disable(true);
  status.textContent=`${names[side]}：${labels[action]}${speed===undefined?'':`，速度 ${speed}%`}，执行中…`;
  status.className='';log(status.textContent);
  try{await api('/api/action',{side,action,target,speed,drop_alarm_policy:document.getElementById('drop-warning').checked?'warn':'stop'});}
  catch(error){log(`请求失败：${error.message}；请查询状态确认设备情况。`);}
  finally{pending=false;await poll();}
}
function renderResult(side,action,result){
  const feedback=document.getElementById(`feedback-${side}`);
  if(result.error){
    feedback.textContent='本次操作失败；当前状态未知，请重新查询。';
    return {message:`${names[side]}：失败：${result.error}`,failed:true};
  }
  const data=result.data, state=action==='status'?data:data.final;
  feedback.textContent=`实际位置 raw=${state.position_raw}；使能=${state.enabled?'是':'否'}\n报警 A=0x${state.alarm_a.toString(16).padStart(4,'0')} / B=0x${state.alarm_b.toString(16).padStart(4,'0')}`;
  const dropWarning=data.drop_alarm_policy==='warn'&&state.alarm_b===0x1000&&state.alarm_a===0;
  if(dropWarning)feedback.textContent+='\n掉落报警仅提示，原始报警保留。';
  const message=action==='status'?'状态已读取':`${outcomes[data.outcome]}：目标 raw=${data.target_raw}，实际 raw=${state.position_raw}，速度 ${data.speed}% ，耗时 ${data.elapsed_s.toFixed(2)}s`;
  return {message:`${names[side]}：${message}`,failed:action==='status'?!!(state.alarm_a||state.alarm_b)&&!dropWarning:data.outcome!=='position_reached'};
}
async function poll(){
  try{
    const state=await api('/api/state');
    if(!built)build(state.grippers);
    disable(state.busy||pending);
    const r=state.result;
    if(r&&r.sequence>seen){
      seen=r.sequence;
      const results=r.side==='both'?['left','right'].map(side=>renderResult(side,r.action,r.error?{error:r.error}:r.data.results[side])):[renderResult(r.side,r.action,r)];
      status.textContent=results.map(item=>item.message).join('；');
      status.className=results.some(item=>item.failed)?'error':'';
      results.forEach(item=>log(item.message));
    }
    if(state.busy)status.textContent='正在执行，请等待反馈…';
    else if(!r&&!pending)status.textContent='就绪';
  }catch(error){
    disable(true);status.className='error';
    status.textContent=`连接失败：${error.message}。请检查服务或网络；连接中断不会停止硬件动作。`;
  }
}
(async function refresh(){await poll();setTimeout(refresh,600);})();
</script>
</html>
"""


class GripperBackend:
    """Use the CLI's identity checks, position limits and feedback handling."""

    def __init__(self, config_path):
        self.config_path = Path(config_path).resolve()
        self.config, _ = devices.load_config(
            self.config_path.read_text(), self.config_path.parent)
        for side in ('left', 'right'):
            devices.gripper_limits(self.config['grippers'][side])

    def execute(self, side, action, target=None, speed=None, drop_alarm_policy='stop'):
        if side not in ('left', 'right', 'both') or action not in ('status', 'open', 'close', 'position'):
            raise ValueError('无效的夹爪或操作')
        if side == 'both' and action not in ('open', 'close'):
            raise ValueError('双爪仅支持同时张开或关闭')
        if speed is not None and (type(speed) is not int or not 1 <= speed <= 50):
            raise ValueError('速度须为1..50之间的整数百分比')
        if drop_alarm_policy not in ('stop', 'warn'):
            raise ValueError('仅支持 stop 或 warn 掉落报警策略')
        # Re-read settings and identify the device before every operation.
        config, _ = devices.load_config(self.config_path.read_text(), self.config_path.parent)
        sides = ('left', 'right') if side == 'both' else (side,)
        if speed is not None:
            for name in sides:
                config['grippers'][name]['speed'] = speed
        grippers_only = dict(cameras={}, grippers=config['grippers'])
        rows = devices.match_devices(grippers_only, devices.scan_devices())
        if side == 'both':
            # Check both devices before starting the two independent serial moves.
            plans = [devices.prepare_gripper_move(config, rows, name, action, max_speed=50)
                     for name in sides]
            module = devices.load_operator('gripper')
            with ExitStack() as stack:
                clients = [stack.enter_context(module.ZXGripper(
                    row['configured_path'], baudrate=settings['baudrate'],
                    max_position_raw=settings['check']['max_raw'],
                    drop_alarm_policy=drop_alarm_policy)) for row, settings, _ in plans]
                for client, (_, settings, _) in zip(clients, plans):
                    devices.read_check_state(client, settings, max_speed=50)
                with ThreadPoolExecutor(max_workers=2) as pool:
                    tasks = {name: pool.submit(devices.execute_check_move, client, settings,
                                               row, position, action, max_speed=50)
                             for name, client, (row, settings, position) in zip(sides, clients, plans)}
                    results = {}
                    for name, task in tasks.items():
                        try:
                            results[name] = dict(data=task.result())
                        except Exception as exc:
                            results[name] = dict(error=str(exc))
                return dict(results=results)
        if action != 'status':
            return devices.move_gripper(config, rows, side, action, target, max_speed=50,
                                        drop_alarm_policy=drop_alarm_policy)
        row, = devices.require_matched(rows, 'grippers', side)
        settings = config['grippers'][side]
        module = devices.load_operator('gripper')
        with module.ZXGripper(row['configured_path'], baudrate=settings['baudrate'],
                              max_position_raw=settings['check']['max_raw'],
                              drop_alarm_policy=drop_alarm_policy) as client:
            return dict(asdict(client.read_state(settings['device_id'])), drop_alarm_policy=drop_alarm_policy)


class DebugServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, backend):
        super().__init__(address, Handler)
        self.public_mode = address[0] not in ('127.0.0.1', 'localhost', '::1')
        self.backend = backend
        self.token = secrets.token_urlsafe(32)
        self.lock = threading.Lock()
        self.busy = False
        self.sequence = 0
        self.result = None

    def run_action(self, side, action, target, speed=None, drop_alarm_policy='stop'):
        try:
            options = {} if speed is None else dict(speed=speed)
            if drop_alarm_policy != 'stop':
                options['drop_alarm_policy'] = drop_alarm_policy
            result = dict(side=side, action=action, data=self.backend.execute(side, action, target, **options))
        except Exception as exc:
            result = dict(side=side, action=action, error=str(exc))
        with self.lock:
            self.sequence += 1
            self.result = dict(sequence=self.sequence, **result)
            self.busy = False


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply(self, status, data, content_type='application/json; charset=utf-8'):
        payload = json.dumps(data, ensure_ascii=False).encode() if not isinstance(data, bytes) else data
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header('Referrer-Policy', 'no-referrer')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(payload)

    def allowed_host(self):
        return self.server.public_mode or urlsplit('http://' + self.headers.get('Host', '')).hostname in ('localhost', '127.0.0.1', '::1')

    def authenticated(self):
        return secrets.compare_digest(self.headers.get('X-Debug-Token', ''), self.server.token)

    def do_GET(self):
        if not self.allowed_host():
            return self.reply(403, {'error': '请使用服务启动时显示的访问链接'})
        url = urlsplit(self.path)
        if url.path == '/':
            if self.server.public_mode:
                token = parse_qs(url.query).get('token', [''])[0]
                if not secrets.compare_digest(token, self.server.token):
                    return self.reply(403, {'error': '请使用服务启动时显示的完整访问链接（包含 token）'})
            page = HTML_PAGE
            return self.reply(200, page.replace('__DEBUG_TOKEN__', self.server.token).encode(),
                              'text/html; charset=utf-8')
        if self.path != '/api/state':
            return self.reply(404, {'error': '不存在'})
        if not self.authenticated():
            return self.reply(403, {'error': '请刷新页面'})
        with self.server.lock:
            state = dict(busy=self.server.busy, result=self.server.result,
                         grippers=self.server.backend.config['grippers'])
        return self.reply(200, state)

    def do_POST(self):
        if not self.allowed_host() or not self.authenticated():
            return self.reply(403, {'error': '请使用启动时显示的完整链接打开或刷新页面'})
        if self.path != '/api/action':
            return self.reply(404, {'error': '不存在'})
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 2048:
                raise ValueError('请求长度无效')
            request = json.loads(self.rfile.read(length))
            if not isinstance(request, dict):
                raise ValueError('请求格式无效')
            side, action, target = request.get('side'), request.get('action'), request.get('target')
            if side not in ('left', 'right', 'both') or action not in ('open', 'close', 'status', 'position'):
                raise ValueError('操作无效')
            if side == 'both' and action not in ('open', 'close'):
                raise ValueError('双爪仅支持同时张开或关闭')
            drop_alarm_policy = request.get('drop_alarm_policy', 'stop')
            if drop_alarm_policy not in ('stop', 'warn'):
                raise ValueError('仅允许将掉落报警 B=0x1000 降级为提示')
            speed = request.get('speed')
            if speed is not None and (type(speed) is not int or not 1 <= speed <= 50):
                raise ValueError('速度须为1..50之间的整数百分比')
            if action == 'position':
                limits = self.server.backend.config['grippers'][side]['check']
                if type(target) is not int or not limits['min_raw'] <= target <= limits['max_raw']:
                    raise ValueError('raw 位置必须是配置范围内的整数')
            elif target is not None:
                raise ValueError('此操作不接受目标位置')
        except (ValueError, UnicodeError) as exc:
            return self.reply(400, {'error': str(exc)})
        with self.server.lock:
            if self.server.busy:
                return self.reply(409, {'error': '正在执行，请等待本次操作完成'})
            self.server.busy = True
        threading.Thread(target=self.server.run_action, args=(side, action, target, speed, drop_alarm_policy), daemon=True).start()
        self.reply(202, {'accepted': True})


def serve(backend, port, host='127.0.0.1'):
    with DebugServer((host, port), backend) as server:
        if server.public_mode:
            addresses = [host] if host != '0.0.0.0' else []
            if not addresses:
                try:
                    addresses = sorted({ip for ip in socket.gethostbyname_ex(socket.gethostname())[2]
                                        if not ip.startswith('127.')})
                except OSError:
                    pass
            for address in addresses or ['<机器人IP>']:
                print(f'夹爪调试页：http://{address}:{server.server_port}/?token={server.token}', flush=True)
            print('在你的电脑浏览器打开上面的完整链接，无需 SSH 转发。', flush=True)
        else:
            print(f'夹爪调试页：http://127.0.0.1:{server.server_port}', flush=True)
            print(f'在你的电脑建立 SSH 转发：ssh -N -L {server.server_port}:127.0.0.1:{server.server_port} robot@<机器人IP>', flush=True)
        print('启动不发送动作。Ctrl+C 退出服务；退出不会停止夹爪或释放力矩。', flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=devices.ROOT / 'configs/a10/a10_dual_devices.py')
    parser.add_argument('--host', default=DEFAULT_HOST,
                        help='监听地址，默认192.168.110.119；127.0.0.1仅本机，0.0.0.0监听全部网卡')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT, help='浏览器服务端口，默认8765')
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error('--port 须在1..65535内')
    try:
        serve(GripperBackend(args.config), args.port, args.host)
    except (OSError, ValueError) as exc:
        parser.exit(1, f'无法启动夹爪界面：{exc}\n')


if __name__ == '__main__':
    main()
