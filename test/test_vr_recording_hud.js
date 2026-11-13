// Node-only HUD regression: exercise the real renderer without WebXR or devices.
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const web = path.join(__dirname, '../fluxvla/engines/operators/vr/web-ui');
const html = fs.readFileSync(path.join(web, 'index.html'), 'utf8');
const nodes = new Map([...html.matchAll(/id="([^"]+)"/g)].map(m => [
  '#' + m[1], {attrs: {}, setAttribute(k, v) { this.attrs[k] = v; }}
]));
const source = fs.readFileSync(path.join(web, 'vr_app.js'), 'utf8');
const context = {document: {querySelector: id => nodes.get(id)}};
vm.createContext(context);
vm.runInContext(source.slice(source.indexOf('    this.recordHud ='), source.indexOf('    // --- WebSocket Setup ---')), context);
const attrs = id => nodes.get('#' + id).attrs;
context.applyHud({phase: 'button_check', hint: 'BUTTON EVENT: right'});
assert.equal(attrs('recordHud').color, '#FFFFFF');
assert.equal(attrs('recordHudDirections').visible, false);
context.applyHud({phase: 'recording', episode: 1, elapsed_s: 2, task: 'Pick up bottle', controls: {
  visible: true, ready: true, prompt: 'LEFT STICK MENU', directions: {
    right: {label: 'SAVE', enabled: true}, up: {label: 'SAVE / DISCARD FIRST', enabled: false}
  }
}});
assert.equal(attrs('recordHud').color, '#66BB6A');
assert.equal(attrs('recordHudDetails').color, '#FFFFFF');
assert.match(attrs('recordHudDetails').value, /ep 1.*time 2.0s/);
assert.equal(attrs('recordHudDirections').visible, true);
assert.equal(attrs('recordHudRight').value, 'right\nsave');
assert.notEqual(attrs('recordHudUp').color, attrs('recordHudRight').color);
context.applyHud({phase: 'recording', pending: 'discard', hint: 'Recenter then repeat LEFT'});
assert.equal(attrs('recordHud').value, 'confirm discard');
assert.equal(attrs('recordHud').color, '#FFD54F');
context.applyHud({phase: 'failed', error: 'Camera disconnected'});
assert.equal(attrs('recordHudNotice').value, 'camera disconnected');
assert.equal(attrs('recordHudNotice').color, '#FF7070');
context.applyHud({phase: 'stop', hint: 'VR DISCONNECTED'});
assert.equal(attrs('recordHudDirections').visible, false);
assert.equal(attrs('recordHudPrompt').value, '');
assert.equal(attrs('recordHudDetails').value, '');
console.log('HUD rendering checks passed (check/record/confirm/fault/disconnect).');

context.applyHud({phase:'input_paused', hint:'LEFT HAND: spike_pending', controls:{prompt:'RELEASE BOTH GRIPS/TRIGGERS + CENTER STICKS'}});
assert.equal(attrs('recordHud').value,'input paused');
assert.equal(attrs('recordHud').color,'#EF5350');
assert.equal(attrs('recordHudPrompt').value,'release grips; center sticks');
context.applyHud({phase:'input_ready', controls:{prompt:'PRESS A SIDE GRIP TO RESUME'}});
assert.equal(attrs('recordHud').color,'#FFD54F');
assert.equal(attrs('recordHudPrompt').value,'press side grip to resume');
context.applyHud({phase:'idle', controls:{}});
assert.equal(attrs('recordHudPrompt').value,'');
console.log('Input pause / ready / resumed HUD checks passed.');

context.applyHud({phase:'input_ready', episode:1, hint:'retry pending', controls:{
  prompt:'release grips; center sticks', visible:true, ready:true,
  directions:{left:{label:'RETRY QUEUED',enabled:true},right:{label:'QUEUE RETRY',enabled:true}}
}});
assert.equal(attrs('recordHudNotice').value, 'retry queued; center sticks');
assert.equal(attrs('recordHudLeft').value, 'left\nretry queued');
assert.equal(attrs('recordHudRight').value, 'right\nqueue retry');
context.applyHud({phase:'failed', error:'VR input stale; episode is incomplete', controls:{
  visible:true, ready:true, directions:{left:{label:'REDO THIS EP',enabled:true},right:{label:'DISCARD + START AGAIN',enabled:true}}
}});
assert.equal(attrs('recordHudNotice').value, 'vr signal delayed');
assert.equal(attrs('recordHudLeft').value, 'left\nredo episode');
assert.equal(attrs('recordHudRight').value, 'right\nrestart');
context.applyHud({phase:'failed', error:'VERY LONG ERROR '.repeat(30)});
for (const id of ['recordHud', 'recordHudNotice', 'recordHudPrompt', 'recordHudLeft', 'recordHudRight']) {
  const value = attrs(id).value;
  assert.equal(value, value.toLowerCase());
  assert.ok(value.length <= 48);
}
console.log('Short lowercase retry menu checks passed.');
context.applyHud({phase:'idle', controls:{prompt:'HOLD RIGHT B FOR LEFT STICK MENU'}});
assert.equal(attrs('recordHudPrompt').value, 'hold right b for menu');

context.applyHud({phase: 'recording', gripper_warnings: {right: 'drop detected'}});
assert.equal(attrs('recordHudNotice').value, 'right drop warning; recording');
assert.equal(attrs('recordHud').color, '#66BB6A');
context.applyHud({phase: 'failed', error: 'camera disconnected', gripper_warnings: {right: 'drop detected'}});
assert.equal(attrs('recordHudNotice').value, 'camera disconnected');
