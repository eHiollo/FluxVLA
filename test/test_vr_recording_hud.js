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
assert.match(attrs('recordHudDetails').value, /EP 1.*TIME 2.0s/);
assert.equal(attrs('recordHudDirections').visible, true);
assert.equal(attrs('recordHudRight').value, 'RIGHT\nSAVE');
assert.notEqual(attrs('recordHudUp').color, attrs('recordHudRight').color);
context.applyHud({phase: 'recording', pending: 'discard', hint: 'Recenter then repeat LEFT'});
assert.equal(attrs('recordHud').value, 'CONFIRM DISCARD');
assert.equal(attrs('recordHud').color, '#FFD54F');
context.applyHud({phase: 'failed', error: 'Camera disconnected'});
assert.equal(attrs('recordHudNotice').value, 'Camera disconnected');
assert.equal(attrs('recordHudNotice').color, '#FF7070');
context.applyHud({phase: 'stop', hint: 'VR DISCONNECTED'});
assert.equal(attrs('recordHudDirections').visible, false);
assert.equal(attrs('recordHudPrompt').value, '');
assert.equal(attrs('recordHudDetails').value, '');
console.log('HUD rendering checks passed (check/record/confirm/fault/disconnect).');

context.applyHud({phase:'input_paused', hint:'LEFT HAND: spike_pending', controls:{prompt:'RELEASE BOTH GRIPS/TRIGGERS + CENTER STICKS'}});
assert.equal(attrs('recordHud').value,'INPUT PAUSED / MOTION STOPPED');
assert.equal(attrs('recordHud').color,'#EF5350');
assert.match(attrs('recordHudPrompt').value,/RELEASE BOTH/);
context.applyHud({phase:'input_ready', controls:{prompt:'PRESS A SIDE GRIP TO RESUME'}});
assert.equal(attrs('recordHud').color,'#FFD54F');
assert.match(attrs('recordHudPrompt').value,/PRESS A SIDE/);
context.applyHud({phase:'idle', controls:{}});
assert.equal(attrs('recordHudPrompt').value,'');
console.log('Input pause / ready / resumed HUD checks passed.');
