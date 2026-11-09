const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
let now = 0, timers = new Set(), attrs = {};
const sent = [];
const context = {performance:{now:()=>now}, document:{visibilityState:'visible',
  querySelector:()=>({setAttribute:(k,v)=>attrs[k]=v})},
  setInterval: f=>{timers.add(f);return f;}, clearInterval:f=>timers.delete(f)};
vm.createContext(context);
vm.runInContext(fs.readFileSync(__dirname+'/../fluxvla/engines/operators/vr/web-ui/latency.js','utf8')+'\nthis.Client=VRLatencyClient;', context);
const socket={readyState:1,bufferedAmount:42,send:text=>sent.push(JSON.parse(text))};
const client=new context.Client(socket,{});
assert.equal(sent[0].type,'latency_ping');
now=12; client.receive({type:'latency_pong',id:1,pose_age_ms:5});
assert.equal(sent[1].rtt_ms,12);
const packet={leftController:{verificationTracking:true}};
now=20;client.stamp(packet);
assert.equal(packet._latency.sent_ms,20);
assert.equal(packet._latency.buffered_bytes,42);
assert.equal(packet._latency.tracking.left,true);
now=1000;client.probe();assert.match(attrs.value,/RTT 12.0 ms/);
now=5000;client.probe();assert.match(attrs.value,/no recent reply/);
const before=sent.length;
client.close();client.probe();client.receive({type:'latency_pong',id:2});
assert.equal(sent.length,before);assert.equal(timers.size,0);
const next=new context.Client(socket,{});const nextPacket={};next.stamp(nextPacket);
assert.equal(nextPacket._latency.seq,1);next.close();
console.log('VR latency browser checks passed: RTT, packet stamps, stale HUD, reconnect cleanup.');
