'use strict';
(() => {
const $ = id => document.getElementById(id);
const MODE = {old:'anchor_relative', world:'world_translation_local_rotation'};
const colors = {left:'#38bdf8', right:'#fbbf24'};
const labels = {tracking:'跟随', holding:'保持', releasing:'减速', frozen:'保持', paused:'暂停', waiting:'等待', stale:'输入过期', release_required:'松开后重新接管'};
const state = {latest:null, frozen:null, lastSuccess:0, offline:true, error:'等待服务', generation:null,
  clearTime:0, rawOrigin:null, rawTrail:{left:[],right:[]}, lastRawSequence:{}, camera:{yaw:-.9,pitch:.55,scale:300}, busy:false};
const add = (a,b) => a.map((v,i)=>v+b[i]);
const mul = (a,s) => a.map(v=>v*s);
const dot = (a,b) => a.reduce((s,v,i)=>s+v*b[i],0);
const cross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
function rotate(q,v) { const t=mul(cross(q.slice(0,3),v),2); return add(v,add(mul(t,q[3]),cross(q.slice(0,3),t))); }
const vrToDisplay = p => [p[0],-p[2],p[1]];
// Both bases are right-handed; the right base is rotated 180 degrees about table Z.
function baseVectorToWorld(data,hand,p) {
  const rows=data?.base_frames?.[hand]?.rotation_table_from_base||
    (hand==='right'?[[-1,0,0],[0,-1,0],[0,0,1]]:[[1,0,0],[0,1,0],[0,0,1]]);
  return rows.map(row=>dot(row,p));
}
function basePointToWorld(data,hand,p) {
  const origin=data?.base_frames?.[hand]?.origin_table_m||[0,hand==='left'?.25:-.25,0];
  return add(origin,baseVectorToWorld(data,hand,p));
}
const number = (v,d=1) => Number.isFinite(v)?v.toFixed(d):'—';
function live(data) {return !!data?.running && !state.offline && performance.now()-state.lastSuccess<700 && data.snapshot_age_s<=.1;}
function fresh(hand,data) {return live(data) && !!hand?.fresh && hand.age_ms+(performance.now()-state.lastSuccess)<=hand.age_limit_s*1000;}
function clearTrails() {state.clearTime=state.latest?.tick_time_s||0; state.rawTrail={left:[],right:[]};}
function ingest(data) {
  if (data.generation!==state.generation) {state.generation=data.generation;clearTrails();state.rawOrigin=null;state.lastRawSequence={};}
  state.latest=data;
  const poses=Object.values(data.hands||{}).filter(h=>h.fresh&&h.pose).map(h=>vrToDisplay(h.pose.slice(0,3)));
  if(!state.rawOrigin&&poses.length) state.rawOrigin=mul(poses.reduce(add,[0,0,0]),1/poses.length);
  for(const hand of ['left','right']) {
    const h=data.hands?.[hand],trail=state.rawTrail[hand];
    if(h?.fresh&&h.pose&&h.sequence!==state.lastRawSequence[hand]) {trail.push([data.tick_time_s,...vrToDisplay(h.pose.slice(0,3))]);state.lastRawSequence[hand]=h.sequence;}
    while(trail.length && (data.tick_time_s-trail[0][0]>10 || trail.length>300)) trail.shift();
  }
}
function projection(v,w,h,center) {
  const c=state.camera,p=v.map((x,i)=>x-center[i]),horizontal=[-Math.sin(c.yaw),Math.cos(c.yaw),0];
  const vertical=[-Math.sin(c.pitch)*Math.cos(c.yaw),-Math.sin(c.pitch)*Math.sin(c.yaw),Math.cos(c.pitch)];
  return [w/2+dot(p,horizontal)*c.scale,h*.53-dot(p,vertical)*c.scale];
}
function draw(id,data) {
  const canvas=$(id),rect=canvas.getBoundingClientRect(),dpr=window.devicePixelRatio||1;
  const w=rect.width,h=rect.height;if(!w||!h)return;
  if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}
  const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
  const center=id==='raw'?((state.frozen?state.frozen.raw_origin:state.rawOrigin)||[0,0,1]):[0,0,.4],proj=v=>projection(v,w,h,center);
  function line(a,b,color,width=1,alpha=1) {ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=width;ctx.globalAlpha=alpha;ctx.moveTo(...proj(a));ctx.lineTo(...proj(b));ctx.stroke();ctx.globalAlpha=1;}
  const floor=center[2]-.2;
  for(let i=-5;i<=5;i++){const d=i*.1;line([center[0]+d,center[1]-.5,floor],[center[0]+d,center[1]+.5,floor],'#2b3a4e',1,.65);line([center[0]-.5,center[1]+d,floor],[center[0]+.5,center[1]+d,floor],'#2b3a4e',1,.65);}
  const axisBase=[center[0]-.35,center[1]-.35,floor];
  const basis=id==='raw'?[[1,0,0],[0,0,1],[0,-1,0]]:[[1,0,0],[0,1,0],[0,0,1]];
  basis.forEach((v,i)=>{const end=add(axisBase,mul(v,.12));line(axisBase,end,['#fb7185','#4ade80','#60a5fa'][i],2);ctx.fillStyle=['#fb7185','#4ade80','#60a5fa'][i];ctx.font='11px system-ui';ctx.fillText('XYZ'[i],...proj(add(end,mul(v,.02))));});
  if(id!=='raw')for(const hand of ['left','right']) {
    const origin=basePointToWorld(data,hand,[0,0,0]);
    [[1,0,0],[0,1,0],[0,0,1]].forEach((v,i)=>{
      const end=add(origin,baseVectorToWorld(data,hand,mul(v,.11)));
      line(origin,end,['#fb7185','#4ade80','#60a5fa'][i],2);
      ctx.fillStyle=['#fb7185','#4ade80','#60a5fa'][i];ctx.fillText('XYZ'[i],...proj(end));
    });
    ctx.fillStyle=colors[hand];ctx.font='11px system-ui';
    const xy=proj(origin);ctx.fillText(hand==='left'?'左臂基座':'右臂基座',xy[0]+6,xy[1]+14);
  }
  for(const hand of ['left','right']) {
    const raw=data?.hands?.[hand],mapped=id==='raw'?null:data?.modes?.[MODE[id]]?.[hand];
    const pose=id==='raw'?raw?.pose:mapped?.pose;if(!pose)continue;
    const color=fresh(raw,data)||state.frozen?colors[hand]:'#64748b';
    const trail=id==='raw'?(state.frozen?state.frozen.raw_trails[hand]:state.rawTrail[hand]):(mapped.trail||[]);
    const points=trail.filter(p=>p[0]>=state.clearTime).map(p=>id==='raw'?p.slice(1):basePointToWorld(data,hand,p.slice(1)));for(let i=1;i<points.length;i++)line(points[i-1],points[i],color,1.5,.4+.4*i/points.length);
    const position=id==='raw'?vrToDisplay(pose.slice(0,3)):basePointToWorld(data,hand,pose.slice(0,3)),q=pose.slice(3);
    const transform=local=>{const rotated=rotate(q,local);return add(position,id==='raw'?vrToDisplay(rotated):baseVectorToWorld(data,hand,rotated));};
    // A fixed stylized tool, not an IK model or physical gripper simulation.
    if(id==='raw')line(transform([0,0,-.055]),transform([0,0,.035]),color,7);
    else {
      line(transform([0,-.035,0]),transform([0,.035,0]),color,5);
      for(const y of [-.035,.035])line(transform([0,y,0]),transform([0,y,.065]),color,4);
      line(transform([0,0,-.04]),position,color,6);
    }
    [[1,0,0],[0,1,0],[0,0,1]].forEach((v,i)=>line(position,transform(mul(v,.09)),['#fb7185','#4ade80','#60a5fa'][i],2));
    const [x,y]=proj(position);ctx.fillStyle=color;ctx.font='12px system-ui';ctx.fillText(hand==='left'?'左 L':'右 R',x+10,y-12);
  }
  ctx.fillStyle='#8294ac';ctx.font='11px system-ui';ctx.fillText(id==='raw'?'VR 追踪空间 · Y 向上':'工作台世界 · X前 / Y左 / Z上',14,h-14);
}
function render() {
  const data=state.frozen||state.latest,ok=live(state.latest),hands=data?.hands||{};
  const status=$('status');status.className='status'+(state.frozen?' warn':ok?' live':' warn');
  if(state.frozen)status.textContent='画面已冻结 · 非实时快照；后台仍接收输入并计算目标';
  else if(!ok)status.textContent=state.offline?'服务连接不可用：'+state.error:'计算数据已过期，等待服务更新';
  else if(data.notice)status.textContent=data.notice;
  else if(Object.values(hands).some(h=>h.state==='release_required'))status.textContent='输入已暂停：松开侧键和前扳机，等待追踪有效后重新接管';
  else status.textContent=Object.values(hands).every(h=>fresh(h,data))?'VR 输入已就绪 · 侧键：仅平移 · 前扳机：平移＋旋转 · 仅虚拟目标':'等待有效 VR 追踪 · 请在头显进入 VR';
  $('controlHz').textContent=number(data?.control_hz)+' Hz';
  for(const hand of ['left','right'])$(hand+'Hz').textContent=number(state.frozen||fresh(hands[hand],data)?hands[hand]?.input_hz:null)+' Hz';
  $('age').textContent=['left','right'].map(h=>number(hands[h]?.age_ms,0)).join(' / ')+' ms';
  $('reset').disabled=!!state.frozen||!ok||Object.values(state.latest?.hands||{}).some(h=>h.active);
  $('hands').replaceChildren();
  for(const hand of ['left','right']) {
    const h=hands[hand]||{},tr=document.createElement('tr');
    const pos=h.pose?h.pose.slice(0,3).map(v=>number(v*1000,0)).join(', '):'—';
    const statusText=!state.frozen&&!fresh(h,data)?'等待 / 过期':
      h.state==='tracking'?(h.rotation_enabled?'平移＋旋转':'仅 XYZ 平移'):labels[h.state]||'等待';
    const cells=[(hand==='left'?'左':'右')+' / '+(h.logical_arm||'—'),statusText,pos,
      (h.grip?'按住':'松开')+' / '+number(h.trigger,2)+' / '+number(h.stick?.x,2)+', '+number(h.stick?.y,2)];
    cells.forEach((text,i)=>{const td=document.createElement('td');td.textContent=text;if(i===0)td.className=hand;tr.appendChild(td);});$('hands').appendChild(tr);
    for(const id of ['old','world'])$(id+(hand==='left'?'Left':'Right')).textContent=(hand==='left'?'左：':'右：')+(statusText==='等待 / 过期'?statusText:labels[data?.modes?.[MODE[id]]?.[hand]?.state]||statusText);
  }
  const matrix=data?.modes?.[MODE.world];
  if(matrix)$('matrix').textContent=['left','right'].map(h=>(h==='left'?'左':'右')+'手平移矩阵：'+JSON.stringify(matrix[h]?.position_axis_remap)).join('\n');
  for(const id of ['raw','old','world'])draw(id,data);
}
async function poll() {
  try {const response=await fetch('/api/vr-sandbox',{cache:'no-store',signal:AbortSignal.timeout(1500)});if(!response.ok)throw Error('HTTP '+response.status);const data=await response.json();state.lastSuccess=performance.now();state.offline=false;ingest(data);}
  catch(error){state.offline=true;state.error=String(error.message||error);}
  render();setTimeout(poll,33);
}
$('freeze').onclick=()=>{state.frozen=state.frozen?null:state.latest?structuredClone({...state.latest,raw_trails:state.rawTrail,raw_origin:state.rawOrigin}):null;$('freeze').textContent=state.frozen?'恢复实时':'冻结显示';render();};
$('clear').onclick=()=>{clearTrails();render();};
$('camera').onclick=()=>{state.camera={yaw:-.9,pitch:.55,scale:300};render();};
$('reset').onclick=async()=>{try{const response=await fetch('/api/vr-sandbox/reset',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({preset:$('preset').value}),signal:AbortSignal.timeout(1500)});const result=await response.json();if(!response.ok)throw Error(result.error);$('status').textContent='虚拟重置已排队';}catch(error){$('status').textContent=error.message;}};
for(const id of ['raw','old','world']) {
  const canvas=$(id);let pointer=null;
  canvas.onpointerdown=e=>{pointer=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);};
  canvas.onpointermove=e=>{if(!pointer)return;state.camera.yaw+=(e.clientX-pointer[0])*.008;state.camera.pitch=Math.max(-1.3,Math.min(1.3,state.camera.pitch+(e.clientY-pointer[1])*.008));pointer=[e.clientX,e.clientY];render();};
  canvas.onpointerup=canvas.onpointercancel=()=>{pointer=null;};
  canvas.addEventListener('wheel',e=>{e.preventDefault();state.camera.scale=Math.max(100,Math.min(1000,state.camera.scale*Math.exp(-e.deltaY*.001)));render();},{passive:false});
}
window.addEventListener('resize',render);
window.sandboxApp={state,ingest,render,baseVectorToWorld,basePointToWorld,vrToDisplay};
render();poll();
})();
