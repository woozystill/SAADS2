#!/usr/bin/env python3

"""
SAADS motor dashboard.

Hosts a browser GUI on the Jetson and communicates only through ROS 2.
It does NOT open the I2C bus directly.

Default URL over the SAADS PtP network:
    http://192.168.50.20:8080
"""

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Empty, Float32MultiArray, String


DASHBOARD_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SAADS Control</title>
<style>
:root {
  color-scheme: dark;
  --bg:#0b1220;
  --panel:#151f2f;
  --panel2:#1c293b;
  --border:#334155;
  --text:#e5edf7;
  --muted:#93a4b8;
  --good:#4ade80;
  --warn:#facc15;
  --bad:#f87171;
  --accent:#60a5fa;
  --accent2:#2563eb;
}
* { box-sizing:border-box; }
body { margin:0; font-family:Inter,Arial,sans-serif; background:var(--bg); color:var(--text); }
main { max-width:1200px; margin:auto; padding:20px; }
header { display:flex; align-items:flex-end; justify-content:space-between; gap:16px; flex-wrap:wrap; margin-bottom:16px; }
h1 { margin:0; font-size:clamp(28px,4vw,44px); letter-spacing:.3px; }
.sub { color:var(--muted); margin-top:5px; }
.status-pill { border:1px solid var(--border); background:var(--panel); padding:8px 12px; border-radius:999px; font-weight:700; }
.ok { color:var(--good); }
.bad { color:var(--bad); }
.warn { color:var(--warn); }
.notice { border-left:4px solid #f59e0b; padding:10px 12px; background:#27231d; margin:14px 0 18px; border-radius:0 8px 8px 0; }
.grid { display:grid; grid-template-columns:repeat(3,minmax(0,1fr)); gap:14px; }
.card { background:var(--panel); border:1px solid var(--border); border-radius:14px; padding:16px; min-width:0; }
.card h2 { margin:0 0 12px; font-size:20px; }
.row { display:flex; justify-content:space-between; align-items:center; gap:14px; margin:8px 0; }
.row span:last-child,.row b { text-align:right; overflow-wrap:anywhere; }
.metric { font-variant-numeric:tabular-nums; }
.tabs { display:flex; gap:8px; margin:18px 0 12px; border-bottom:1px solid var(--border); padding-bottom:8px; }
.tab { border:1px solid var(--border); background:transparent; color:var(--muted); padding:10px 16px; border-radius:9px; font-weight:800; cursor:pointer; }
.tab.active { background:var(--accent2); color:white; border-color:var(--accent2); }
.tab-panel { display:none; }
.tab-panel.active { display:block; }
.control-layout { display:grid; grid-template-columns:minmax(300px,1.2fr) minmax(260px,.8fr); gap:14px; }
.control-top { display:flex; align-items:center; justify-content:space-between; gap:14px; flex-wrap:wrap; margin-bottom:14px; }
.power-controls { display:grid; gap:10px; min-width:min(100%,420px); }
.power-box { display:grid; grid-template-columns:92px 42px minmax(130px,220px) 42px 64px; align-items:center; gap:8px; }
.power-box input[type=range] { width:100%; }
.power-number { min-width:58px; font-size:18px; font-weight:800; text-align:right; }
.nudge { border:1px solid #526176; background:#26364b; color:white; border-radius:8px; min-width:42px; height:36px; font-size:20px; font-weight:800; cursor:pointer; }
.shortcut-grid { display:grid; grid-template-columns:1fr 1fr; gap:8px; margin-top:12px; }
.shortcut-box { background:var(--panel2); border:1px solid var(--border); border-radius:9px; padding:9px 10px; color:var(--muted); }
.shortcut-box b { color:var(--text); }
.dpad-wrap { display:flex; justify-content:center; align-items:center; padding:10px 0 16px; }
.dpad { display:grid; grid-template-columns:92px 92px 92px; grid-template-rows:76px 76px 76px; gap:10px; }
.dpad button { margin:0; font-size:34px; border:1px solid #526176; background:#26364b; color:white; border-radius:14px; cursor:pointer; touch-action:none; user-select:none; }
.dpad button.active { background:var(--accent2); border-color:#93c5fd; transform:translateY(1px); }
.dpad .up { grid-column:2; grid-row:1; }
.dpad .left { grid-column:1; grid-row:2; }
.dpad .center { grid-column:2; grid-row:2; font-size:12px; color:var(--muted); display:flex; align-items:center; justify-content:center; text-align:center; border:1px dashed var(--border); border-radius:14px; }
.dpad .right { grid-column:3; grid-row:2; }
.dpad .down { grid-column:2; grid-row:3; }
.command-grid { display:grid; grid-template-columns:1fr 1fr; gap:10px; margin-top:8px; }
.command-box { background:var(--panel2); border:1px solid var(--border); border-radius:10px; padding:12px; text-align:center; }
.command-box .name { color:var(--muted); font-size:13px; }
.command-box .value { font-size:24px; font-weight:800; margin-top:3px; font-variant-numeric:tabular-nums; }
.drive-state { text-align:center; font-size:20px; font-weight:800; margin:4px 0 10px; }
.help { color:var(--muted); line-height:1.45; margin:8px 0 0; }
kbd { border:1px solid #64748b; background:#0f172a; border-bottom-width:3px; border-radius:6px; padding:2px 7px; font-family:inherit; color:#e2e8f0; }
.actionbar { display:flex; flex-wrap:wrap; gap:8px; margin-top:14px; }
button.action { border:0; border-radius:9px; padding:12px 17px; font-weight:800; cursor:pointer; margin:0; color:white; }
.arm { background:#16a34a; }
.stop { background:#dc2626; font-size:17px; }
.secondary { background:#475569; }
.test-controls { display:grid; grid-template-columns:120px 1fr 76px; align-items:center; gap:12px; margin:13px 0; }
input[type=range] { width:100%; accent-color:#60a5fa; }
input[type=number] { width:72px; padding:8px; background:#0f172a; color:var(--text); border:1px solid #475569; border-radius:7px; font-size:16px; }
.mode-badge { display:inline-flex; gap:8px; align-items:center; border:1px solid var(--border); border-radius:999px; padding:7px 11px; color:var(--muted); }
.dot { width:9px; height:9px; border-radius:50%; background:#64748b; }
.dot.live { background:var(--good); }
@media (max-width:900px) {
  .grid { grid-template-columns:1fr; }
  .control-layout { grid-template-columns:1fr; }
}
@media (max-width:560px) {
  main { padding:12px; }
  .test-controls { grid-template-columns:92px 1fr 68px; gap:8px; }
  .dpad { grid-template-columns:78px 78px 78px; grid-template-rows:68px 68px 68px; }
  .power-box { grid-template-columns:78px 38px minmax(100px,1fr) 38px 56px; gap:6px; }
  .shortcut-grid { grid-template-columns:1fr; }
}
</style>
</head>
<body>
<main>
<header>
  <div>
    <h1>SAADS Control</h1>
    <div class="sub">Jetson-hosted control over MikroTik PtP → ROS 2 → I²C → ESP32</div>
  </div>
  <div class="status-pill"><span id="conn" class="warn">Waiting for ROS / I²C…</span></div>
</header>

<div class="notice"><b>Powered testing:</b> keep the drivetrain lifted/off the ground until direction, STOP, watchdogs, and keyboard release behavior are verified.</div>

<div class="grid">
  <section class="card">
    <h2>Controller</h2>
    <div class="row"><span>I²C</span><span id="i2c">--</span></div>
    <div class="row"><span>Remote armed</span><b id="armed">--</b></div>
    <div class="row"><span>Remote stopped</span><b id="stopped">--</b></div>
    <div class="row"><span>Setpoint fresh</span><b id="fresh">--</b></div>
    <div class="row"><span>Good packets</span><span id="good" class="metric">--</span></div>
    <div class="row"><span>Bad packets</span><span id="bad" class="metric">--</span></div>
  </section>

  <section class="card">
    <h2>Wheel speed</h2>
    <div class="row"><span>Front Left</span><b id="flspeed" class="metric">0.000 m/s</b></div>
    <div class="row"><span>Front Right</span><b id="frspeed" class="metric">0.000 m/s</b></div>
    <div class="row"><span>Rear Left</span><b id="rlspeed" class="metric">0.000 m/s</b></div>
    <div class="row"><span>Rear Right</span><b id="rrspeed" class="metric">0.000 m/s</b></div>
  </section>

  <section class="card">
    <h2>Wheel odometry</h2>
    <div class="row"><span>X</span><b id="x" class="metric">0.000 m</b></div>
    <div class="row"><span>Y</span><b id="y" class="metric">0.000 m</b></div>
    <div class="row"><span>Yaw</span><b id="yaw" class="metric">0.0°</b></div>
    <div class="row"><span>Linear X</span><b id="vx" class="metric">0.000 m/s</b></div>
    <div class="row"><span>Angular Z</span><b id="wz" class="metric">0.000 rad/s</b></div>
  </section>
</div>

<div class="tabs" role="tablist" aria-label="Control mode">
  <button id="driveTab" class="tab active" type="button" role="tab" aria-selected="true">Drive</button>
  <button id="testTab" class="tab" type="button" role="tab" aria-selected="false">Motor Testing</button>
</div>

<section id="drivePanel" class="tab-panel active">
  <div class="control-layout">
    <section class="card">
      <div class="control-top">
        <div>
          <h2>Keyboard Drive</h2>
          <div class="mode-badge"><span id="driveDot" class="dot"></span><span id="driveModeText">Not controlling</span></div>
        </div>
        <div class="power-controls">
          <div class="power-box">
            <label for="drivePower"><b>Drive %</b></label>
            <button id="driveDown" class="nudge" type="button" aria-label="Decrease drive power">−</button>
            <input id="drivePower" type="range" min="0" max="100" step="1" value="10">
            <button id="driveUp" class="nudge" type="button" aria-label="Increase drive power">+</button>
            <span id="drivePowerValue" class="power-number">10%</span>
          </div>
          <div class="power-box">
            <label for="turnPower"><b>Turn %</b></label>
            <button id="turnDown" class="nudge" type="button" aria-label="Decrease turn power">−</button>
            <input id="turnPower" type="range" min="0" max="100" step="1" value="10">
            <button id="turnUp" class="nudge" type="button" aria-label="Increase turn power">+</button>
            <span id="turnPowerValue" class="power-number">10%</span>
          </div>
        </div>
      </div>

      <div id="driveState" class="drive-state">IDLE</div>
      <div class="dpad-wrap">
        <div class="dpad" aria-label="Drive controls">
          <button id="upButton" class="up" type="button" aria-label="Forward">↑</button>
          <button id="leftButton" class="left" type="button" aria-label="Turn left">←</button>
          <div class="center">ARROW<br>KEYS</div>
          <button id="rightButton" class="right" type="button" aria-label="Turn right">→</button>
          <button id="downButton" class="down" type="button" aria-label="Reverse">↓</button>
        </div>
      </div>

      <p class="help">
        Hold <kbd>↑</kbd> forward, <kbd>↓</kbd> reverse, <kbd>←</kbd> turn left, and <kbd>→</kbd> turn right.
        <b>Drive %</b> sets forward/reverse power. <b>Turn %</b> sets pivot strength and how strongly a held left/right key bends an arc.
        Releasing the direction keys commands 0% immediately.
      </p>
      <div class="shortcut-grid">
        <div class="shortcut-box"><b>Drive power:</b> <kbd>+</kbd> / <kbd>=</kbd> adds 5%, <kbd>−</kbd> subtracts 5%</div>
        <div class="shortcut-box"><b>Turn power:</b> <kbd>]</kbd> adds 5%, <kbd>[</kbd> subtracts 5%</div>
      </div>
    </section>

    <section class="card">
      <h2>Current wheel command</h2>
      <div class="command-grid">
        <div class="command-box"><div class="name">Front Left</div><div id="cmdFL" class="value">0%</div></div>
        <div class="command-box"><div class="name">Front Right</div><div id="cmdFR" class="value">0%</div></div>
        <div class="command-box"><div class="name">Rear Left</div><div id="cmdRL" class="value">0%</div></div>
        <div class="command-box"><div class="name">Rear Right</div><div id="cmdRR" class="value">0%</div></div>
      </div>
      <div class="actionbar">
        <button id="armDrive" class="action arm" type="button">ARM REMOTE CONTROL</button>
        <button id="zeroDrive" class="action secondary" type="button">ZERO COMMAND</button>
        <button id="stopDrive" class="action stop" type="button">STOP ALL</button>
      </div>
      <p class="help">The keyboard only commands motion after you arm from this browser session. If the browser loses focus, all arrow keys are released and a 0% command is sent.</p>
    </section>
  </div>
</section>

<section id="testPanel" class="tab-panel">
  <section class="card">
    <div class="control-top">
      <div>
        <h2>Individual Motor Testing</h2>
        <div class="sub">Use this tab for wiring, direction, Hall, and drivetrain troubleshooting.</div>
      </div>
      <div class="mode-badge"><span class="dot"></span><span>Independent FL / FR / RL / RR</span></div>
    </div>

    <div class="test-controls"><label for="fl">Front Left</label><input id="fl" type="range" min="-100" max="100" value="0"><input id="fln" type="number" min="-100" max="100" value="0"></div>
    <div class="test-controls"><label for="fr">Front Right</label><input id="fr" type="range" min="-100" max="100" value="0"><input id="frn" type="number" min="-100" max="100" value="0"></div>
    <div class="test-controls"><label for="rl">Rear Left</label><input id="rl" type="range" min="-100" max="100" value="0"><input id="rln" type="number" min="-100" max="100" value="0"></div>
    <div class="test-controls"><label for="rr">Rear Right</label><input id="rr" type="range" min="-100" max="100" value="0"><input id="rrn" type="number" min="-100" max="100" value="0"></div>

    <div class="actionbar">
      <button id="armTest" class="action arm" type="button">ARM REMOTE CONTROL</button>
      <button id="zeroTest" class="action secondary" type="button">SET ALL 0%</button>
      <button id="stopTest" class="action stop" type="button">STOP ALL</button>
      <button id="resetHalls" class="action secondary" type="button">RESET HALL COUNTS</button>
    </div>
    <p class="help">Testing commands use the same ROS → I²C safety path and watchdogs as Drive mode. Switching tabs automatically commands 0%.</p>
  </section>
</section>
</main>

<script>
let controlEnabled = false;
let activeMode = 'drive';
const keyState = {up:false, down:false, left:false, right:false};
let driveCommand = {fl:0,fr:0,rl:0,rr:0};
const motorIds = ['fl','fr','rl','rr'];

const $ = (id) => document.getElementById(id);
const ui = {
  conn:$('conn'), i2c:$('i2c'), armed:$('armed'), stopped:$('stopped'), fresh:$('fresh'), good:$('good'), bad:$('bad'),
  flspeed:$('flspeed'), frspeed:$('frspeed'), rlspeed:$('rlspeed'), rrspeed:$('rrspeed'),
  x:$('x'), y:$('y'), yaw:$('yaw'), vx:$('vx'), wz:$('wz'),
  driveTab:$('driveTab'), testTab:$('testTab'), drivePanel:$('drivePanel'), testPanel:$('testPanel'),
  drivePower:$('drivePower'), drivePowerValue:$('drivePowerValue'), turnPower:$('turnPower'), turnPowerValue:$('turnPowerValue'),
  driveDown:$('driveDown'), driveUp:$('driveUp'), turnDown:$('turnDown'), turnUp:$('turnUp'),
  driveState:$('driveState'), driveDot:$('driveDot'), driveModeText:$('driveModeText'),
  cmdFL:$('cmdFL'), cmdFR:$('cmdFR'), cmdRL:$('cmdRL'), cmdRR:$('cmdRR'),
  upButton:$('upButton'), downButton:$('downButton'), leftButton:$('leftButton'), rightButton:$('rightButton'),
  armDrive:$('armDrive'), zeroDrive:$('zeroDrive'), stopDrive:$('stopDrive'),
  armTest:$('armTest'), zeroTest:$('zeroTest'), stopTest:$('stopTest'), resetHalls:$('resetHalls'),
};

function clamp(v){
  v=Number(v);
  if(!Number.isFinite(v)) return 0;
  return Math.max(-100,Math.min(100,Math.round(v)));
}

async function post(url,obj={}){
  const response=await fetch(url,{
    method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(obj), cache:'no-store'
  });
  if(!response.ok) throw new Error(url+' returned HTTP '+response.status);
  return response;
}

function setTestInputs(value){
  for(const id of motorIds){
    $(id).value=value;
    $(id+'n').value=value;
  }
}

for(const id of motorIds){
  const range=$(id);
  const number=$(id+'n');
  range.addEventListener('input',()=>{ number.value=range.value; });
  number.addEventListener('input',()=>{
    const v=clamp(number.value);
    number.value=v;
    range.value=v;
  });
}

function testValues(){
  return {fl:clamp($('fl').value),fr:clamp($('fr').value),rl:clamp($('rl').value),rr:clamp($('rr').value)};
}

function setCommandDisplay(v){
  ui.cmdFL.textContent=clamp(v.fl)+'%';
  ui.cmdFR.textContent=clamp(v.fr)+'%';
  ui.cmdRL.textContent=clamp(v.rl)+'%';
  ui.cmdRR.textContent=clamp(v.rr)+'%';
}

function updateDriveCommand(){
  const forward=(keyState.up?1:0)-(keyState.down?1:0);
  const turn=(keyState.right?1:0)-(keyState.left?1:0);
  const drivePower=Math.abs(clamp(ui.drivePower.value));
  const turnPower=Math.abs(clamp(ui.turnPower.value));

  // Differential/skid-steer mix:
  // forward/reverse uses Drive %, while left/right uses Turn %.
  // If the mixed request would exceed 100%, scale both sides together
  // to preserve the requested turning ratio rather than clipping one side.
  let left=(forward*drivePower)+(turn*turnPower);
  let right=(forward*drivePower)-(turn*turnPower);
  const peak=Math.max(Math.abs(left),Math.abs(right));
  if(peak>100){
    const scale=100/peak;
    left*=scale;
    right*=scale;
  }

  driveCommand={fl:clamp(left),fr:clamp(right),rl:clamp(left),rr:clamp(right)};
  setCommandDisplay(driveCommand);

  const names=[];
  if(keyState.up) names.push('FORWARD');
  if(keyState.down) names.push('REVERSE');
  if(keyState.left) names.push('LEFT');
  if(keyState.right) names.push('RIGHT');
  ui.driveState.textContent=names.length?names.join(' + '):'IDLE';

  ui.upButton.classList.toggle('active',keyState.up);
  ui.downButton.classList.toggle('active',keyState.down);
  ui.leftButton.classList.toggle('active',keyState.left);
  ui.rightButton.classList.toggle('active',keyState.right);

  if(controlEnabled && activeMode==='drive') sendCurrentControl();
}

function releaseDriveKeys(send=true){
  keyState.up=keyState.down=keyState.left=keyState.right=false;
  updateDriveCommand();
  if(send && controlEnabled && activeMode==='drive') sendCurrentControl();
}

function currentValues(){
  return activeMode==='drive' ? driveCommand : testValues();
}

async function sendCurrentControl(){
  if(!controlEnabled) return;
  const v=currentValues();
  try {
    await post('/api/control',{...v,active:true});
  } catch(e) {
    ui.conn.textContent='CONTROL LINK ERROR';
    ui.conn.className='bad';
  }
}

async function armRemote(){
  releaseDriveKeys(false);
  setTestInputs(0);
  controlEnabled=true;
  try {
    await post('/api/arm');
    await sendCurrentControl();
  } catch(e) {
    controlEnabled=false;
    ui.conn.textContent='ARM FAILED';
    ui.conn.className='bad';
  }
}

async function zeroCommand(){
  releaseDriveKeys(false);
  setTestInputs(0);
  driveCommand={fl:0,fr:0,rl:0,rr:0};
  setCommandDisplay(driveCommand);
  await sendCurrentControl();
}

async function stopAll(){
  controlEnabled=false;
  releaseDriveKeys(false);
  setTestInputs(0);
  driveCommand={fl:0,fr:0,rl:0,rr:0};
  setCommandDisplay(driveCommand);
  try {
    await post('/api/stop');
  } catch(e) {
    ui.conn.textContent='STOP REQUEST FAILED';
    ui.conn.className='bad';
  }
}

function setMode(mode){
  if(mode===activeMode) return;
  // Zero before changing control semantics.
  releaseDriveKeys(false);
  setTestInputs(0);
  driveCommand={fl:0,fr:0,rl:0,rr:0};
  setCommandDisplay(driveCommand);
  if(controlEnabled) post('/api/control',{fl:0,fr:0,rl:0,rr:0,active:true}).catch(()=>{});

  activeMode=mode;
  const drive=mode==='drive';
  ui.driveTab.classList.toggle('active',drive);
  ui.testTab.classList.toggle('active',!drive);
  ui.driveTab.setAttribute('aria-selected',String(drive));
  ui.testTab.setAttribute('aria-selected',String(!drive));
  ui.drivePanel.classList.toggle('active',drive);
  ui.testPanel.classList.toggle('active',!drive);
}

ui.driveTab.addEventListener('click',()=>setMode('drive'));
ui.testTab.addEventListener('click',()=>setMode('test'));
function setDrivePower(value){
  const v=Math.max(0,Math.min(100,Math.round(Number(value)||0)));
  ui.drivePower.value=v;
  ui.drivePowerValue.textContent=v+'%';
  updateDriveCommand();
}

function setTurnPower(value){
  const v=Math.max(0,Math.min(100,Math.round(Number(value)||0)));
  ui.turnPower.value=v;
  ui.turnPowerValue.textContent=v+'%';
  updateDriveCommand();
}

function nudgeDrive(delta){ setDrivePower(Number(ui.drivePower.value)+delta); }
function nudgeTurn(delta){ setTurnPower(Number(ui.turnPower.value)+delta); }

ui.drivePower.addEventListener('input',()=>setDrivePower(ui.drivePower.value));
ui.turnPower.addEventListener('input',()=>setTurnPower(ui.turnPower.value));
ui.driveDown.addEventListener('click',()=>nudgeDrive(-5));
ui.driveUp.addEventListener('click',()=>nudgeDrive(+5));
ui.turnDown.addEventListener('click',()=>nudgeTurn(-5));
ui.turnUp.addEventListener('click',()=>nudgeTurn(+5));

ui.armDrive.addEventListener('click',armRemote);
ui.armTest.addEventListener('click',armRemote);
ui.zeroDrive.addEventListener('click',zeroCommand);
ui.zeroTest.addEventListener('click',zeroCommand);
ui.stopDrive.addEventListener('click',stopAll);
ui.stopTest.addEventListener('click',stopAll);
ui.resetHalls.addEventListener('click',()=>post('/api/reset_halls').catch(()=>{}));

function editableTarget(target){
  if(!target) return false;
  const tag=(target.tagName||'').toUpperCase();
  return tag==='INPUT'||tag==='TEXTAREA'||tag==='SELECT'||target.isContentEditable;
}

const arrowMap={ArrowUp:'up',ArrowDown:'down',ArrowLeft:'left',ArrowRight:'right'};
window.addEventListener('keydown',(event)=>{
  if(activeMode!=='drive' || editableTarget(event.target)) return;

  const k=arrowMap[event.key];
  if(k){
    event.preventDefault();
    if(!keyState[k]){
      keyState[k]=true;
      updateDriveCommand();
    }
    return;
  }

  // Power adjustment shortcuts. Ignore auto-repeat so one physical keypress
  // changes the setting by one predictable 5% step.
  if(event.repeat) return;

  if(event.key==='+' || event.key==='='){
    event.preventDefault();
    nudgeDrive(+5);
  } else if(event.key==='-' || event.key==='_'){
    event.preventDefault();
    nudgeDrive(-5);
  } else if(event.key===']'){
    event.preventDefault();
    nudgeTurn(+5);
  } else if(event.key==='['){
    event.preventDefault();
    nudgeTurn(-5);
  }
});
window.addEventListener('keyup',(event)=>{
  const k=arrowMap[event.key];
  if(!k) return;
  event.preventDefault();
  keyState[k]=false;
  updateDriveCommand();
});
window.addEventListener('blur',()=>releaseDriveKeys(true));
document.addEventListener('visibilitychange',()=>{
  if(document.hidden) releaseDriveKeys(true);
});

function bindHold(button,key){
  const press=(event)=>{
    if(activeMode!=='drive') return;
    event.preventDefault();
    keyState[key]=true;
    button.setPointerCapture?.(event.pointerId);
    updateDriveCommand();
  };
  const release=(event)=>{
    if(event) event.preventDefault();
    keyState[key]=false;
    updateDriveCommand();
  };
  button.addEventListener('pointerdown',press);
  button.addEventListener('pointerup',release);
  button.addEventListener('pointercancel',release);
  button.addEventListener('lostpointercapture',release);
}
bindHold(ui.upButton,'up');
bindHold(ui.downButton,'down');
bindHold(ui.leftButton,'left');
bindHold(ui.rightButton,'right');

// Browser -> dashboard heartbeat. The dashboard then publishes ROS setpoints.
setInterval(()=>{ if(controlEnabled) sendCurrentControl(); },100);

async function refresh(){
  try {
    const response=await fetch('/api/status',{cache:'no-store'});
    if(!response.ok) throw new Error('HTTP '+response.status);
    const s=await response.json();
    const c=s.controller||{};

    ui.conn.textContent='CONNECTED';
    ui.conn.className='ok';
    ui.i2c.textContent=(c.i2c_address||'--')+' on bus '+(c.i2c_bus??'--');
    ui.armed.textContent=String(c.remote_armed??'--');
    ui.stopped.textContent=String(c.remote_stopped??'--');
    ui.fresh.textContent=String(c.setpoint_fresh??'--');
    ui.good.textContent=c.good_packets??'--';
    ui.bad.textContent=c.bad_packets??'--';

    const armed=Boolean(c.remote_armed);
    ui.driveDot.classList.toggle('live',controlEnabled && armed);
    ui.driveModeText.textContent=(controlEnabled && armed)?'Keyboard control armed':'Not controlling';

    const w=s.wheel_speeds||[0,0,0,0];
    ui.flspeed.textContent=(Number(w[0])||0).toFixed(3)+' m/s';
    ui.frspeed.textContent=(Number(w[1])||0).toFixed(3)+' m/s';
    ui.rlspeed.textContent=(Number(w[2])||0).toFixed(3)+' m/s';
    ui.rrspeed.textContent=(Number(w[3])||0).toFixed(3)+' m/s';

    const o=s.odom||{};
    ui.x.textContent=(Number(o.x)||0).toFixed(3)+' m';
    ui.y.textContent=(Number(o.y)||0).toFixed(3)+' m';
    ui.yaw.textContent=(Number(o.yaw_deg)||0).toFixed(1)+'°';
    ui.vx.textContent=(Number(o.vx)||0).toFixed(3)+' m/s';
    ui.wz.textContent=(Number(o.wz)||0).toFixed(3)+' rad/s';
  } catch(e) {
    ui.conn.textContent='NO DATA';
    ui.conn.className='bad';
  }
}

setDrivePower(ui.drivePower.value);
setTurnPower(ui.turnPower.value);
setCommandDisplay(driveCommand);
updateDriveCommand();
setInterval(refresh,250);
refresh();
</script>
</body>
</html>
"""


class MotorDashboard(Node):

    def __init__(self):
        super().__init__("motor_dashboard")

        self.declare_parameter("http_port", 8080)
        self.declare_parameter("browser_timeout_s", 0.60)

        self.http_port = int(self.get_parameter("http_port").value)
        self.browser_timeout_s = float(
            self.get_parameter("browser_timeout_s").value
        )

        self.lock = threading.Lock()

        self.wheel_speeds = [0.0, 0.0, 0.0, 0.0]
        self.controller_status = {}
        self.odom = {
            "x": 0.0,
            "y": 0.0,
            "yaw_deg": 0.0,
            "vx": 0.0,
            "wz": 0.0,
        }

        self.command_values = [0.0, 0.0, 0.0, 0.0]
        self.control_enabled = False
        self.last_browser_heartbeat = None
        self.pending_arm = False
        self.pending_stop = False
        self.pending_reset_halls = False

        self.setpoint_pub = self.create_publisher(
            Float32MultiArray,
            "/motor/setpoints",
            10,
        )
        self.arm_pub = self.create_publisher(Empty, "/motor/arm", 10)
        self.stop_pub = self.create_publisher(Empty, "/motor/stop", 10)
        self.reset_pub = self.create_publisher(
            Empty,
            "/motor/reset_halls",
            10,
        )

        self.create_subscription(
            Float32MultiArray,
            "/motor/wheel_speeds",
            self.on_wheel_speeds,
            10,
        )
        self.create_subscription(
            Odometry,
            "/wheel_odom",
            self.on_odom,
            10,
        )
        self.create_subscription(
            String,
            "/motor/controller_status",
            self.on_controller_status,
            10,
        )

        # 10 Hz browser -> ROS heartbeat publisher.
        self.command_timer = self.create_timer(0.05, self.publish_commands)

        self.httpd = self.make_http_server()
        self.http_thread = threading.Thread(
            target=self.httpd.serve_forever,
            daemon=True,
        )
        self.http_thread.start()

        self.get_logger().info(
            f"SAADS dashboard listening on 0.0.0.0:{self.http_port}"
        )
        self.get_logger().info(
            f"Open http://192.168.50.20:{self.http_port} over the PtP link"
        )

    def on_wheel_speeds(self, msg):
        if len(msg.data) == 4:
            with self.lock:
                self.wheel_speeds = [float(v) for v in msg.data]

    def on_controller_status(self, msg):
        try:
            parsed = json.loads(msg.data)
        except Exception:
            return

        with self.lock:
            self.controller_status = parsed

    def on_odom(self, msg):
        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        with self.lock:
            self.odom = {
                "x": float(msg.pose.pose.position.x),
                "y": float(msg.pose.pose.position.y),
                "yaw_deg": math.degrees(yaw),
                "vx": float(msg.twist.twist.linear.x),
                "wz": float(msg.twist.twist.angular.z),
            }

    def publish_commands(self):
        with self.lock:
            pending_arm = self.pending_arm
            pending_stop = self.pending_stop
            pending_reset = self.pending_reset_halls

            self.pending_arm = False
            self.pending_stop = False
            self.pending_reset_halls = False

            enabled = self.control_enabled
            heartbeat = self.last_browser_heartbeat
            values = list(self.command_values)

        if pending_arm:
            self.arm_pub.publish(Empty())

        if pending_stop:
            self.stop_pub.publish(Empty())

        if pending_reset:
            self.reset_pub.publish(Empty())

        if not enabled:
            return

        if heartbeat is None:
            return

        age = time.monotonic() - heartbeat

        if age > self.browser_timeout_s:
            # Stop publishing. The bridge's own setpoint timeout then forces
            # zero, while the bridge continues its I2C heartbeat to ESP32.
            with self.lock:
                self.control_enabled = False
            return

        msg = Float32MultiArray()
        msg.data = [float(v) for v in values]
        self.setpoint_pub.publish(msg)

    def set_control(self, payload):
        values = []

        for key in ("fl", "fr", "rl", "rr"):
            value = float(payload.get(key, 0.0))

            if not math.isfinite(value):
                raise ValueError("non-finite command")

            value = max(-100.0, min(100.0, value))
            values.append(value)

        with self.lock:
            self.command_values = values
            self.control_enabled = bool(payload.get("active", True))
            self.last_browser_heartbeat = time.monotonic()

    def request_arm(self):
        with self.lock:
            self.command_values = [0.0, 0.0, 0.0, 0.0]
            self.control_enabled = True
            self.last_browser_heartbeat = time.monotonic()
            self.pending_arm = True

    def request_stop(self):
        with self.lock:
            self.command_values = [0.0, 0.0, 0.0, 0.0]
            self.control_enabled = False
            self.pending_stop = True

    def request_reset_halls(self):
        with self.lock:
            self.pending_reset_halls = True

    def get_status_snapshot(self):
        with self.lock:
            return {
                "wheel_speeds": list(self.wheel_speeds),
                "controller": dict(self.controller_status),
                "odom": dict(self.odom),
                "control_enabled": self.control_enabled,
            }

    def make_http_server(self):
        node = self

        class Handler(BaseHTTPRequestHandler):

            def log_message(self, _format, *_args):
                return

            def send_bytes(self, code, content_type, body):
                self.send_response(code)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def read_json(self):
                length = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(length) if length > 0 else b"{}"
                return json.loads(raw.decode("utf-8"))

            def do_GET(self):
                if self.path == "/" or self.path.startswith("/?"):
                    self.send_bytes(
                        200,
                        "text/html; charset=utf-8",
                        DASHBOARD_HTML.encode("utf-8"),
                    )
                    return

                if self.path == "/api/status":
                    body = json.dumps(
                        node.get_status_snapshot(),
                        separators=(",", ":"),
                    ).encode("utf-8")
                    self.send_bytes(200, "application/json", body)
                    return

                self.send_bytes(404, "text/plain", b"Not found")

            def do_POST(self):
                try:
                    if self.path == "/api/control":
                        node.set_control(self.read_json())
                    elif self.path == "/api/arm":
                        node.request_arm()
                    elif self.path == "/api/stop":
                        node.request_stop()
                    elif self.path == "/api/reset_halls":
                        node.request_reset_halls()
                    else:
                        self.send_bytes(404, "text/plain", b"Not found")
                        return

                    self.send_bytes(
                        200,
                        "application/json",
                        b'{"ok":true}',
                    )

                except Exception as exc:
                    body = json.dumps(
                        {"ok": False, "error": str(exc)}
                    ).encode("utf-8")
                    self.send_bytes(400, "application/json", body)

        return ThreadingHTTPServer(("0.0.0.0", self.http_port), Handler)

    def destroy_node(self):
        try:
            # Request STOP once before shutting down the dashboard.
            self.stop_pub.publish(Empty())
        except Exception:
            pass

        try:
            self.httpd.shutdown()
            self.httpd.server_close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDashboard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
