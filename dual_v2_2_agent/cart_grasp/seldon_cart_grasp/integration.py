"""Read-only Seldon cockpit access and explicit actuator interface contract.

Never bind RT UDP ports, start a driver, claim control, change startup mismatch,
call chassis enable, or use a command echo as measured grip evidence.
"""
import json
from pathlib import Path
import urllib.request
import urllib.parse
import ipaddress
import numpy as np
from .geometry import Rejected,finite

def read_arm_state(credentials):
    c=json.loads(Path(credentials).read_text());u=urllib.parse.urlparse(c['url']);token=urllib.parse.parse_qs(u.fragment).get('token',[None])[0]
    if not token:raise Rejected('Cockpit credentials missing token')
    port=int(c.get('port',u.port or 8780))
    if not 1024<=port<=65535:raise Rejected('Invalid cockpit port')
    request=urllib.request.Request('http://127.0.0.1:%d/api/state'%port,headers={'X-Arm-Token':token})
    with urllib.request.urlopen(request,timeout=2) as response:return json.load(response)

def measured_arm_inputs(snapshot):
    if snapshot.get('fresh') is not True or snapshot.get('coordinate_system')!='V2 calibrated':raise Rejected('Need fresh, calibrated V2 feedback')
    age=snapshot.get('age_ms')
    if age is None or not np.isfinite(age) or not 0<=age<=250:raise Rejected('Stale arm state')
    state=snapshot.get('state') or {};arms=state.get('arms',[])
    if len(arms)!=2:raise Rejected('Incomplete arm feedback')
    q=finite([a['position'] for a in arms],(2,7));bounds=finite([a['soft_position'] for a in arms],(2,7,2))
    if snapshot.get('simulation') is not True and not all(all(a['enabled']) for a in arms):raise Rejected('Arms not enabled by operator')
    if state.get('diagnostics',0)&0xffff0000:raise Rejected('RT diagnostic fault')
    return q,bounds

def actuator_contract():
    return {'base_velocity_topic':'/cmd_vel_joy','chassis_services':['/chassis/set_mode_0','/chassis/enable','/chassis/disable'],'arm_state':'GET /api/state (X-Arm-Token, localhost)','arm_commands':'Authenticated cockpit WebSocket; V2 angles converted by existing backend','end_effectors':{'left':{'index':0,'joints':6,'close':[1,.7,.5,.7,.7,.8]},'right':{'index':1,'joints':1,'close':[1],'feedback':'command_echo'}},'unsupported_for_execution':['Atomic arbitrary 14-joint trajectory action absent from current main cockpit','Measured right-gripper contact absent','Calibrated camera/waist/TCP transforms absent','Whole robot/environment continuous collision planner and force-stop transport interface not commissioned'],'policy':'This package publishes proposals only. Hardware command topics/services are never created.'}
