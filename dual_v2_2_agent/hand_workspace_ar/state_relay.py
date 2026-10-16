"""Passive PCAP input -> atomic measured right EE state. Never opens robot sockets."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import sys
import time
from .core import BOOT, WORKSPACE, RUNTIME, atomic_json, ee_from_joints

def packets(stream,destination_port=10165):
    def exact(n):
        data=bytearray()
        while len(data)<n:
            chunk=stream.read(n-len(data))
            if not chunk:
                if not data:return None
                raise ValueError('Truncated PCAP')
            data.extend(chunk)
        return bytes(data)
    header=exact(24)
    if header is None:return
    formats={b'\xd4\xc3\xb2\xa1':('<',1e6),b'\xa1\xb2\xc3\xd4':('>',1e6),b'\x4d\x3c\xb2\xa1':('<',1e9),b'\xa1\xb2\x3c\x4d':('>',1e9)}
    endian,scale=formats[header[:4]]
    link=struct.unpack(endian+'I',header[20:24])[0]
    if link not in (0,1,113,276):raise ValueError(f'Unsupported PCAP link type {link}')
    while True:
        h=exact(16)
        if h is None:return
        sec,sub,n,original=struct.unpack(endian+'IIII',h)
        if n>262144:raise ValueError('Oversized PCAP packet')
        p=exact(n)
        if p is None:raise ValueError('Truncated PCAP packet')
        offset={0:4,1:14,113:16,276:20}[link]
        if link==1 and p[12:14]!=b'\x08\x00':continue
        p=p[offset:]
        if len(p)<28 or p[0]>>4!=4 or p[9]!=17:continue
        ihl=(p[0]&15)*4
        total=struct.unpack('!H',p[2:4])[0]
        if ihl<20 or total>len(p) or total<ihl+8:continue
        if struct.unpack('!H',p[6:8])[0]&0x3fff:continue
        if p[12:16]!=b'\x7f\x00\x00\x01' or p[16:20]!=b'\x7f\x00\x00\x01':continue
        src,dst,length,_=struct.unpack('!HHHH',p[ihl:ihl+8])
        if dst!=destination_port or length<8 or ihl+length>total:continue
        yield sec+sub/scale,p[ihl+8:ihl+length]

def codec():
    config=WORKSPACE.parents[3]/'dual_v2_2_configuration'
    # Import serialization definitions only; bypass all socket constructors.
    for p in [config/'robot_platform_utils/python/scripts',config/'curi_udp/python']:
        sys.path.insert(0,str(p))
    from cuarm_udp import CuarmUdp
    return object.__new__(CuarmUdp)

def decode(payload,stamp,decoder,workspace):
    raw=payload.decode('ascii')
    if not raw.startswith('2#7#7#') or not raw.endswith('#'):raise ValueError('Unexpected state layout')
    state=decoder.unpack_curobo_state(raw)
    q=state.arm_joint_position[1]
    p,R=ee_from_joints(q,workspace['chain'])
    return dict(boot_id=BOOT,captured_unix_s=stamp,frame='dummy_world',reference='arm_end_effector_r',
                position_mm=p.tolist(),joints_rad=q,source='passive measured CuroboState',urdf_sha256=workspace['stats']['sha256'])

def decode_panel(payload,stamp,decoder,workspace):
    # CuarmUdp::send sends strlen(buffer)+1, including the C-string NUL.
    # Older senders may pad the fixed buffer with additional zero bytes.
    # Remove only terminal NULs, never arbitrary/interior data.
    raw=payload.rstrip(b'\x00').decode('ascii')
    if '\x00' in raw:raise ValueError('Embedded NUL in measured PanelState')
    parts=raw.split('#',4)
    if len(parts)!=5 or parts[1:4]!=['2','7','7'] or not raw.endswith('#'):
        raise ValueError('Unexpected measured PanelState layout')
    state=decoder.unpack_panel_state(raw)
    q=state.arm_joint_position[1];p,_=ee_from_joints(q,workspace['chain'])
    return dict(boot_id=BOOT,captured_unix_s=stamp,frame='dummy_world',reference='arm_end_effector_r',
                position_mm=p.tolist(),joints_rad=q,source='PanelState measured joints',source_kind='panel_measured',
                received_unix_s=time.time(),urdf_sha256=workspace['stats']['sha256'])

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=RUNTIME/'state.json')
    args=parser.parse_args()
    workspace=json.loads((WORKSPACE/'forward_workspace_data.json').read_text())
    if hashlib.sha256(Path(workspace['stats']['urdf']).read_bytes()).hexdigest()!=workspace['stats']['sha256']:
        raise ValueError('URDF changed; regenerate workspace before state relay')
    decoder=codec();last=0.;accepted=0;last_error=0.;rejected=0
    for stamp,payload in packets(sys.stdin.buffer,destination_port=30241):
        # Kernel capture time, not pipe arrival time: queued packets cannot revive stale state.
        if not 0<=time.time()-stamp<=.5 or stamp-last<.03:continue
        try:s=decode_panel(payload,stamp,decoder,workspace)
        except (ValueError,IndexError,KeyError,UnicodeError) as e:
            rejected+=1
            if time.monotonic()-last_error>2:
                print(f'Ignoring invalid state ({rejected} total): {e}',file=sys.stderr)
                last_error=time.monotonic()
            continue
        atomic_json(args.output,s);last=stamp;accepted+=1
        if accepted==1:print('Measured right hand state connected (read-only).',file=sys.stderr)
    print(f'State tap ended; accepted {accepted} packets.',file=sys.stderr)

if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:print('State tap stopped.',file=sys.stderr)
