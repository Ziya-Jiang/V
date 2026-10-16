#!/usr/bin/env python3
"""V4.1：闭指→头部上沿Y对齐→抬升→前移→腕部中心列Y伺服。

--execute 显式执行上述动作；--observe-head 只识别；其余为电流/示教工具。
到对齐即保持，不自动下降、接触或拨书。模块导入不连接设备。
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import select
import statistics
import sys
import time
import urllib.request
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parent
CONFIG = ROOT/'v4_1_config.json'


def dump(path, data):
    Path(path).write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')


def load_config(path=CONFIG):
    c=json.loads(Path(path).read_text())
    if c.get('contact_geometry')!='right_gripper_both_fingers_on_book_top_edge':
        raise ValueError('此入口仅用于右夹爪双指，不能复用旧单指示教')
    u=urlsplit(c['telemetry_url'])
    if u.scheme!='http' or u.hostname not in ('127.0.0.1','localhost') or u.username or u.password or u.path not in ('','/'):
        raise ValueError('电流接口只允许本机HTTP')
    for k in ('contact_validated','automatic_descent_enabled','automatic_pull_enabled'):
        if c[k] is not False:
            raise ValueError('当前是先采集验证版，不能通过配置开关启用未经验证的接触运动')
    if c['contact_threshold_A'] is not None:
        raise ValueError('阈值尚未验证；比较结果保存在报告中，不直接装入运动控制')
    bounds={'baseline_seconds':(2,10),'max_feedback_age_seconds':(.05,.5),
        'max_sample_gap_seconds':(.02,.2),'baseline_max_velocity_rad_s':(.001,.02),
        'baseline_max_joint_spread_rad':(.0001,.01),'minimum_baseline_samples':(20,1000),
        'contact_persistence_seconds':(.1,1),'minimum_comparison_trials':(3,20),
        'maximum_start_pose_difference_rad':(.001,.05),'teaching_pixel_tolerance':(1,5),
        'teaching_angle_tolerance_deg':(.5,5),'target_angle_deg':(1,45)}
    for k,(lo,hi) in bounds.items():
        if type(c[k]) not in (int,float) or not math.isfinite(c[k]) or not lo<=c[k]<=hi:raise ValueError('Invalid '+k)
    if type(c['contact_joint']) is not int or not 1<=c['contact_joint']<=7:raise ValueError('Invalid contact_joint')
    from book_top_coarse_alignment import settings
    settings(c)
    if c.get('alignment_reference') != 'verified_wrist_image_center':raise ValueError('Unsupported alignment reference')
    if not isinstance(c.get('target_description'),str) or not c['target_description'].strip():raise ValueError('Missing book description')
    return c


def telemetry(c, after=0):
    with urllib.request.urlopen(c['telemetry_url'].rstrip('/')+'/api/state?after='+str(after),timeout=1) as r:
        data=json.load(r)
    if data.get('unit')!='A' or data.get('source')!='RT PlannerState.MotorTor / Realman current':
        raise ValueError('电流数据源/单位不匹配')
    if not data.get('live') or data.get('age_s') is None or not 0<=data['age_s']<=c['max_feedback_age_seconds']:
        raise ValueError('电流反馈断流/过期；停止记录，不补零')
    return data


def validate_rows(rows, c):
    if not rows:raise ValueError('没有电流样本')
    for i,r in enumerate(rows):
        if type(r['rt_us']) is not int or not math.isfinite(r['t']) or abs(r['t']-r['rt_us']/1e6)>.5:
            raise ValueError('电流样本时间戳无效')
        if i and not 0<r['t']-rows[i-1]['t']<=c['max_sample_gap_seconds']:
            raise ValueError('样本重复/间断；不能用于连续接触验证')
        for k in ('current','q','dq'):
            v=r['arms'][1][k]
            if len(v)!=7 or not all(type(x) in (int,float) and math.isfinite(x) for x in v):
                raise ValueError('无效七关节数据：'+k)
        if r['arms'][1].get('enabled') != [1]*7:raise ValueError('右臂关节未全部使能，不能比较该试验')


def baseline(rows,c):
    validate_rows(rows,c)
    if len(rows)<c['minimum_baseline_samples'] or rows[-1]['t']-rows[0]['t']<c['baseline_seconds']-.15:
        raise ValueError('静止基线时间/样本不足')
    arms=[r['arms'][1] for r in rows]
    if max(abs(x) for a in arms for x in a['dq'])>c['baseline_max_velocity_rad_s']:
        raise ValueError('基线期间机械臂在运动')
    if max(max(a['q'][j] for a in arms)-min(a['q'][j] for a in arms) for j in range(7))>c['baseline_max_joint_spread_rad']:
        raise ValueError('基线期间关节位置不稳定')
    median=[statistics.median(a['current'][j] for a in arms) for j in range(7)]
    return dict(current_A=median,noise_mad_A=[statistics.median(abs(a['current'][j]-median[j]) for a in arms) for j in range(7)],
        joints_rad=[statistics.median(a['q'][j] for a in arms) for j in range(7)],n=len(rows),start=rows[0]['t'],end=rows[-1]['t'])


def record(c,label,seconds,out):
    """Only GET telemetry. User moves arm in the existing controller."""
    if not c['baseline_seconds']+2<=seconds<=300:raise ValueError('记录时长需包含基线和至少2秒观察，最多300秒')
    out.mkdir(parents=True,exist_ok=False)
    start=time.time();data=telemetry(c);boot=data['boot'];last=data['rows'][-1]['id'] if data['rows'] else 0
    rows=[];events=[];base=None;ok=False
    meta=dict(schema='v41_current_trial_v1',label=label,started_unix_s=start,unit='A',arm='right',
        contact_geometry=c['contact_geometry'],source=data['source'],motion_commands_sent=False,
        physical_contact_confirmed=False,status='recording',config=c)
    dump(out/'trial.json',meta)
    print(f'记录 {label}：{out}\n先静止 {c["baseline_seconds"]:.1f} 秒；本程序不移动机器人。',flush=True)
    try:
        with (out/'samples.jsonl').open('w') as f:
            while time.time()-start<seconds:
                d=telemetry(c,last)
                if d['boot']!=boot:raise ValueError('监视器重启，不能拼接两个采集会话')
                fresh=[r for r in d['rows'] if r['id']>last and r['t']>=start]
                if fresh:
                    validate_rows((rows[-1:] if rows else [])+fresh,c)
                    for r in fresh:f.write(json.dumps(r,allow_nan=False)+'\n')
                    f.flush();rows.extend(fresh);last=fresh[-1]['id']
                if base is None and rows and rows[-1]['t']-rows[0]['t']>=c['baseline_seconds']:
                    b=[r for r in rows if r['t']<=rows[0]['t']+c['baseline_seconds']]
                    base=baseline(b,c);events.append(dict(t=rows[-1]['t'],kind='baseline_complete'))
                    print('基线完成。现在可用原控制界面手动做本次对照动作。\n接触时输入 c 回车作人工标记；输入 s 回车结束。标记不是传感器确认。',flush=True)
                if sys.stdin.isatty() and select.select([sys.stdin],[],[],0)[0]:
                    key=sys.stdin.readline().strip().lower()
                    if key=='s':break
                    if key=='c' and base is not None:
                        events.append(dict(t=time.time(),kind='human_contact_marker'));print('已添加人工接触标记',flush=True)
                time.sleep(.04)
        if base is None:raise ValueError('记录未完成静止基线')
        ok=True;meta['status']='recorded'
    except BaseException as exc:
        meta['status']='interrupted';meta['error']=f'{type(exc).__name__}: {exc}';raise
    finally:
        meta.update(ended_unix_s=time.time(),samples=len(rows),baseline=base,events=events)
        dump(out/'trial.json',meta)
        with (out/'currents.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['unix_s']+[f'J{j}_{unit}' for unit in ('A','rad','rad_s') for j in range(1,8)])
            for r in rows:w.writerow([r['t']]+[v for k in ('current','q','dq') for v in r['arms'][1][k]])
        print('记录完成' if ok else '记录未完整完成，已保存已有证据',flush=True)


def read_trial(path,c):
    path=Path(path);m=json.loads((path/'trial.json').read_text())
    rows=[json.loads(x) for x in (path/'samples.jsonl').read_text().splitlines() if x]
    if (m.get('schema')!='v41_current_trial_v1' or m.get('status')!='recorded' or m.get('unit')!='A'
            or m.get('contact_geometry')!=c['contact_geometry'] or not m.get('baseline')):
        raise ValueError(f'无效/未完整完成的对照记录：{path}')
    validate_rows(rows,c)
    b=m['baseline'];actual=baseline([r for r in rows if b['start']<=r['t']<=b['end']],c)
    if any(abs(a-z)>1e-9 for a,z in zip(actual['current_A'],b['current_A'])):raise ValueError('基线与原始记录不一致')
    return m,rows


def sustained_deviation(rows,base,j,duration):
    """Maximum minimum |delta| over continuous windows: rejects single spikes."""
    best=0.;left=0
    for right,r in enumerate(rows):
        while left<right and r['t']-rows[left+1]['t']>=duration:left+=1
        if r['t']-rows[left]['t']>=duration and right-left+1>=5:
            values=[x['arms'][1]['current'][j]-base for x in rows[left:right+1]]
            # A consistent sign is required; oscillation across baseline is not sustained load.
            if min(values)*max(values)>0:best=max(best,min(abs(x) for x in values))
    return best


def compare(c,free_paths,contact_paths,out):
    pairs=[];j=c['contact_joint']-1
    if len(free_paths)!=len(contact_paths):raise ValueError('空载/接触文件夹需一一对应')
    for fp,cp in zip(free_paths,contact_paths):
        fm,fr=read_trial(fp,c);cm,cr=read_trial(cp,c)
        if fm['label']!='free_descent' or cm['label']!='contact':raise ValueError('记录分组标签不匹配')
        pose=max(abs(a-b) for a,b in zip(fm['baseline']['joints_rad'],cm['baseline']['joints_rad']))
        if pose>c['maximum_start_pose_difference_rad']:raise ValueError('对照起始姿态差异过大，不能直接比较J1')
        marks=[x['t'] for x in cm['events'] if x['kind']=='human_contact_marker']
        if not marks:raise ValueError('接触记录缺人工接触时刻；不能自动猜测事件')
        f=[r for r in fr if r['t']>fm['baseline']['end']]
        post=[r for r in cr if marks[0]<=r['t']<=marks[0]+1.]
        if len(f)<20 or len(post)<20 or post[-1]['t']-post[0]['t']<.5:raise ValueError('下降/接触后观察不足')
        free_peak=max(abs(r['arms'][1]['current'][j]-fm['baseline']['current_A'][j]) for r in f)
        held=sustained_deviation(post,cm['baseline']['current_A'][j],j,c['contact_persistence_seconds'])
        pairs.append(dict(free=str(fp),contact=str(cp),free_peak_delta_A=free_peak,
            contact_sustained_delta_A=held,starting_pose_difference_rad=pose,
            contact_marker_source='human',physical_contact_confirmed=False))
    if not pairs:raise ValueError('至少需要一组对照')
    noise=max(x['free_peak_delta_A'] for x in pairs);signal=min(x['contact_sustained_delta_A'] for x in pairs)
    separated=signal>noise*1.5+1e-6
    enough=len(pairs)>=c['minimum_comparison_trials']
    report=dict(status='candidate_for_review' if separated and enough else 'insufficient_evidence',
        joint=j+1,unit='A',pairs=pairs,repeats=len(pairs),separated=separated,
        free_envelope_A=noise,minimum_contact_response_A=signal,
        candidate_threshold_A=(noise+signal)/2 if separated and enough else None,
        persistence_seconds=c['contact_persistence_seconds'],contact_validated=False,
        automatic_descent_enabled=False,automatic_pull_enabled=False,
        limitations=['人工标记可能延迟','电流不等于接触力/下压力','需核对下压速度、姿态和重复实验','候选阈值不自动写入控制器'])
    out.mkdir(parents=True,exist_ok=True);dump(out/'comparison.json',report)
    print(json.dumps(report,ensure_ascii=False,indent=2));return report


def line_features(points):
    a,b=points
    if len(a)!=2 or len(b)!=2 or not all(math.isfinite(x) for p in points for x in p):raise ValueError('需要两个有效二维点')
    d=[b[0]-a[0],b[1]-a[1]]
    if math.hypot(*d)<10:raise ValueError('示教两点距离太短')
    return [(a[0]+b[0])/2,(a[1]+b[1])/2,math.degrees(math.atan2(d[1],d[0]))]


def angle_error(a,b):return (a-b+90)%180-90


def teach(c,out):
    """Camera-only four clicks: true book top segment and the two finger tips."""
    import cv2
    import numpy as np
    from cart_book_grasp_v3_camera import WristCamera
    out.mkdir(parents=True,exist_ok=False);camera=WristCamera()
    points=[];title='V4.1: book edge L,R then left/right finger contact points; Enter save; Esc cancel'
    try:
        f=camera.capture();rgb=f['rgb'];cv2.namedWindow(title,cv2.WINDOW_NORMAL)
        def mouse(event,x,y,flags,param):
            if event==cv2.EVENT_LBUTTONDOWN and len(points)<4:points.append([float(x),float(y)])
            if event==cv2.EVENT_RBUTTONDOWN and points:points.pop()
        cv2.setMouseCallback(title,mouse)
        while True:
            draw=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
            for i,p in enumerate(points):
                cv2.circle(draw,tuple(map(int,p)),5,(0,215,255) if i<2 else (0,255,100),-1)
                cv2.putText(draw,str(i+1),tuple(map(int,p)),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2)
            cv2.imshow(title,draw);key=cv2.waitKey(30)&255
            if key==27:raise ValueError('示教取消')
            if key in (10,13) and len(points)==4:
                book=line_features(points[:2]);fingers=line_features(points[2:])
                e=math.dist(book[:2],fingers[:2]);angle=abs(angle_error(book[2],fingers[2]))
                if e>c['teaching_pixel_tolerance'] or angle>c['teaching_angle_tolerance_deg']:
                    print(f'这张图双指与上沿未对齐：中心差{e:.1f}px，方向差{angle:.1f}°；右键撤点或Esc退出重新摆放。',flush=True);continue
                value=dict(kind='dual_finger_upper_edge_visual_teaching',contact_geometry=c['contact_geometry'],
                    coordinate_system='undistorted_K',book_edge_uv=points[:2],finger_contact_uv=points[2:],
                    target_features_uv_angle=fingers,upper_edge_features=book,
                    calibration_sha256=camera.calibration_hash,created_unix_s=time.time(),
                    physical_contact_confirmed=False,motion_commands_sent=False,
                    **{k:camera.config[k] for k in ('device','width','height')})
                cv2.imwrite(str(out/'wrist_reference.jpg'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
                cv2.imwrite(str(out/'teaching_overlay.jpg'),draw)
                value['image_sha256']=hashlib.sha256((out/'wrist_reference.jpg').read_bytes()).hexdigest()
                dump(out/'teaching.json',value);print('双指视觉示教已保存：',out);return
    finally:camera.close();cv2.destroyAllWindows()


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=CONFIG)
    mode=p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check-config',action='store_true')
    mode.add_argument('--record-current',choices=['free_descent','contact'])
    mode.add_argument('--compare-current',action='store_true')
    mode.add_argument('--teach-upper-edge',action='store_true')
    mode.add_argument('--execute',action='store_true',help='闭指、Y对齐、抬升、前移、腕部中线伺服；对齐后保持')
    mode.add_argument('--observe-head',action='store_true',help='只识别头部书上沿；不连接控制接口')
    p.add_argument('--free',type=Path,nargs='+')
    p.add_argument('--contact',type=Path,nargs='+')
    p.add_argument('--seconds',type=float,default=30)
    p.add_argument('--output',type=Path)
    p.add_argument('--target',help='头部选择目标书的描述')
    a=p.parse_args(argv)
    try:
        c=load_config(a.config)
        if a.target:c['target_description']=a.target
        if a.check_config:
            if c.get('head_camera_review_required',False):
                print('V4.1 格式有效，但图像源待核对：--execute已阻止；--observe-head仍可只读查看。');return 0
            print('V4.1 配置有效：闭指→Y对齐→抬升→前移→腕部中心列Y对齐。\n--execute执行；--observe-head只感知。对齐后保持，不自动下降或拨书。');return 0
        if a.execute or a.observe_head:
            from book_top_coarse_runtime import run
            run(a,c);return 0
        out=a.output or ROOT/'RobotRAM/output'/('v4_1_study_'+time.strftime('%Y%m%d_%H%M%S'))
        if a.record_current:record(c,a.record_current,a.seconds,out)
        elif a.compare_current:
            if not a.free or not a.contact:raise ValueError('需提供--free与--contact对照文件夹')
            compare(c,a.free,a.contact,out)
        else:teach(c,out)
        return 0
    except (ValueError,OSError,KeyError,IndexError) as exc:
        print('V4.1：'+str(exc),file=sys.stderr);return 2
    except KeyboardInterrupt:print('已中断；执行模式已尝试请求保持，具体确认状态见events.jsonl。' if a.execute else '已停止；此模式没有机器人运动指令。');return 130

if __name__=='__main__':raise SystemExit(main())
