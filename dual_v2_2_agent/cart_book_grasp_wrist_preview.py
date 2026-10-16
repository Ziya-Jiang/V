"""Once-per-selection local visual report, including failed detections.

No robot commands, network, model calls or per-servo-frame rendering. The latest
viewer loads a small local JS manifest and only swaps on a new completed report.
"""
import base64
import html
import json
import os
from pathlib import Path
import tempfile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parent/'RobotRAM/output/wrist_first_detection'


def atomic_text(path, text):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    fd,name=tempfile.mkstemp(prefix='.preview-',dir=path.parent)
    try:
        with os.fdopen(fd,'w') as f:f.write(text)
        os.replace(name,path)
    finally:
        if os.path.exists(name):os.unlink(name)


def mask_paths(mask, color):
    contours,_=cv2.findContours(np.asarray(mask,np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    paths=[]
    for contour in contours:
        if cv2.contourArea(contour)<50:continue
        points=cv2.approxPolyDP(contour,1.,True)[:,0]
        paths.append('<polygon points="'+ ' '.join(f'{x},{y}' for x,y in points)+
                     f'" fill="{color}" fill-opacity=".20" stroke="{color}" stroke-width="2"/>')
    return ''.join(paths)


def write_preview(rgb, report, directory, chosen=None, latest_root=None):
    directory=Path(directory).resolve();directory.mkdir(parents=True,exist_ok=True)
    latest=Path(latest_root) if latest_root is not None else ROOT
    h,w=rgb.shape[:2];cy=(h-1)/2;cx=(w-1)/2
    ok,png=cv2.imencode('.png',cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
    if not ok:raise ValueError('Cannot encode wrist preview')
    uri='data:image/png;base64,'+base64.b64encode(png).decode()
    layers=[];notes=[]
    for ai,attempt in enumerate(report.get('attempts',[])):
        prompt=attempt['prompt'];bits=[];archive=directory/f'wrist_detection_{prompt.replace(" ","_")}.npz'
        masks=np.load(archive,allow_pickle=False) if archive.exists() else None
        try:
            for record in attempt.get('candidates',[]):
                i=record['index'];key=f'mask_{i}'
                if masks is not None and key in masks and masks[key].shape==(h,w):
                    bits.append(mask_paths(masks[key], '#e5ad4c' if ai==0 else '#53bfe5'))
                box=record.get('box',record.get('detector_box'))
                if box is not None and len(box)==4:
                    x,y,x2,y2=box
                    bits.append(f'<rect x="{x}" y="{y}" width="{max(0,x2-x)}" height="{max(0,y2-y)}" fill="none" stroke="#e5ad4c"/>')
                score=record.get('score');score='—' if score is None else f'{score:.3f}'
                reason=record.get('reason','几何检查通过')
                notes.append(f'<li>{html.escape(prompt)} #{i} · {score}<br>{html.escape(reason)}</li>')
        finally:
            if masks is not None:masks.close()
        layers.append(f'<g id="mask{ai}" style="display:{"" if ai==len(report.get("attempts",[]))-1 else "none"}">'+''.join(bits)+'</g>')
    proposal=directory/'wrist_gpt_proposal.npz'
    if proposal.exists():
        with np.load(proposal,allow_pickle=False) as archive:
            layers.append('<g id="gpt">'+mask_paths(archive['mask'],'#d098eb')+'</g>')
    if report.get('gpt'):
        gpt=report['gpt'];response=gpt.get('response',{})
        notes.append('<li>GPT-6 · '+html.escape(gpt.get('status',''))+'<br>'+html.escape(str(response.get('reason',gpt.get('reason',''))))+'</li>')
    boundaries=report.get('boundaries',{});bits=[]
    for i,line in enumerate(boundaries.get('edges',[])):
        a,b=line['top'],line['bottom'];x1=line['u']+line['slope']*(a-cy);x2=line['u']+line['slope']*(b-cy)
        bits.append(f'<line x1="{x1}" y1="{a}" x2="{x2}" y2="{b}" stroke="#edb84b" stroke-width="2"/>'
                    f'<text x="{x1+5}" y="{max(18,a+16)}" fill="#fff" stroke="#111" paint-order="stroke" font-size="18">E{i}</text>')
    layers.append('<g id="edges">'+''.join(bits)+'</g>')
    selected=report.get('status')=='selected' and chosen is not None
    if selected:
        layers.append('<g id="selected">'+mask_paths(chosen['mask'],'#53dc9e')+'</g>')
        geom=report.get('selected_geometry',boundaries.get('selected_geometry',{}))
        if geom:
            u=geom['center_u'];slope=np.tan(np.radians(geom['angle_deg']))
            rows=np.flatnonzero(chosen['mask'].any(axis=1));a,b=int(rows[0]),int(rows[-1])
            layers.append(f'<line x1="{u+slope*(a-cy)}" y1="{a}" x2="{u+slope*(b-cy)}" y2="{b}" stroke="#53dc9e" stroke-width="3"/>')
    status='成功 · 已选中书脊' if report.get('status')=='selected' else '失败 · 未下发视觉纠偏目标'
    reason=report.get('reason','绿色为选中的书脊区域与中线。')
    rejected=''.join(f'<li>E{p["left_index"]} / E{p["right_index"]}: {html.escape(p["reason"])}</li>' for p in boundaries.get('rejected_pairs',[]))
    controls=''.join(f'<label><input type="checkbox" {"checked" if i==len(report.get("attempts",[]))-1 else ""} onchange="toggleLayer(\'mask{i}\',this.checked)">{html.escape(a["prompt"])} 掩膜</label>' for i,a in enumerate(report.get('attempts',[])))
    if proposal.exists():controls+='<label><input type="checkbox" checked onchange="toggleLayer(\'gpt\',this.checked)">GPT 候选区域</label>'
    geometry=report.get('selected_geometry',{})
    if geometry.get('support_rows'):
        reason+=' 中线采用连续可见区域：行 '+str(geometry['support_rows'])+'，端部未用于拟合。'
    page=f'''<!doctype html><meta charset="utf-8"><title>腕部首次感知</title>
<style>body{{margin:0;background:#121820;color:#e6edf4;font:15px system-ui}}header{{padding:18px 24px}}h1{{font-size:22px;margin:0 0 8px}}p{{line-height:1.6}}main{{display:grid;grid-template-columns:minmax(0,3fr) minmax(270px,1fr);gap:20px;padding:0 24px 24px}}svg{{width:100%;background:#080d12}}aside{{max-height:80vh;overflow:auto}}li{{margin:10px 0;overflow-wrap:anywhere}}label{{margin-right:16px;display:inline-block}}small{{color:#aab8c4}}details{{margin:16px 0}}@media(max-width:850px){{main{{display:block}}}}</style>
<header><h1>{status}</h1><small>{html.escape(directory.name)} · 仅首次感知，非实时视频</small><p>{html.escape(reason)}</p>
{controls}<label><input type="checkbox" checked onchange="toggleLayer('edges',this.checked)">拟合边界 E编号</label>
<small>黄/蓝：候选，未必通过；绿：选中；白色虚线：图像中心</small></header>
<main><svg viewBox="0 0 {w} {h}" xmlns="http://www.w3.org/2000/svg"><image href="{uri}" width="{w}" height="{h}"/>
{''.join(layers)}<path d="M{cx},0 V{h} M0,{cy} H{w}" stroke="white" stroke-dasharray="7 7" opacity=".75" fill="none"/></svg>
<aside><h3>分割候选与原因</h3><ul>{''.join(notes) or '<li>未返回候选或检测中断</li>'}</ul><details><summary>边界对拒绝原因（{len(boundaries.get('rejected_pairs',[]))}）</summary><ul>{rejected}</ul></details>
<a style="color:#8ac8ff" href="wrist_detection.json">查看完整诊断 JSON</a></aside></main>
<script>function toggleLayer(id,on){{const e=document.getElementById(id);if(e)e.style.display=on?'':'none';}}</script>'''
    path=directory/'first_detection.html';atomic_text(path,page)
    latest.mkdir(parents=True,exist_ok=True)
    viewer='''<!doctype html><meta charset="utf-8"><title>腕部首次感知 · 最新一轮</title>
<style>html,body{margin:0;height:100%;background:#121820;color:#ddd;font:15px system-ui}#bar{height:35px;padding:8px 18px;box-sizing:border-box}iframe{border:0;position:absolute;top:35px;width:100%;height:calc(100% - 35px)}</style>
<div id="bar">每轮首次感知自动更新 · 等待新结果时保留旧画面</div><div id="frame"></div>
<script>let current='';window.wristPreview=function(data){if(current===data.id)return;current=data.id;
const f=document.createElement('iframe');f.style.visibility='hidden';f.src=data.url;
f.onload=()=>{document.getElementById('frame').replaceChildren(f);f.style.visibility='visible';};document.body.appendChild(f);};
function poll(){const s=document.createElement('script');s.src='latest.js?t='+Date.now();s.onload=s.onerror=()=>s.remove();document.head.appendChild(s);}poll();setInterval(poll,2000);</script>'''
    atomic_text(latest/'index.html',viewer)
    manifest=dict(id=str(path)+':'+str(path.stat().st_mtime_ns),url=path.as_uri())
    atomic_text(latest/'latest.js','window.wristPreview('+json.dumps(manifest)+');')
    print(f'首次腕部感知可视化：{path}\n固定查看入口：{latest/"index.html"}')
    return path
