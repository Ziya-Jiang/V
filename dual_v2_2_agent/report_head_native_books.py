"""Build a local diagnostic viewer from the saved head/wrist resolution experiment."""
import argparse
import json
from pathlib import Path
from head_wrist_match_acceptance import annotate_report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    root = parser.parse_args().capture
    report = json.loads((root / 'book_matching/report.json').read_text())
    policy = json.loads(Path(__file__).with_name('head_wrist_match_policy.json').read_text())
    report = annotate_report(report, policy)
    (root / 'book_matching/report_accepted.json').write_text(json.dumps(report, indent=2))
    capture = json.loads((root / 'capture.json').read_text())
    assert len(report['rows']) == 54 and len(report['candidates']) == 162
    summary = {
        'native_profile': capture['selected_profile'],
        'host_gap_ms': [p['host_gap_s'] * 1000 for p in capture['pairs']],
        'geometric_passes': sum(r['screen_pass'] for r in report['rows']),
        'candidate_texture_passes': sum(r['identity_texture_screen'] for r in report['candidates']),
        'relaxed_single_frame_matches': sum(r['match_success'] for r in report['rows']),
        'acceptance_policy': policy,
        'motion_sent': capture['motion_sent'],
        'best_same_candidate_inliers': {
            book: {res: max(r['inliers'] for r in report['candidates']
                            if r['book'] == r['wrist_candidate'] == book and r['resolution'] == res)
                   for res in ['1280x800', '848x530']}
            for book in ['cream', 'red', 'dark']
        },
    }
    (root / 'summary.json').write_text(json.dumps(summary, indent=2))
    # Embed data so this viewer also works when opened directly with file://.
    data = json.dumps(report).replace('<', '\\u003c')
    page = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>最高分辨率 · 头腕匹配诊断</title>
<style>
:root{color-scheme:dark;font-family:system-ui,sans-serif;background:#101619;color:#e8edef}
body{max-width:1450px;margin:auto;padding:32px}h1{font-size:30px;margin-bottom:10px}h2{font-size:19px}
p{line-height:1.8;color:#bac7cc}strong{color:#edc88b}.cards,.pair{display:grid;grid-template-columns:repeat(2,1fr);gap:16px}
.cards{grid-template-columns:repeat(3,1fr)}article,section{background:#192226;border:1px solid #2d3c42;border-radius:14px;padding:18px;margin:16px 0}
.number{font-size:28px;color:#edc88b}select{padding:10px;background:#263338;color:#fff;border:1px solid #51636a;border-radius:8px;margin:5px 12px 5px 0}
img{width:100%;display:block;border-radius:7px}a{color:#eac68b}small{color:#a8b7bd;line-height:1.8}table{width:100%;border-collapse:collapse}td,th{text-align:left;border-bottom:1px solid #35454b;padding:10px}
.bad{color:#eeac9d}.good{color:#8ce2ae}.note{border-left:3px solid #b99353;padding-left:16px}button{cursor:pointer}
@media(max-width:800px){.cards,.pair{grid-template-columns:1fr}body{padding:16px}}
</style>
<h1>最高分辨率 · 头腕匹配诊断</h1>
<p>观测位置重新采集 · Orbbec Gemini 305 · 2026-10-10 · 离线实验</p>
<div class="cards"><article><small>设备当前 SDK 暴露的最大彩色档位</small><div class="number">1280 × 800 / 10 FPS</div></article>
<article><small>3 组图像 × 3 本书 × 2 种分辨率 × 3 种方法</small><div class="number">54 次比较</div></article>
<article><small>新标准：单帧匹配成功 / 54 次比较</small><div class="number">__MATCH_COUNT__ / 54</div></article></div>
<p class="note"><strong>已按用户要求降低匹配阈值；成功结果见下方绿色判定。</strong><br>
本次离线同书候选判定：内点 ≥ __MIN_INLIERS__（原为 12），内点率 ≥ __MIN_RATIO__%，重投影 RMS ≤ __MAX_RMS__ px，源点凸包覆盖 ≥ __MIN_COVERAGE__%，领先其他腕部候选 ≥ __MARGIN__ 个内点。阈值在查看本组结果后调整，尚未经独立数据验证。单帧成功不代表三帧连续跟踪成功；原几何筛选结果保留，未改变在线 V3 运动条件。</p>
<section><h2>查看每次匹配</h2><label>帧 <select id="pair"><option>pair_0</option><option>pair_1</option><option>pair_2</option></select></label>
<label>头部候选 <select id="book"><option value="cream">浅色书</option><option value="red" selected>红色书</option><option value="dark">深色书</option></select></label>
<label>方法 <select id="method"><option value="SuperPoint_LightGlue">SuperPoint + LightGlue 全图</option><option value="SuperPoint_LightGlue_ROI">SuperPoint + LightGlue ROI</option><option value="SIFT_ROI">SIFT ROI</option></select></label>
<div id="views" class="pair"></div><p><small>连线绿色表示 RANSAC 几何内点。图片内原有 screen=False 是旧的完整几何筛选结果；新“匹配成功”判定单独显示在图片下方。图中统计使用整个腕部画面；下表按腕部候选过滤后重新拟合，因此内点数可能不同。</small></p>
<div id="candidates"></div></section>
<section><h2>采集原图</h2><div class="pair"><div><p>原生头部 RGB</p><a id="headLink"><img id="head"></a></div><div><p>腕部图：使用 V3 的 undistorted_K 坐标</p><a id="wristLink"><img id="wrist"></a></div></div></section>
<section><h2>实验边界与下一步</h2><p>低分辨率对照来自同一张原生图缩小到 848 × 530，隔离场景变化，但不等价于单独采集相机低分辨率档位。主机接收时间差 11–37 ms，不是硬件同步。候选多边形为本次观测人工圈选，用于诊断，不是自动选书或已验证的身份真值。</p>
<p>现在可以把满足新门槛的结果作为单帧匹配成功。将结果用于连续伺服前，还需要验证独立数据、连续帧稳定性、腕部跟踪与控制点支持。本次未用少量匹配点外推整本书的控制位置。</p>
<p>原生高清图仅用于本次离线匹配。共享 RGB-D 与 AR 已恢复原来的实时模式，未把原 848 × 530 标定参数直接套到高清图上，未更改 V3 在线动作。</p>
<p><a href="book_matching/report_accepted.json">新标准判定数据</a> · <a href="book_matching/report.json">原始匹配数据</a> · <a href="capture.json">采集参数与时间戳</a> · <a href="profiles.json">设备支持档位</a> · <a href="diagnostic_rois.json">人工诊断区域</a> · <a href="summary.json">摘要数据</a></p></section>
<script>const report=__DATA__;
const labels={cream:'浅色书',red:'红色书',dark:'深色书'};
function render(){
const pair=document.querySelector('#pair').value,book=document.querySelector('#book').value,method=document.querySelector('#method').value;
const selected=report.rows.filter(r=>r.pair===pair&&r.book===book&&r.method===method);
document.querySelector('#views').innerHTML=selected.map(r=>`<article><h2>${r.resolution==='1280x800'?'原生高清 1280 × 800':'同帧缩小 848 × 530'}</h2><a href="book_matching/${r.image}"><img src="book_matching/${r.image}" loading="lazy"></a><p>${r.matches} 个匹配 · ${r.inliers} 个内点 · ${(r.inlier_ratio*100).toFixed(1)}% 内点率<br>源区域特征数 ${r.source_roi_features}<br><strong class="${r.match_success?'good':'bad'}">${r.match_success?'单帧匹配成功 → '+labels[r.matched_candidate]:'单帧匹配未通过'}</strong><br>原完整几何筛选：${r.screen_pass?'通过':'未通过'}</p><small>${r.reasons.join(' / ')}</small></article>`).join('');
const candidates=report.candidates.filter(r=>r.pair===pair&&r.book===book&&r.method===method);
document.querySelector('#candidates').innerHTML='<h2>匹配落在哪本书上</h2><table><thead><tr><th>头部尺寸</th><th>腕部候选</th><th>候选内匹配</th><th>重新拟合内点</th><th>内点率</th><th>新匹配判定</th></tr></thead><tbody>'+candidates.map(r=>`<tr><td>${r.resolution}</td><td>${labels[r.wrist_candidate]}</td><td>${r.matches}</td><td>${r.inliers}</td><td>${(r.inlier_ratio*100).toFixed(1)}%</td><td class="${r.match_success?'good':'bad'}">${r.match_success?'成功':'未通过'}</td></tr>`).join('')+'</tbody></table>';
for(const [id,suffix] of [['head','head_native'],['wrist','wrist_undistorted']]){const path=pair+'_'+suffix+'.png';document.getElementById(id).src=path;document.getElementById(id+'Link').href=path;}
}
document.querySelectorAll('select').forEach(s=>s.addEventListener('change',render));render();</script></html>'''
    replacements = {'__DATA__': data,
                    '__MATCH_COUNT__': str(summary['relaxed_single_frame_matches']),
                    '__MIN_INLIERS__': str(policy['min_inliers']),
                    '__MIN_RATIO__': f"{policy['min_inlier_ratio'] * 100:g}",
                    '__MAX_RMS__': str(policy['max_rms_px']),
                    '__MIN_COVERAGE__': f"{policy['min_source_hull_mask_fraction'] * 100:g}",
                    '__MARGIN__': str(policy['min_inlier_margin'])}
    for token, value in replacements.items():
        page = page.replace(token, value)
    (root / 'index.html').write_text(page)
    print(root / 'index.html')


if __name__ == '__main__':
    main()
