"""Local semantic multiview selection. Wrist identity -> head ID -> head-only geometry.

No GPT client, credentials, network request, camera or robot initialization on
import. Correspondence must be supported by appearance before order is used.
"""
from pathlib import Path
import json
import re
import time
import uuid
import cv2
import numpy as np

from cart_book_grasp_v2_agent import (
    BookSelectionAgent, SelectionError, COLORS as BASE_COLORS, color_fractions as base_colors,
    annotate_candidates, candidate_gallery, load_geometry_config,
)
from cart_book_grasp_v2_instances import clean_masks
from cart_book_grasp_v2_pause import PauseRequested

COLORS = dict(BASE_COLORS, brown=('棕色', '棕', '褐色', '褐', 'brown'))

CONFIG_PATH = Path(__file__).with_name('cart_book_grasp_v2_1_config.json')


def load_config(path=CONFIG_PATH, *, require_semantic_model=True):
    c = json.loads(Path(path).read_text())
    if c.get('backend') != 'local':
        raise ValueError('V2.1 requires backend=local')
    if c.get('expected_books') is not None:
        raise ValueError('expected_books must be null; counts are inferred per view')
    if c.get('semantic_device') not in ('auto', 'cpu', 'cuda'):
        raise ValueError('Invalid semantic_device')
    for field in ('max_candidates', 'semantic_batch_size'):
        if type(c.get(field)) is not int or not 1 <= c[field] <= 64:
            raise ValueError(f'Invalid {field}')
    if require_semantic_model:
        from cart_book_grasp_v2_1_semantics import model_path
        model_path(c)
    load_geometry_config(path)
    if c.get('reference_path') is not None and (not isinstance(c['reference_path'],str) or not c['reference_path'].strip()):
        raise ValueError('reference_path must be null or a nonempty path')
    if c.get('order_direction') not in ('auto', 'same', 'reversed'):
        raise ValueError('order_direction must be auto/same/reversed')
    ranges = {
        'book_min_score': (.1, 1), 'min_area_fraction': (.0001, .1),
        'max_overlap_fraction': (.05, .8), 'min_mask_fill': (.1, .9),
        'min_wrist_gap_fraction': (.001, .3), 'max_row_residual_fraction': (.005, .3),
        'min_head_y_gap_mm': (1, 100), 'max_head_x_span_mm': (10, 600),
        'pose_tolerance_deg': (.05, 3), 'max_observation_age_seconds': (1, 60),
        'semantic_book_margin': (0, 10), 'semantic_identity_min': (.3, 1),
        'semantic_category_floor': (-5, 0), 'semantic_category_transfer_min': (.5, 1),
        'semantic_query_margin': (.1, 10), 'semantic_query_min_logit': (-30, 10),
        'min_color_fused_fraction': (.05, 1),
        'color_min_fraction': (.1, 1), 'color_margin': (.01, .8),
        'color_match_min': (.5, 1), 'pair_margin': (.01, .5),
        'assignment_margin': (.01, .5), 'texture_min_inliers': (4, 100),
        'texture_min_inlier_ratio': (.2, 1), 'texture_min_coverage': (.005, .8),
    }
    for key, (lo, hi) in ranges.items():
        v = c.get(key)
        if type(v) not in (int, float) or not np.isfinite(v) or not lo <= v <= hi:
            raise ValueError(f'Invalid local selector configuration: {key}')
    if type(c['texture_min_inliers']) is not int:
        raise ValueError('texture_min_inliers must be an integer')
    for key in ('head_roi', 'wrist_roi'):
        a = np.asarray(c.get(key), float)
        if a.shape != (4,) or not np.isfinite(a).all() or not (0 <= a[0] < a[2] <= 1 and 0 <= a[1] < a[3] <= 1):
            raise ValueError(f'{key} must be normalized [x0,y0,x1,y1]')
    a = np.asarray(c.get('head_base_bounds_mm'), float)
    if a.shape != (3, 2) or not np.isfinite(a).all() or not np.all(a[:, 0] < a[:, 1]):
        raise ValueError('head_base_bounds_mm must contain X/Y/Z min/max pairs')
    from cart_book_grasp_v2_1_bookends import filter_config
    c['bookend_filter']=filter_config(c)
    return c


def parse_instruction(text):
    """Finite Chinese/pinyin grammar plus supported English appearance concepts.

    Unknown qualifiers/negation never disappear. English-only SigLIP is not
    treated as a multilingual or general instruction-following model.
    """
    if not isinstance(text, str) or not text.strip():
        raise SelectionError('请输入选书指令')
    s = re.sub(r'[\s，。！？,.!?]', '', text.lower())
    pinyin = {'geiwo':'给我', 'bangwo':'帮我', 'zhongjian':'中间', 'zuobian':'左边',
              'zuoce':'左侧', 'youbian':'右边', 'youce':'右侧', 'huangse':'黄色',
              'hongse':'红色', 'zongse':'棕色', 'huise':'灰色', 'lanse':'蓝色',
              'lvse':'绿色', 'baise':'白色', 'heise':'黑色', 'chengse':'橙色',
              'zise':'紫色', 'fense':'粉色'}
    was_pinyin = any(word in s for word in pinyin)
    for word in sorted(pinyin, key=len, reverse=True): s = s.replace(word, pinyin[word])
    if was_pinyin:
        s = re.sub(r'(?:de)?shu(?:ji)?$', '的书', s)
    if any(word in s for word in ('不要','不是','除了','倒数','或者','或','not','except','without')):
        raise SelectionError('暂不支持否定、倒数或多选指令，请给出正向的唯一描述')
    rank = None
    spatial = None
    positions = {'最左边':'left', '左边':'left', '左侧':'left', 'leftmost':'left', 'left':'left',
                 '最右边':'right','右边':'right','右侧':'right','rightmost':'right','right':'right',
                 '中间':'middle','中央':'middle','middle':'middle','center':'middle'}
    found = set()
    for word in sorted(positions, key=len, reverse=True):
        if word in s: found.add(positions[word]); s=s.replace(word,'')
    numerals = {'一':1,'二':2,'三':3,'四':4,'五':5,'六':6,'七':7,'八':8,'九':9,'十':10}
    for match in list(re.finditer(r'第([0-9]+|[一二三四五六七八九十])(?:本)?', s)):
        n = match[1]; value = int(n) if n.isdigit() else numerals[n]
        if rank is not None or value < 1: raise SelectionError('序号不明确')
        rank = value-1; s=s.replace(match[0], '')
    if s.isdigit(): rank=int(s)-1; s=''
    if len(found)>1 or (found and rank is not None) or (rank is not None and rank<0):
        raise SelectionError('位置描述不明确')
    spatial=next(iter(found),None)
    colors=set()
    for word,color in sorted(((w,k) for k,words in COLORS.items() for w in words),key=lambda x:-len(x[0])):
        if word in s: colors.add(color); s=s.replace(word,'')
    if len(colors)>1: raise SelectionError('请指定一种主要颜色')
    color=next(iter(colors),None)
    concepts = {'强化学习':'reinforcement learning', '机器人':'robotics', '机器学习':'machine learning',
                '计算机视觉':'computer vision', '深度学习':'deep learning', 'reinforcementlearning':'reinforcement learning',
                'machinelearning':'machine learning', 'computervision':'computer vision', 'deeplearning':'deep learning',
                'robotics':'robotics'}
    topics=[]
    for word in sorted(concepts,key=len,reverse=True):
        if word in s: topics.append(concepts[word]);s=s.replace(word,'')
    for word in ('please','giveme','about','the','books','book','抓取','拿取','给我','帮我','拿出','选择',
                 '这本','那本','一本','书籍','方面','关于','请','抓','拿','选','的','本','个','书'):
        s=s.replace(word,'')
    if s or len(topics)>1 or not (spatial or rank is not None or color or topics):
        raise SelectionError('描述未被完整理解；支持颜色、左/右/中间、第N本及机器人/学习/视觉主题：'+text)
    query = 'a photo of a '+((color+' ') if color else '')+'book'+((' about '+topics[0]) if topics else '')+'.'
    return dict(rank=rank, spatial=spatial, color=color, topic=topics[0] if topics else None, query=query)


def color_fractions(rgb, mask):
    result = base_colors(rgb, mask)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)[np.asarray(mask, bool)]
    h,s,v = hsv.T
    result['brown'] = float(((h>=5)&(h<23)&(s>=65)&(v>=45)&(v<180)).mean()) if len(h) else 0.
    # Brown is not gray. The semantic query resolves brown/orange lighting cases.
    return result


def _head_points(mask, depth, k, c2w):
    valid = mask & np.isfinite(depth) & (depth > 0)
    yy, xx = np.nonzero(valid)
    if len(xx) < 60 or len(xx) < .5 * mask.sum():
        raise SelectionError('头部候选缺少有效深度；不删除该书后重新排序')
    z = np.asarray(depth[yy, xx], float)
    if np.median(z) > 20:
        z = z * .001
    rays = np.linalg.solve(k, np.array([xx, yy, np.ones(len(xx))]))
    if not np.isfinite(rays).all() or np.any(rays[2] <= 0):
        raise SelectionError('头部反投影无效')
    t = np.asarray(c2w, float).copy()
    if np.linalg.norm(t[:3, 3]) > 10:
        t[:3, 3] *= .001
    return ((rays / rays[2] * z).T @ t[:3, :3].T + t[:3, 3]) * 1000


def build_observations(raw, rgb, c, view, depth=None, k=None, c2w=None, *, diagnostics=None,
                       compute_appearance=True):
    """Variable-count proposals; geometry is not a prerequisite for identity."""
    rgb=np.asarray(rgb)
    if rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3:
        raise SelectionError('无效的原始 RGB 图像')
    if view=='head' and (np.shape(depth)!=rgb.shape[:2] or np.shape(k)!=(3,3) or np.shape(c2w)!=(4,4)):
        raise SelectionError('无效的头部 RGB-D/标定尺寸')
    if view=='head' and (not np.isfinite(k).all() or not np.isfinite(c2w).all()):
        raise SelectionError('头部标定包含无效数值')
    h,w=rgb.shape[:2]
    fc=dict(min_score=c['book_min_score'], min_area_px=max(100,int(h*w*c['min_area_fraction'])),
            mask_nms_iou=.65,nms_min_area_ratio=.65)
    raw=list(raw)
    report=diagnostics if diagnostics is not None else {}
    report.update(raw_count=len(raw), min_score=fc['min_score'], min_area_px=fc['min_area_px'],
        raw_scores=[float(v[0]) if np.isfinite(v[0]) else None for v in raw],
        after_score_count=sum(bool(np.isfinite(v[0]) and v[0]>=fc['min_score']) for v in raw),
        rejected=[], accepted_count=0)
    masks=clean_masks(raw,(h,w),fc)
    report['after_mask_cleanup_count']=len(masks)
    x0,y0,x1,y1=np.asarray(c[f'{view}_roi'])*[w,h,w,h]
    gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
    observations=[]
    for score,mask,box in masks:
        def reject(reason, **details):
            report['rejected'].append(dict(score=float(score),box=box,reason=reason,**details))
        yy,xx=np.nonzero(mask);u,v=float(np.median(xx)),float(np.median(yy))
        if not (x0<=u<x1 and y0<=v<y1):
            reject('outside_image_roi',uv=[u,v]);continue
        rect=cv2.minAreaRect(np.column_stack((xx,yy)).astype(np.float32));rw,rh=rect[1]
        fill=float(mask.sum()/max(1,rw*rh))
        if min(rw,rh)<5 or fill<c['min_mask_fill']:
            reject('thin_or_sparse_mask',min_side_px=float(min(rw,rh)),fill_fraction=fill);continue
        clipped=bool(xx.min()<=1 or yy.min()<=1 or xx.max()>=w-2 or yy.max()>=h-2)
        quality=float(np.clip(np.std(gray[mask])/50.,.25,1.))*(.5 if clipped else 1.)
        item=dict(score=float(score),mask=mask,book_mask=mask,box=box,uv=[u,v],
            colors=color_fractions(rgb,mask) if compute_appearance else {},
            quality=quality,clipped=clipped,geometry=None,geometry_status='not_checked',error='',
            recognition='book_proposal' if view=='head' else 'wrist_book_proposal',proposal_method='sam3_instances')
        if view=='head':
            try:
                xyz=np.median(_head_points(mask,depth,k,c2w),axis=0)
                bounds=np.asarray(c['head_base_bounds_mm'])
                if not ((xyz>=bounds[:,0])&(xyz<=bounds[:,1])).all():
                    reject('outside_base_workspace',base_xyz_mm=xyz.tolist(),bounds_mm=bounds.tolist());continue
                item.update(base_xyz_mm=xyz.tolist(),base_y_mm=float(xyz[1]))
            except (SelectionError,np.linalg.LinAlgError) as error:
                item.update(base_y_mm=None,error=str(error))
        observations.append(item)
    report['accepted_count']=len(observations)
    report['missing_depth_count']=sum(v.get('base_y_mm') is None for v in observations) if view=='head' else 0
    if len(observations)>c['max_candidates']:
        raise SelectionError('候选过多，请缩小工作区；不会截取前几个候选')
    observations.sort(key=lambda item: (-item['base_y_mm'] if item.get('base_y_mm') is not None else float('inf'))
                      if view=='head' else item['uv'][0])
    for i,item in enumerate(observations,1): item['id']=f'{"book" if view=="head" else "wrist"}_{i}'
    return observations


def proposal_filter_summary(view, report):
    name='头部' if view=='head' else '手部'
    text=(f"V2.1 {name}候选：SAM3 原始 {report['raw_count']} → "
          f"分数 ≥ {report['min_score']:g}：{report['after_score_count']} → "
          f"掩码清理 {report['after_mask_cleanup_count']} → 区域/形状筛选 {report['accepted_count']}。")
    reasons={reason:sum(v['reason']==reason for v in report['rejected'])
             for reason in sorted({v['reason'] for v in report['rejected']})}
    if reasons: text+=f' 过滤原因：{reasons}'
    return text


def verified_observations(items, c):
    kept=[item for item in items if item.get('verified_book')]
    # A mask covering two separate surviving instances is a group proposal.
    groups=set()
    for i,a in enumerate(kept):
        contained=[b for j,b in enumerate(kept) if j!=i and b['mask'].sum()<.75*a['mask'].sum()
                   and np.count_nonzero(a['mask']&b['mask'])/b['mask'].sum()>.85]
        if any(np.count_nonzero(b['mask']&d['mask'])/min(b['mask'].sum(),d['mask'].sum())<.1
               for j,b in enumerate(contained) for d in contained[j+1:]): groups.add(i)
    for i in groups: kept[i].update(verified_book=False,semantic_rejection='multiple_book_group')
    kept=[item for i,item in enumerate(kept) if i not in groups]
    for i,a in enumerate(kept):
        for b in kept[i+1:]:
            if np.count_nonzero(a['mask']&b['mask'])/min(a['mask'].sum(),b['mask'].sum())>c['max_overlap_fraction']:
                a['instance_ambiguous']=b['instance_ambiguous']=True
    return kept


def _features(rgb, items):
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    detector = cv2.ORB_create(nfeatures=1800, scaleFactor=1.15, nlevels=8, edgeThreshold=8, fastThreshold=8)
    result = []
    for item in items:
        mask = cv2.erode(item['book_mask'].astype(np.uint8), np.ones((3, 3), np.uint8))
        keypoints, descriptors = detector.detectAndCompute(gray, mask*255)
        # H/S captures chromatic identity. Neutral brightness is not unique ID.
        hist = cv2.calcHist([hsv], [0, 1], mask*255, [18, 8], [0, 180, 0, 256]).reshape(-1)
        hist /= max(1., hist.sum())
        chroma = float(((hsv[:, :, 1] >= 65) & (hsv[:, :, 2] >= 55))[mask.astype(bool)].mean())
        result.append(dict(hist=hist, chroma=chroma, keypoints=keypoints, descriptors=descriptors,
                           area=float(item['book_mask'].sum())))
    return result


def _texture(a, b, c):
    da, db = a['descriptors'], b['descriptors']
    empty = dict(inliers=0, ratio=0., coverage=0., score=0.)
    if da is None or db is None or min(len(da), len(db)) < 4:
        return empty
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    def good(left, right):
        return {m.queryIdx: m.trainIdx for pair in matcher.knnMatch(left, right, k=2)
                if len(pair) == 2 for m, n in [pair] if m.distance < .72*n.distance}
    forward, backward = good(da, db), good(db, da)
    pairs = [(i, j) for i, j in forward.items() if backward.get(j) == i]
    if len(pairs) < 4:
        return empty
    pa = np.float32([a['keypoints'][i].pt for i, _ in pairs])
    pb = np.float32([b['keypoints'][j].pt for _, j in pairs])
    matrix, inliers = cv2.findHomography(pa, pb, cv2.RANSAC, 3.)
    if matrix is None or inliers is None or not np.isfinite(matrix).all():
        return empty
    keep = inliers.ravel().astype(bool); n = int(keep.sum()); ratio = n/len(pairs)
    coverage = min(cv2.contourArea(cv2.convexHull(pa[keep]))/a['area'],
                   cv2.contourArea(cv2.convexHull(pb[keep]))/b['area']) if n >= 3 else 0.
    valid = n >= c['texture_min_inliers'] and ratio >= c['texture_min_inlier_ratio'] and coverage >= c['texture_min_coverage']
    score = min(.99, .8+.19*min(1., n/(2*c['texture_min_inliers']))) if valid else 0.
    return dict(inliers=n, ratio=ratio, coverage=coverage, score=score)


def match_views(head_rgb, head, wrist_rgb, wrist, c, checkpoint=lambda: None):
    hf,wf=_features(head_rgb,head),_features(wrist_rgb,wrist)
    scores=np.zeros((len(head),len(wrist)));evidence=[]
    for i,a in enumerate(hf):
        row=[]
        for j,b in enumerate(wf):
            checkpoint()
            similarity=float(np.minimum(a['hist'],b['hist']).sum())
            color_score=similarity if min(a['chroma'],b['chroma'])>=c['color_min_fraction'] and similarity>=c['color_match_min'] else 0.
            texture=_texture(a,b,c)
            embedding=0.
            if 'semantic_embedding' in head[i] and 'semantic_embedding' in wrist[j]:
                embedding=float(np.clip(np.dot(head[i]['semantic_embedding'],wrist[j]['semantic_embedding']),-1,1))
            # Retain sub-threshold similarities as alternatives when checking
            # the best-vs-runner-up gap; zeroing them creates false uniqueness.
            semantic_score=max(0.,embedding)
            scores[i,j]=max(color_score,texture['score'],semantic_score)
            row.append(dict(head_id=head[i]['id'],wrist_id=wrist[j]['id'],color_similarity=similarity,
                color_usable=bool(color_score),texture=texture,semantic_similarity=embedding,score=float(scores[i,j])))
        evidence.append(row)
    mapping,decision=solve_assignment(scores,c)
    decision.update(pair_evidence=evidence,score_matrix=scores.tolist(),
        matches=[evidence[i][j] for i,j in enumerate(mapping) if j is not None],
        unmatched_head_ids=[head[i]['id'] for i,j in enumerate(mapping) if j is None],
        unmatched_wrist_ids=[v['id'] for j,v in enumerate(wrist) if j not in mapping])
    return mapping,decision


def solve_assignment(scores,c):
    """Partial one-to-one association, requiring mutual distinct best matches.

    Rectangular sets and arbitrary permutations are allowed. Uncertain rows stay
    unmatched; another book's ambiguity must not manufacture this book's ID.
    """
    scores=np.asarray(scores,float)
    if scores.ndim!=2 or not np.isfinite(scores).all(): raise SelectionError('无效的跨视角评分矩阵')
    mapping=[None]*len(scores)
    for i,row in enumerate(scores):
        if not len(row): continue
        j=int(np.argmax(row));score=float(row[j])
        alternatives=np.r_[np.delete(row,j),np.delete(scores[:,j],i)]
        gap=score-(float(alternatives.max()) if alternatives.size else 0.)
        if score>=max(c['color_match_min'],c['semantic_identity_min']) and gap>=c['pair_margin'] and int(np.argmax(scores[:,j]))==i:
            mapping[i]=j
    paired=[j for j in mapping if j is not None]
    direction=('unknown' if len(paired)<2 else 'same' if np.all(np.diff(paired)>0)
               else 'reversed' if np.all(np.diff(paired)<0) else 'mixed')
    return tuple(mapping),dict(order_direction=direction,method='local_semantic_appearance',
        assignment_score=float(np.mean([scores[i,j] for i,j in enumerate(mapping) if j is not None])) if paired else 0.)


def choose_target(rule,head,wrist,mapping,c):
    if not head: raise SelectionError('头部没有确认书本，未生成抓取目标')
    rank=rule.get('rank');spatial=rule.get('spatial')
    if spatial or rank is not None:
        if any(v.get('base_y_mm') is None or v.get('instance_ambiguous') for v in head):
            raise SelectionError('头部书本位置/实例不完整，不能可靠计算空间顺序')
        if len(wrist)>len(head):
            raise SelectionError('手部有尚未对应到头部的书，空间排序可能漏书；请调整视角')
        if len(head)>1 and np.min(-np.diff([v['base_y_mm'] for v in head]))<c['min_head_y_gap_mm']:
            raise SelectionError('头部基座 Y 排序不明确')
        if spatial=='left': rank=0
        elif spatial=='right': rank=len(head)-1
        elif spatial=='middle':
            if len(head)%2==0: raise SelectionError('偶数本书没有唯一的中间一本，请指定序号或颜色')
            rank=len(head)//2
    matches=[]
    for i,item in enumerate(head):
        if rank is not None and i!=rank: continue
        j=mapping[i]
        if j is None or item.get('base_y_mm') is None or item.get('instance_ambiguous'): continue
        partner=wrist[j]
        if partner.get('instance_ambiguous'): continue
        weights=np.array([item.get('quality',1.),partner.get('quality',1.)]);weights/=weights.sum()
        color=rule.get('color')
        if color is not None:
            fraction=float(weights@np.array([item['colors'][color],partner['colors'][color]]))
            if fraction<c['min_color_fused_fraction']: continue
        score=float(weights@np.array([item.get('semantic_query_logit',0.),partner.get('semantic_query_logit',0.)]))
        item['fused_query_logit']=score
        if (color or rule.get('topic')) and score<c['semantic_query_min_logit']: continue
        matches.append((score,item))
    if not matches: raise SelectionError('没有找到同时满足文字描述、跨视角对应和头部深度的目标')
    matches.sort(key=lambda pair:pair[0],reverse=True)
    if len(matches)>1 and (not (rule.get('color') or rule.get('topic')) or
            matches[0][0]-matches[1][0]<c['semantic_query_margin']):
        raise SelectionError(f'指令匹配 {len(matches)} 本近似书籍，请补充颜色或位置')
    return matches[0][1]


def check_ready_pose(robot, c):
    if robot is None:
        raise SelectionError('缺少 first_reset 实际关节状态')
    state = robot.get_current_state()
    if not isinstance(state, dict) or state.get('joint_state') is None:
        raise SelectionError('缺少 first_reset 实际关节状态：joint_state 为空')
    try:
        joints = np.array(state['joint_state'], dtype=float, copy=True)
        ready = np.array(getattr(robot, 'right_safe_joint_deg', None), dtype=float, copy=True)
    except (TypeError, ValueError) as exc:
        raise SelectionError('无效的 first_reset 关节状态：关节数据必须为数值数组') from exc
    # CuroboState is [left, right] x 7 radians; legacy adapters return flat 14.
    # Validate before reshaping so a transposed (7, 2) state cannot swap joints.
    if joints.shape not in ((2, 7), (14,)):
        raise SelectionError(f'无效的 first_reset 关节状态：joint_state 形状为 {joints.shape}，应为 (2, 7) 或 (14,)')
    if ready.shape != (7,):
        raise SelectionError(f'无效的 first_reset 目标关节状态：right_safe_joint_deg 形状为 {ready.shape}，应为 (7,)')
    if not np.isfinite(joints).all() or not np.isfinite(ready).all():
        raise SelectionError('无效的 first_reset 关节状态：关节数据包含 NaN 或 Inf')
    right_joint_deg = np.rad2deg(joints.reshape(2, 7)[1])
    error = float(np.max(np.abs(right_joint_deg-ready)))
    if error > c['pose_tolerance_deg']:
        raise SelectionError(f'当前右臂未到 first_reset 观察姿态：最大误差 {error:.2f}°')
    return dict(right_joint_deg=right_joint_deg.tolist(), expected_right_joint_deg=ready.tolist(), max_error_deg=error)


class LocalBookSelectionAgent(BookSelectionAgent):
    label = '本地双视角语义'

    def __init__(self, config_path=CONFIG_PATH, *, observe_only=False, output_root=None, semantic_model=None):
        # Intentionally do not call the parent constructor (it creates GPTBookClient).
        self.config_path = Path(config_path)
        self.checkpoint = lambda: None
        self.instruction = ''
        self.robot = None
        self.observe_only = observe_only
        self.semantic_model = semantic_model
        self._model_signature = None
        self.output_root = Path(output_root or Path(__file__).parent/'RobotRAM/output/v2_1')
        self._reset()

    def _reset(self):
        self.last_candidates = []; self.last_decision = {}; self.last_image = None
        self.last_filter = {}; self.last_gate_image = None; self.last_review_image = None
        self.last_wrist_view = None; self.last_wrist_image = None; self.last_wrist_candidates_image = None

    def check_ready(self):
        config = load_config(self.config_path)
        if config.get('reference_path') and not (self.config_path.parent/config['reference_path']).is_file():
            raise SelectionError('配置的参考文件不存在；未初始化机器人')
        from cart_book_grasp_v2_dual_view import load_config as camera_config
        if not camera_config()['enabled']:
            raise SelectionError('V2.1 要求开启手部相机')

    def bind_observation_robot(self, robot):
        check_ready_pose(robot, load_config(self.config_path))
        self.robot = robot

    def warmup(self):
        """Load semantic weights before acquiring time-sensitive RGB-D."""
        from cart_book_grasp_v2_1_semantics import get_model, BOOK_TEXTS, OTHER_TEXTS
        c=load_config(self.config_path)
        if self.semantic_model is None: self.semantic_model=get_model(c)
        self.semantic_model.encode([np.full((224,224,3),127,np.uint8)],BOOK_TEXTS+OTHER_TEXTS)
        self._model_signature=tuple(c[key] for key in ('semantic_model_path','semantic_device','semantic_batch_size'))

    def select_instances(self, rgb, depth, intrinsic, c2w, book_geom, prepare_geometry,
                         on_recognized=None, *, head_captured_at=None, dual_config=None, observation_guard=lambda: None):
        from cart_book_grasp_v2_dual_view import load_config as camera_config, capture_wrist
        self._reset()
        c = load_config(self.config_path)
        signature=tuple(c[key] for key in ('semantic_model_path','semantic_device','semantic_batch_size'))
        if self._model_signature is not None and signature!=self._model_signature:
            raise SelectionError('本地模型路径/设备/批量参数已改变，请重新启动以加载新模型')
        rule = parse_instruction(self.instruction)
        dual = camera_config() if dual_config is None else dual_config
        if not dual['enabled']:
            raise SelectionError('本地双视角选书不能关闭手部相机')
        pose = check_ready_pose(self.robot, c)
        if head_captured_at is None or not np.isfinite(head_captured_at):
            raise SelectionError('缺少本轮头部采集时间')
        started = time.monotonic()
        scene_id = uuid.uuid4().hex
        output = self.output_root/scene_id
        output.mkdir(parents=True, exist_ok=False)
        self.last_decision = dict(scene_id=scene_id, backend='local', instruction=self.instruction,
                                  status='observing', pose=pose, gpt_called=False, output=str(output))
        timings = {}
        def checkpoint():
            self.checkpoint(); observation_guard(); check_ready_pose(self.robot, c)
            age = time.monotonic()-head_captured_at
            if age < 0 or age > c['max_observation_age_seconds']:
                raise SelectionError(f'观测已过期（{age:.1f}s），请重新采集')
        def publish():
            if on_recognized is not None:
                on_recognized()
        try:
            checkpoint()
            view = capture_wrist(dual, head_captured_at, self.checkpoint)
            checkpoint()
            timings['capture_wrist_s'] = time.monotonic()-started
            self.last_wrist_view = view; self.last_wrist_image = view['rgb']
            from PIL import Image
            import hashlib
            Image.fromarray(rgb).save(output/'head_rgb.png')
            Image.fromarray(view['rgb']).save(output/'wrist_rgb.png')
            np.save(output/'depth.npy', depth)
            (output/'camera.json').write_text(json.dumps(dict(K=np.asarray(intrinsic).tolist(), c2w=np.asarray(c2w).tolist(),
                pair_gap_seconds=view['pair_gap_seconds'], pose_verified=True, pose=pose, wrist_camera=dual), indent=2))
            processor, to_numpy, image_cls = book_geom._load_sam3_detector()
            def detect(frame, name):
                checkpoint(); t = time.monotonic()
                raw = book_geom._sam3_raw_books(processor, to_numpy, image_cls.fromarray(frame), prompt='book')
                checkpoint(); timings[name+'_sam3_s'] = time.monotonic()-t
                np.savez_compressed(output/(name+'_raw.npz'), frame_sha256=hashlib.sha256(frame.tobytes()).hexdigest(), scores=np.array([v[0] for v in raw]),
                                    masks=np.array([v[1] for v in raw]), boxes=np.array([v[2] for v in raw]))
                return raw
            head_raw = detect(rgb, 'head'); wrist_raw = detect(view['rgb'], 'wrist')
            self.last_filter=dict(head={},wrist={})
            self.last_decision['proposal_filter']=self.last_filter
            all_head = build_observations(head_raw, rgb, c, 'head', depth, intrinsic, c2w,
                                          diagnostics=self.last_filter['head'])
            all_wrist = build_observations(wrist_raw, view['rgb'], c, 'wrist',
                                           diagnostics=self.last_filter['wrist'])
            for name,report in self.last_filter.items():
                print(proposal_filter_summary(name,report),flush=True)
            self.last_candidates=all_head
            self.last_image=annotate_candidates(rgb,all_head)
            self.last_wrist_image=annotate_candidates(view['rgb'],all_wrist)
            publish()
            from cart_book_grasp_v2_1_semantics import annotate_views
            semantics=annotate_views(rgb,all_head,view['rgb'],all_wrist,rule,c,checkpoint,self.semantic_model)
            self.last_decision['semantic_review']=semantics
            timings['local_semantics_s']=semantics['local_semantics_s']
            head=verified_observations(all_head,c);wrist=verified_observations(all_wrist,c)
            print(f'V2.1 本地语义审核：头部 {len(all_head)} → {len(head)} 本，手部 {len(all_wrist)} → {len(wrist)} 本。',flush=True)
            view['candidates'] = wrist
            self.last_image = annotate_candidates(rgb, head)
            self.last_review_image = semantic_overlay(rgb,all_head)
            self.last_wrist_image = semantic_overlay(view['rgb'],all_wrist)
            self.last_wrist_candidates_image = candidate_gallery(view['rgb'], wrist)
            publish()
            t = time.monotonic()
            if c.get('reference_path'):
                reference = self.config_path.parent/c['reference_path']
                mapping, decision = match_reference(reference, rgb, head, view['rgb'], wrist, c,
                    camera_metadata=dict(K=np.asarray(intrinsic).tolist(), c2w=np.asarray(c2w).tolist(), wrist_camera=dual), pose=pose)
            else:
                mapping, decision = match_views(rgb, head, view['rgb'], wrist, c, checkpoint)
            self.last_decision.update(decision)
            self.last_gate_image=association_overlay(rgb,head,view['rgb'],wrist,mapping)
            publish()
            selected = choose_target(rule, head, wrist, mapping, c)
            timings['local_match_select_s'] = time.monotonic()-t
            self.last_decision.update(decision, status='selected', selected_id=selected['id'],
                                      selected_wrist_id=wrist[mapping[next(i for i, item in enumerate(head) if item is selected)]]['id'])
            self.last_image = annotate_candidates(rgb, head, selected['id'])
            self.last_gate_image=association_overlay(rgb,head,view['rgb'],wrist,mapping,selected['id'])
            self.last_decision['target_scores']=[dict(id=v['id'],fused_query_logit=v.get('fused_query_logit')) for v in head]
            checkpoint(); publish()
            print(f'V2.1 {decision["order_direction"]}: 手部身份 -> {selected["id"]}；仅使用头部几何。', flush=True)
            if self.observe_only:
                self.last_decision['status'] = 'observed_only'
                raise SelectionError('仅观测完成；未计算抓取几何，未执行抓取。')
            t = time.monotonic()
            try:
                prepare_geometry(selected)
                if selected.get('geometry') is None:
                    raise SelectionError('头部目标几何未完成')
            except Exception:
                self.last_decision['status'] = 'geometry_failed'
                raise
            timings['head_geometry_s'] = time.monotonic()-t
            checkpoint()
            return selected
        except PauseRequested:
            self.last_decision['status'] = 'interrupted'
            raise
        except Exception as error:
            if self.last_decision['status'] not in ('observed_only', 'geometry_failed'):
                self.last_decision['status'] = 'rejected'
            self.last_decision['error'] = str(error)
            self.last_decision.update(getattr(error, 'diagnostics', {}))
            raise
        finally:
            timings['total_s'] = time.monotonic()-started
            self.last_decision['timings'] = timings
            self.last_decision['config'] = c
            (output/'decision.json').write_text(json.dumps(self.last_decision, indent=2, ensure_ascii=False))
            for name, frame in [('head_result', self.last_image), ('wrist_result', self.last_wrist_image)]:
                if frame is not None:
                    cv2.imwrite(str(output/(name+'.png')), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            publish()


def _reference_scores(rgb, items, reference_rgb, reference_items, c):
    """Same-camera matching tolerates neutral books; uniqueness still required."""
    a, b = _features(rgb, items), _features(reference_rgb, reference_items)
    def brightness(frame, item):
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        mask = cv2.erode(item['book_mask'].astype(np.uint8),np.ones((3,3),np.uint8))*255
        hist = cv2.calcHist([hsv],[2],mask,[16],[0,256]).ravel()
        return hist/max(1.,hist.sum())
    ah = [brightness(rgb,v) for v in items]
    bh = [brightness(reference_rgb,v) for v in reference_items]
    scores = np.zeros((len(a),len(b)))
    for i in range(len(a)):
        for j in range(len(b)):
            color = float(.65*np.minimum(a[i]['hist'],b[j]['hist']).sum()+.35*np.minimum(ah[i],bh[j]).sum())
            scores[i,j] = max(color,_texture(a[i],b[j],c)['score'])
    mapping,_=solve_assignment(scores,c)
    if any(j is None for j in mapping):
        raise SelectionError('当前画面与同相机参考样本对应不唯一')
    return mapping,scores.tolist()


def match_reference(reference_path, head_rgb, head, wrist_rgb, wrist, c, *, camera_metadata, pose=None):
    """Match each camera to its own registered appearance, then transfer IDs."""
    from PIL import Image
    reference_path=Path(reference_path)
    document=json.loads(reference_path.read_text())
    if document.get('schema') != 1 or document.get('verification') != 'operator_confirmed':
        raise SelectionError('参考样本没有明确登记过跨视角对应')
    if document.get('direction') not in ('same','reversed'):
        raise SelectionError('无效的参考排列方向')
    if document.get('pose_verified') is not True:
        raise SelectionError('参考样本未在 first_reset 姿态采集')
    source=reference_path.parent/document['pair_directory']
    import hashlib
    if hashlib.sha256((source/'camera.json').read_bytes()).hexdigest() != document['sha256']['camera.json']:
        raise SelectionError('参考相机信息已被修改')
    meta=json.loads((source/'camera.json').read_text())
    current=camera_metadata
    for key in ('K','c2w'):
        if not np.allclose(np.asarray(current[key]),np.asarray(meta[key]),atol=1e-5,rtol=1e-5):
            raise SelectionError('头部标定已变化，需要重新登记参考')
    for key in ('device','width','height'):
        if current['wrist_camera'][key] != meta['wrist_camera'][key]:
            raise SelectionError('手部相机配置已变化，需要重新登记参考')
    if pose is not None:
        difference=np.max(np.abs(np.asarray(pose['right_joint_deg'])-np.asarray(meta['pose']['right_joint_deg'])))
        if difference > c['pose_tolerance_deg']:
            raise SelectionError('当前观察姿态与参考样本不同')
    results={}; matrices={}
    for name,rgb,items in [('head',head_rgb,head),('wrist',wrist_rgb,wrist)]:
        path=source/(name+'_rgb.png')
        import hashlib
        if hashlib.sha256(path.read_bytes()).hexdigest() != document['sha256'][name+'_rgb.png']:
            raise SelectionError('参考图片已被修改')
        ref_rgb=np.asarray(Image.open(path).convert('RGB'))
        if ref_rgb.shape != rgb.shape:
            raise SelectionError('参考图与当前图分辨率不同')
        with np.load(source/(name+'_reference_masks.npz'),allow_pickle=False) as data:
            masks=data['masks'].copy()
        if not len(masks) or any(m.shape!=rgb.shape[:2] or not m.any() for m in masks):
            raise SelectionError('无效的参考实例 mask')
        if hashlib.sha256((source/(name+'_reference_masks.npz')).read_bytes()).hexdigest() != document['sha256'][name+'_reference_masks.npz']:
            raise SelectionError('参考 mask 已被修改')
        ref_items=[dict(book_mask=m) for m in masks]
        results[name],matrices[name]=_reference_scores(rgb,items,ref_rgb,ref_items,c)
    reference_mapping=document['head_to_wrist']
    if not isinstance(reference_mapping,list) or sorted(reference_mapping)!=list(range(len(reference_mapping))):
        raise SelectionError('无效的参考跨视角实例关系')
    if any(h>=len(reference_mapping) for h in results['head']):
        raise SelectionError('参考实例关系不完整')
    mapping=tuple(results['wrist'].index(reference_mapping[h]) if reference_mapping[h] in results['wrist'] else None
                  for h in results['head'])
    paired=[j for j in mapping if j is not None]
    direction=('unknown' if len(paired)<2 else 'same' if np.all(np.diff(paired)>0)
               else 'reversed' if np.all(np.diff(paired)<0) else 'mixed')
    return mapping,dict(order_direction=direction,method='same_camera_references',reference=str(reference_path),
        reference_score_matrices=matrices,matches=[dict(head_id=head[i]['id'],wrist_id=wrist[j]['id']) for i,j in enumerate(mapping) if j is not None])


def semantic_overlay(rgb,items):
    image=rgb.copy()
    for item in items:
        contours,_=cv2.findContours(item['book_mask'].astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        color=(0,220,80) if item.get('verified_book') else (255,60,60)
        cv2.drawContours(image,contours,-1,color,2)
        x,y=np.rint(item['uv']).astype(int)
        cv2.putText(image,item['id'],(max(0,x-20),max(18,y)),cv2.FONT_HERSHEY_SIMPLEX,.45,color,1)
    if items:
        cv2.rectangle(image,(5,34),(min(image.shape[1]-1,325),min(image.shape[0]-1,57+18*len(items))),(25,25,25),-1)
        cv2.putText(image,'ID       book margin    text logit',(10,49),cv2.FONT_HERSHEY_SIMPLEX,.4,(240,240,240),1)
        for index,item in enumerate(items):
            color=(70,240,110) if item.get('verified_book') else (255,100,90)
            text=f"{item['id']:<9} {item.get('semantic_book_margin',0):+6.2f}         {item.get('semantic_query_logit',0):+6.2f}"
            cv2.putText(image,text,(10,67+index*18),cv2.FONT_HERSHEY_SIMPLEX,.4,color,1)
    return image


def association_overlay(head_rgb,head,wrist_rgb,wrist,mapping,selected_id=None):
    height=480
    scales=[height/head_rgb.shape[0],height/wrist_rgb.shape[0]]
    images=[cv2.resize(semantic_overlay(rgb,items),(round(rgb.shape[1]*scale),height))
            for rgb,items,scale in zip((head_rgb,wrist_rgb),(head,wrist),scales)]
    canvas=np.concatenate(images,axis=1);offset=images[0].shape[1]
    for i,j in enumerate(mapping):
        if j is None:continue
        start=tuple(np.rint(np.array(head[i]['uv'])*scales[0]).astype(int))
        end=tuple(np.rint(np.array(wrist[j]['uv'])*scales[1]+[offset,0]).astype(int))
        color=(255,230,0) if head[i]['id']==selected_id else (70,230,140)
        cv2.line(canvas,start,end,color,2);cv2.circle(canvas,start,6,color,-1);cv2.circle(canvas,end,6,color,-1)
    cv2.rectangle(canvas,(0,0),(canvas.shape[1],30),(25,25,25),-1)
    cv2.putText(canvas,'HEAD (robot-base order) | WRIST (image order) | lines = verified identity',
                (12,21),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
    return canvas
