"""Read-only CPU endpoint evidence; never changes planner feasibility or commands.

Uses the existing CPU collision model. These are geometric diagnostics, not an
exact replacement for GPU rollout constraints (which can include extra margins).
"""
import numpy as np


def diagnose_endpoint(collision, joint_names, joints, max_contacts=8):
    q=np.asarray(joints,dtype=float).reshape(-1)
    bounds=collision.bounds
    if q.shape!=(len(joint_names),) or bounds.shape!=(2,len(q)) or not np.isfinite(q).all():
        raise ValueError('Invalid endpoint joint vector')
    # Compare in the model's actual floating-point representation, as CuRobo
    # does on input. Preserve raw decimal excess separately, never widen bounds.
    model_q=q.astype(bounds.dtype)
    raw_bad=(q<bounds[0])|(q>bounds[1])
    model_bad=(model_q<bounds[0])|(model_q>bounds[1])
    bad=np.flatnonzero(model_bad)
    roundoff=np.flatnonzero(raw_bad & ~model_bad)
    out=dict(joint_positions_si=q.tolist(),joint_names=list(joint_names),
             joint_limit_violations=[dict(joint=joint_names[i],value_si=float(q[i]),
                 lower_si=float(bounds[0,i]),upper_si=float(bounds[1,i])) for i in bad])
    out['model_dtype']=str(bounds.dtype)
    out['roundoff_only']=[dict(joint=joint_names[i],value_si=float(q[i]),
        model_value_si=float(model_q[i]),raw_excess_si=float(max(bounds[0,i]-q[i],q[i]-bounds[1,i]))) for i in roundoff]
    transforms=collision.cpu.transforms(q[None,:],collision.links)[0,collision.inverse]
    centers=np.einsum('sij,sj->si',transforms[:,:3,:3],collision.spheres[:,:3])+transforms[:,:3,3]
    a,b=collision.pairs.T
    gaps=np.linalg.norm(centers[a]-centers[b],axis=-1)-(collision.spheres[:,3]+collision.offset)[a]-(collision.spheres[:,3]+collision.offset)[b]
    contacts=np.flatnonzero(gaps<0)
    contacts=contacts[np.argsort(gaps[contacts])]
    names={int(i):name for name,i in collision.cpu.store.items()}
    sphere_links=collision.links[collision.inverse]
    def link(sphere):
        idx=int(sphere_links[sphere]);return dict(index=idx,name=names.get(idx))
    out['self_collision_count']=len(contacts)
    out['self_collision_pairs']=[dict(sphere_a=int(a[i]),sphere_b=int(b[i]),
        link_a=link(a[i]),link_b=link(b[i]),penetration_mm=float(-gaps[i]*1000)) for i in contacts[:max_contacts]]
    active=np.flatnonzero(collision.spheres[:,3]>0)
    world=[]
    for box_index,(origin,rotation,half) in enumerate(collision.boxes):
        local=(centers[active]-origin)@rotation
        d=np.abs(local)-half
        gap=np.linalg.norm(np.maximum(d,0),axis=-1)+np.minimum(np.max(d,axis=-1),0)-collision.spheres[active,3]
        for i in np.flatnonzero(gap<0):world.append(dict(box_index=box_index,sphere=int(active[i]),link=link(active[i]),penetration_mm=float(-gap[i]*1000)))
    world.sort(key=lambda x:-x['penetration_mm'])
    out['world_collision_count']=len(world)
    out['world_collisions']=world[:max_contacts]
    out['cpu_geometry_feasible']=not (len(bad) or len(contacts) or len(world))
    return out


def report_failure(planner, joint_names, start, goal):
    import json
    try:
        collision=planner._v31_direct.collision
        report=dict(action=planner.target_description,source='CPU model diagnostics; GPU constraints remain authoritative',
                    start=diagnose_endpoint(collision,joint_names,start),
                    goal=diagnose_endpoint(collision,joint_names,goal))
        print('V4.1 joint endpoint diagnostics: '+json.dumps(report),flush=True)
    except Exception as error:
        # Diagnostic failure must never replace the original planning failure.
        print(f'V4.1 endpoint diagnostics unavailable: {type(error).__name__}: {error}',flush=True)


def report_optimizer_failure(result):
    """Summarize already-computed GPU results, without re-running any solver."""
    import json
    def summary(value, terminal=False):
        if value is None:return None
        if hasattr(value,'detach'):value=value.detach().cpu().numpy()
        a=np.asarray(value)
        if terminal and a.ndim>1:a=a[..., -1]
        if not a.size:return dict(count=0)
        if a.dtype.kind=='b':return dict(count=int(a.size),passed=int(a.sum()),failed=int((~a).sum()))
        finite=a[np.isfinite(a)]
        return dict(count=int(a.size),nonfinite=int(a.size-finite.size),
                    minimum=float(finite.min()) if finite.size else None,
                    maximum=float(finite.max()) if finite.size else None)
    try:
        debug=result.debug_info or {}
        traj=debug.get('trajopt_result')
        if traj is None:
            print('V4.1 trajopt diagnostics: '+json.dumps(dict(available=False,status=str(result.status))),flush=True)
            return
        metrics=getattr(traj,'metrics',None)
        report=dict(available=True,status=str(result.status),success=summary(traj.success),
                    feasible=summary(getattr(metrics,'feasible',None)),
                    terminal_cspace_error=summary(getattr(metrics,'cspace_error',None),terminal=True),
                    terminal_position_error=summary(getattr(metrics,'position_error',None),terminal=True),
                    terminal_rotation_error=summary(getattr(metrics,'rotation_error',None),terminal=True),
                    optimized_dt_s=summary(getattr(traj,'optimized_dt',None)),
                    smooth_label=summary(getattr(traj,'smooth_label',None)),
                    dt_exception=(traj.debug_info or {}).get('dt_exception'))
        print('V4.1 trajopt diagnostics: '+json.dumps(report),flush=True)
    except Exception as error:
        print(f'V4.1 trajopt diagnostics unavailable: {type(error).__name__}: {error}',flush=True)
