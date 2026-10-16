"""Small box-constrained image Jacobian correction, only for V4.2 roll boundary."""
import numpy as np
from cart_book_grasp_v3_servo import ServoError


def correction_at_roll_bound(preferred,jacobian,error,roll,tolerance,budget,config):
    limit=config['max_roll_deg'];step=config['max_roll_step_deg']
    near=limit-abs(roll)<=step
    if not near and abs(roll+preferred[1])<=limit:
        return preferred,'ordinary'
    extent=min(budget,config['probe_mm'])
    lo=np.array([-extent,max(-step,-limit-roll)])
    hi=np.array([extent,min(step,limit-roll)])
    if np.any(lo>hi):raise ServoError('V4.2 no feasible roll correction interval')
    a=jacobian/tolerance[:,None];b=error/tolerance
    h=a.T@a+.02*np.eye(2);g=config['gain']*(a.T@b)
    # A valid angle should not be unnecessarily corrected through a boundary.
    # Restrict Y so its predicted angular cross-coupling stays inside tolerance.
    if abs(error[1])<=tolerance[1] and lo[1]<=0<=hi[1]:
        lower,upper=lo[0],hi[0];coupling=jacobian[1,0]
        if abs(coupling)>1e-10:
            ends=sorted([(-tolerance[1]-error[1])/coupling,(tolerance[1]-error[1])/coupling])
            lower=max(lower,ends[0]);upper=min(upper,ends[1])
        if lower<=upper:
            dy=float(np.clip(-g[0]/h[0,0],lower,upper))
            candidate=np.array([dy,0.])
            predicted=error+jacobian@candidate
            if (np.abs(error)<=tolerance).all() or np.linalg.norm(predicted/tolerance)<np.linalg.norm(b)-1e-6:
                return candidate,'y_only_roll_boundary'
    # Convex two-variable quadratic: interior optimum and all four edges
    # include every possible box minimum. Re-solve Y when roll is saturated.
    candidates=[]
    unconstrained=-np.linalg.solve(h,g)
    if np.all(unconstrained>=lo) and np.all(unconstrained<=hi):candidates.append(unconstrained)
    for dr in (lo[1],hi[1]):
        candidates.append(np.array([np.clip(-(g[0]+h[0,1]*dr)/h[0,0],lo[0],hi[0]),dr]))
    for dy in (lo[0],hi[0]):
        candidates.append(np.array([dy,np.clip(-(g[1]+h[1,0]*dy)/h[1,1],lo[1],hi[1])]))
    chosen=min(candidates,key=lambda v:float(.5*v@h@v+g@v))
    return chosen,'bounded_y_roll'
