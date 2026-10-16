"""CPU differential servo for the visual phase only. No motion optimizer.

Geometry is copied once from the configured robot model at startup. Runtime
uses NumPy/SciPy, joint position streaming, and sampled CPU collision checks.
"""
import json
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

VISUAL_ACTIONS = {'book_v3_xyz_servo', 'book_v3_1_stream'}
CONFIG = Path(__file__).with_name('v3_1_direct_servo.json')


def load_settings(path=CONFIG):
    c = json.loads(Path(path).read_text())
    limits = dict(period_s=(.01,.05), raw_velocity_ratio=(.05,1.), position_gain=(.1,10.), rotation_gain=(.1,10.),
                  linear_speed_m_s=(.005,.1), angular_speed_rad_s=(.02,.4),
                  joint_speed_rad_s=(.05,.6), joint_acceleration_rad_s2=(.1,2.),
                  joint_lead_rad=(.005,.04), damping=(.001,.05),
                  stream_lease_s=(.2,.5), probe_timeout_s=(2.,10.))
    limits['arrival_position_mm']=(.1,3.)
    c.setdefault('arrival_position_mm',2.)
    if set(c) != set(limits): raise ValueError('Invalid direct servo configuration fields')
    for k,(lo,hi) in limits.items():
        if isinstance(c[k],bool) or not isinstance(c[k],(int,float)) or not lo<=c[k]<=hi:
            raise ValueError('Invalid direct servo setting: '+k)
    return c


class CpuCollision:
    """Same configured spheres, ignored pairs and cuboids; no CUDA calls."""
    def __init__(self, cpu, model, self_config, world):
        self.cpu = cpu
        def host(t): return t.detach().cpu().numpy().copy()
        self.spheres = host(model.link_spheres).reshape(-1,4)
        self.links, self.inverse = np.unique(host(model.link_sphere_idx_map).astype(int),return_inverse=True)
        self.offset = host(self_config.offset).reshape(-1)
        if self_config.experimental_kernel:
            self.pairs = host(self_config.thread_location)[:int(self_config.thread_max)].reshape(-1,2).astype(int)
        else:
            self.pairs = np.argwhere(np.triu(host(self_config.collision_matrix).astype(bool),1))
        active = self.spheres[:,3]>0
        self.pairs = self.pairs[active[self.pairs].all(axis=1)]
        self.bounds = host(model.joint_limits.position)
        self.boxes = []
        if any(v for k,v in world.items() if k!='cuboid'):
            raise ValueError('CPU visual servo currently supports configured cuboid obstacles only')
        for box in world.get('cuboid',{}).values():
            pose=np.asarray(box['pose'],float);dims=np.asarray(box['dims'],float)
            if pose.shape!=(7,) or dims.shape!=(3,) or not np.isfinite(np.r_[pose,dims]).all() or (dims<=0).any():
                raise ValueError('Invalid CPU collision cuboid')
            self.boxes.append((pose[:3],Rotation.from_quat(pose[[4,5,6,3]]).as_matrix(),dims/2))

    def check(self, ordered):
        q=np.atleast_2d(ordered)
        if not np.isfinite(q).all() or (q<self.bounds[0]).any() or (q>self.bounds[1]).any():return False
        t=self.cpu.transforms(q,self.links)[:,self.inverse]
        centers=np.einsum('bsij,sj->bsi',t[:,:,:3,:3],self.spheres[:,:3])+t[:,:,:3,3]
        a,b=self.pairs.T
        radius=self.spheres[:,3]+self.offset
        if (np.linalg.norm(centers[:,a]-centers[:,b],axis=-1)<radius[a]+radius[b]).any():return False
        active=self.spheres[:,3]>0
        for origin,rotation,half in self.boxes:
            local=(centers[:,active]-origin)@rotation
            d=np.abs(local)-half
            sdf=np.linalg.norm(np.maximum(d,0),axis=-1)+np.minimum(np.max(d,axis=-1),0)
            if (sdf<self.spheres[active,3]).any():return False
        return True


def norm_limit(x, limit):
    return x*min(1.,limit/max(np.linalg.norm(x),1e-12))


class DifferentialStep:
    def __init__(self,tcp,joint_limits,settings):
        self.tcp=tcp;self.bounds=np.asarray(joint_limits);self.c=settings
        self.command=None;self.velocity=np.zeros(7)

    def reset(self):self.command=None;self.velocity=np.zeros(7)

    def step(self,q,goal,dt):
        """q: both arms, goal: right TCP xyz + wxyz; SI throughout."""
        c=self.c;q=np.asarray(q,float)
        if self.command is None:self.command=q.copy()
        pos,quat=self.tcp(q);position=pos[0,1];rotation=Rotation.from_quat(quat[0,1,[1,2,3,0]])
        eps=1e-4;perturbed=np.tile(q,(7,1));perturbed[np.arange(7),7+np.arange(7)]+=eps
        pp,rr=self.tcp(perturbed)
        jac=np.vstack(((pp[:,1]-position).T/eps,
            (Rotation.from_quat(rr[:,1][:,[1,2,3,0]])*rotation.inv()).as_rotvec().T/eps))
        error=np.r_[goal[:3]-position,
                    (Rotation.from_quat(goal[[4,5,6,3]])*rotation.inv()).as_rotvec()]
        twist=np.r_[norm_limit(c['position_gain']*error[:3],c['linear_speed_m_s']),
                    norm_limit(c['rotation_gain']*error[3:],c['angular_speed_rad_s'])]
        # Scaling makes orientation rows comparable to translation in metres.
        weight=np.array([1.,1.,1.,.15,.15,.15]);a=jac*weight[:,None]
        velocity=a.T@np.linalg.solve(a@a.T+c['damping']**2*np.eye(6),twist*weight)
        velocity*=min(1.,c['joint_speed_rad_s']/max(np.max(np.abs(velocity)),1e-12))
        velocity=self.velocity+np.clip(velocity-self.velocity,
            -c['joint_acceleration_rad_s2']*dt,c['joint_acceleration_rad_s2']*dt)
        proposal=self.command.copy();proposal[7:]+=velocity*dt
        # Limit the whole coordinated lead, not each joint independently:
        # component-wise clipping destroys the Cartesian cancellation in J*dq.
        lead=proposal[7:]-q[7:]
        lead_scale=min(1.,c['joint_lead_rad']/max(np.max(np.abs(lead)),1e-12))
        proposal[7:]=q[7:]+lead_scale*lead
        velocity*=lead_scale
        if not np.isfinite(proposal).all() or (proposal<self.bounds[0]).any() or (proposal>self.bounds[1]).any():
            raise ValueError('Direct servo joint limit')
        return proposal,velocity,error


class DirectVisualServo:
    def __init__(self,planner,collision,settings=None):
        self.p=planner;self.c=load_settings() if settings is None else settings
        self.collision=collision
        self.stepper=DifferentialStep(planner._v31_bridge.tcp,planner.arm_soft_joint_limits,self.c)
        self.active=False;self.output_ready=False;self.last_tick=None;self.request=None
        self.paused=False;self.key=0
        self.z_blocked_since=None;self.z_hold=None;self.last_z_log=-np.inf
        self.recovering_z=False;self.z_recovery_started=None;self.z_recovery_goal=None
        self.last_z_fraction=1.

    def reset(self):
        self.active=False;self.output_ready=False;self.last_tick=None;self.request=None
        self.paused=False;self.stepper.reset()
        self.z_blocked_since=None;self.z_hold=None;self.last_z_log=-np.inf
        self.recovering_z=False;self.z_recovery_started=None;self.z_recovery_goal=None
        self.last_z_fraction=1.

    def configure_panel(self,command):
        # RAW still passes through the RT low-pass filter. A positive target
        # time repeatedly stretches tiny streaming updates (legacy default 5s).
        # Zero disables only that time-based ramp, retaining filter/speed limits.
        command.interpolation_acceleration_time=0.0
        command.no_interpolation_max_velocity_ratio=self.c['raw_velocity_ratio']

    def bounded_z_step(self,snapshot,command,velocity,reference,now):
        """Back off the complete measured-to-command lead, including windup.

        A safety backoff resets feedforward velocity. It is a constrained hold/
        recovery, not a claim that the emergency change is acceleration-smooth.
        """
        q=snapshot['joints'];tcp=self.p._v31_bridge.tcp
        limit=max(.002,abs(snapshot['position'][1,2]-reference))
        samples=np.linspace(q,command,5)
        positions,_=tcp(samples)
        original=float(np.max(np.abs(positions[:,1,2]-reference)))
        fraction=1.;predicted=original
        if original>limit+1e-6:
            fractions=2.**-np.arange(1,8)
            candidates=np.tile(command,(len(fractions),1))
            candidates[:,7:]=q[7:]+fractions[:,None]*(command[7:]-q[7:])
            paths=q[None,None,:]+np.linspace(0,1,5)[None,:,None]*(candidates[:,None,:]-q)
            positions,_=tcp(paths.reshape(-1,len(q)))
            deviations=np.max(np.abs(positions[:,1,2].reshape(-1,5)-reference),axis=1)
            good=np.flatnonzero(deviations<=limit+1e-6)
            if len(good):
                i=good[0];command=candidates[i];samples=paths[i]
                fraction=float(fractions[i]);predicted=float(deviations[i])
                self.z_blocked_since=None;self.z_hold=None
            else:
                # Do not continually move the hold with changing feedback.
                if self.z_blocked_since is None:
                    self.z_blocked_since=now;self.z_hold=command.copy();self.z_hold[7:]=q[7:]
                command=self.z_hold.copy();samples=np.linspace(q,command,5)
                positions,_=tcp(samples)
                predicted=float(np.max(np.abs(positions[:,1,2]-reference)));fraction=0.
                if predicted>limit+1e-6 or now-self.z_blocked_since>=1.:
                    raise ValueError(f'Direct servo Z backoff exhausted: reference_mm={reference*1000:.3f}, '
                        f'actual_mm={snapshot["position"][1,2]*1000:.3f}, predicted_mm={original*1000:.3f}, '
                        f'hold_deviation_mm={predicted*1000:.3f}, limit_mm={limit*1000:.3f}')
            velocity=np.zeros(7)  # discarded lead must not survive in the integrator
            if now-self.last_z_log>=.5:
                print(f'V3.1 direct Z backoff: fraction={fraction:g}, reference_mm={reference*1000:.3f}, '
                      f'actual_mm={snapshot["position"][1,2]*1000:.3f}, predicted_before_mm={original*1000:.3f}, '
                      f'predicted_after_mm={predicted*1000:.3f}, limit_mm={limit*1000:.3f}',flush=True)
                self.last_z_log=now
        else:
            self.z_blocked_since=None;self.z_hold=None
        self.last_z_fraction=fraction
        return command,velocity,samples

    def start_z_recovery(self,snapshot,reference,now,reason):
        self.recovering_z=True;self.z_recovery_started=now
        self.z_recovery_goal=np.r_[snapshot['position'][1],snapshot['orientation'][1]].copy()
        self.z_recovery_goal[2]=reference
        # Keep the last dispatched joint command. Replacing it with measured
        # joints would turn tracking/load error into a new command jump; with
        # a persistent bias this can double the height error through RAW lag.
        self.stepper.velocity[:]=0
        print(f'V3.1 direct Z recovery start: reason={reason}, reference_mm={reference*1000:.3f}, '
              f'actual_mm={snapshot["position"][1,2]*1000:.3f}; hold X/Y/orientation',flush=True)

    def compute(self,receipt):
        from cuarm_state import CuroboStatus
        p=self.p;b=p._v31_bridge;g=b.guard;now=time.monotonic()
        self.output_ready=False
        snapshot=b.snapshot(max_age=.1)
        if g is None:raise ValueError('Direct servo has no output guard')
        target=np.c_[p.target_arm_tool_position,p.target_arm_tool_orientation].copy()
        if not np.isfinite(target).all() or target.shape!=(2,7):raise ValueError('Invalid direct target')
        if not self.active:
            g.reset();g.begin(float(target[1,2]));self.active=True
            self.stepper.reset();self.last_tick=now
            print('V3.1 visual backend: CPU differential servo, RAW joint positions; MotionGen bypassed; '
                  f'raw_target_time_s=0, raw_velocity_ratio={self.c["raw_velocity_ratio"]:g}')
        g.update(snapshot)
        if g.fault:raise ValueError(g.fault)
        if abs(target[1,2]-g.reference)>.002:raise ValueError('Direct target changes fixed Z plane')
        if np.linalg.norm(target[0,:3]-snapshot['position'][0])>.003:
            raise ValueError('Visual servo cannot move left arm')
        dt=now-self.last_tick
        if dt>.1:raise ValueError('Direct servo control deadline missed')
        dt=min(max(dt,.001),self.c['period_s']*2);self.last_tick=now
        if receipt is None or not 0<=now-receipt:
            raise ValueError('Direct servo missing command timestamp')
        lease=self.c['stream_lease_s'] if p.target_description=='book_v3_1_stream' else self.c['probe_timeout_s']
        stale=now-receipt>lease
        if stale and p.target_description!='book_v3_1_stream':raise ValueError('Direct probe/final command timed out')
        if stale:
            if not self.paused:
                self.stepper.command=snapshot['joints'].copy();self.stepper.velocity[:]=0
                print('V3.1 direct servo: visual target lease expired; holding joints')
            self.paused=True;command=self.stepper.command.copy();velocity=np.zeros(7)
        else:
            if self.paused:self.stepper.reset()
            self.paused=False
            error_z=abs(snapshot['position'][1,2]-g.reference)
            if self.recovering_z and error_z<=.00075:
                self.recovering_z=False;self.z_recovery_started=None;self.z_recovery_goal=None
                self.stepper.velocity[:]=0
                print(f'V3.1 direct Z recovery complete: deviation_mm={error_z*1000:.3f}; resume target',flush=True)
            if not self.recovering_z and error_z>=.0015:
                self.start_z_recovery(snapshot,g.reference,now,'measured height margin')
            if self.recovering_z and now-self.z_recovery_started>=2.:
                raise ValueError(f'Direct servo Z recovery timed out: reference_mm={g.reference*1000:.3f}, '
                                 f'actual_mm={snapshot["position"][1,2]*1000:.3f}')
            goal=self.z_recovery_goal.copy() if self.recovering_z else target[1].copy()
            goal[2]=g.reference
            command,velocity,_=self.stepper.step(snapshot['joints'],goal,dt)
        command,velocity,samples=self.bounded_z_step(snapshot,command,velocity,g.reference,now)
        if not stale and not self.recovering_z and self.last_z_fraction<=.125:
            self.start_z_recovery(snapshot,g.reference,now,'severe predicted-path backoff')
            command,velocity,_=self.stepper.step(snapshot['joints'],self.z_recovery_goal,dt)
            command,velocity,samples=self.bounded_z_step(snapshot,command,velocity,g.reference,now)
        grip=list(p.gripper_joint_position)
        ordered=np.asarray([p.reorder_input(q,grip,as_curobo=True) for q in samples])
        if not self.collision.check(ordered):raise ValueError('Direct servo CPU collision/limit check failed')
        # Guard verifies the measured handoff again at actual dispatch.
        ahead,_=b.tcp(snapshot['joints']+snapshot['velocity']*.04)
        snapshot['tcp_velocity']=(ahead[0,1]-snapshot['position'][1])/.04
        b.checked_start=snapshot;b.serial+=1;b.requested_target=target
        self.request=target;self.proposed=command;self.proposed_velocity=velocity
        p.curobo_cmd.arm_joint_command=command.reshape(2,7).tolist()
        p.curobo_cmd.waypoints_size=0;p.curobo_cmd.waypoints=[];p.curobo_cmd.cmd_plan_size=0
        p.status=CuroboStatus.PLANNING_SUCCEEDED
        p.set_gripper_command();self.output_ready=True

    def dispatched(self,accepted):
        if accepted:
            self.stepper.command=self.proposed.copy();self.stepper.velocity=self.proposed_velocity.copy()
        else:
            self.stepper.reset()
