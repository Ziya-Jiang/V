"""Offline geometry for single-finger book tipping. No robot or camera I/O.

The pivot is a point ON an axis, not an unconstrained 3-D point pivot.
All positions are in robot base millimetres. Positive angle uses right-hand
rotation around axis_base. Contact/no-slip are hypotheses, not measurements.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation


def vector(value, name):
    result=np.asarray(value,dtype=float)
    if result.shape!=(3,) or not np.isfinite(result).all():
        raise ValueError(f'{name} must contain three finite values')
    return result.copy()


@dataclass
class TiltReference:
    pivot_base_mm: object
    axis_base: object
    contact_base_mm: object
    tip_offset_tcp_mm: object
    initial_tool_rotation: object
    orientation_mode: str = "fixed"

    def __post_init__(self):
        if self.orientation_mode not in ("fixed","co_rotate"):
            raise ValueError("orientation_mode must be fixed or co_rotate")
        self.pivot_base_mm=vector(self.pivot_base_mm,'pivot')
        self.axis_base=vector(self.axis_base,'axis')
        norm=np.linalg.norm(self.axis_base)
        if norm<1e-8:raise ValueError('Pivot axis cannot be zero')
        self.axis_base/=norm
        self.contact_base_mm=vector(self.contact_base_mm,'contact')
        self.tip_offset_tcp_mm=vector(self.tip_offset_tcp_mm,'finger contact offset')
        matrix=np.asarray(self.initial_tool_rotation,float)
        if (matrix.shape!=(3,3) or not np.isfinite(matrix).all()
                or not np.allclose(matrix.T@matrix,np.eye(3),atol=1e-6)
                or not np.isclose(np.linalg.det(matrix),1.,atol=1e-6)):
            raise ValueError('Tool rotation must be a proper rotation matrix')
        self.initial_tool_rotation=matrix.copy()
        radius=self.contact_base_mm-self.pivot_base_mm
        if np.linalg.norm(np.cross(self.axis_base,radius))<1.:
            raise ValueError('Contact must be off the pivot axis')

    def target(self, angle_deg):
        """Nominal TCP for a fixed material contact on the book.

        This is only a no-slip reference. A freely pushing finger may slip;
        the observed book angle and contact must be checked independently.
        """
        if not np.isfinite(angle_deg):raise ValueError('Angle must be finite')
        rotation=Rotation.from_rotvec(self.axis_base*np.deg2rad(angle_deg)).as_matrix()
        tip=self.pivot_base_mm+rotation@(self.contact_base_mm-self.pivot_base_mm)
        tool=rotation@self.initial_tool_rotation if self.orientation_mode=="co_rotate" else self.initial_tool_rotation.copy()
        tcp=tip-tool@self.tip_offset_tcp_mm
        quat=Rotation.from_matrix(tool).as_quat()
        return dict(tip_base_mm=tip,tcp_base_mm=tcp,tool_rotation=tool,
                    tool_quaternion_wxyz=quat[[3,0,1,2]],angle_deg=float(angle_deg))

    def angle_towards(self, direction_base, magnitude_deg=45.):
        """Choose the sign so initial contact velocity points towards robot."""
        direction=vector(direction_base,'desired tilt direction')
        dot=np.dot(np.cross(self.axis_base,self.contact_base_mm-self.pivot_base_mm),direction)
        if abs(dot)<1e-8:raise ValueError('Desired direction does not determine a tilt sign')
        if not np.isfinite(magnitude_deg) or not 0<magnitude_deg<=90:
            raise ValueError('Requested angle must be in (0,90] degrees')
        return float(np.copysign(magnitude_deg,dot))

    def measured_angle(self, observed_contact_base_mm, observed_pivot_base_mm):
        """Angle from observed book points, not from robot wrist rotation."""
        v0=self.contact_base_mm-self.pivot_base_mm
        v=vector(observed_contact_base_mm,'observed book point')-vector(observed_pivot_base_mm,'observed pivot')
        v0=v0-self.axis_base*np.dot(v0,self.axis_base)
        v=v-self.axis_base*np.dot(v,self.axis_base)
        if min(np.linalg.norm(v0),np.linalg.norm(v))<1.:
            raise ValueError('Book angle is unobservable from these points')
        return float(np.rad2deg(np.arctan2(np.dot(self.axis_base,np.cross(v0,v)),np.dot(v0,v))))

    def incremental_target(self, observed_angle_deg, goal_angle_deg, *, max_tip_step_mm, max_angle_step_deg):
        """Bound the lead from the measured book angle; never integrate blindly."""
        values=[observed_angle_deg,goal_angle_deg,max_tip_step_mm,max_angle_step_deg]
        if not np.isfinite(values).all() or min(max_tip_step_mm,max_angle_step_deg)<=0:
            raise ValueError('Invalid measured angle or step limits')
        radius=np.linalg.norm(np.cross(self.axis_base,self.contact_base_mm-self.pivot_base_mm))
        chord_angle=np.rad2deg(2*np.arcsin(min(1.,max_tip_step_mm/(2*radius))))
        bound=min(chord_angle,max_angle_step_deg)
        angle=observed_angle_deg+np.clip(goal_angle_deg-observed_angle_deg,-bound,bound)
        return self.target(angle)


@dataclass
class FingerPad:
    """Measured rectangular underside patch in TCP coordinates.

    normal_tcp points out of the finger towards the book. A positive signed
    gap means the observed book point is on that side of the pad plane. This
    geometric test does NOT measure force or establish physical contact.
    """
    center_tcp_mm: object
    normal_tcp: object
    along_tcp: object
    length_mm: float
    width_mm: float
    edge_margin_mm: float

    def __post_init__(self):
        self.center_tcp_mm=vector(self.center_tcp_mm,'pad center')
        normal=vector(self.normal_tcp,'pad outward normal')
        along=vector(self.along_tcp,'pad longitudinal axis')
        if min(np.linalg.norm(normal),np.linalg.norm(along))<1e-8:
            raise ValueError('Pad axes cannot be zero')
        normal/=np.linalg.norm(normal);along/=np.linalg.norm(along)
        if abs(np.dot(normal,along))>1e-6:
            raise ValueError('Pad axes must be orthogonal')
        self.normal_tcp=normal;self.along_tcp=along
        self.across_tcp=np.cross(normal,along)
        if (not np.isfinite([self.length_mm,self.width_mm,self.edge_margin_mm]).all()
            or min(self.length_mm,self.width_mm)<=0 or self.edge_margin_mm<0
            or 2*self.edge_margin_mm>=min(self.length_mm,self.width_mm)):
            raise ValueError('Invalid measured pad dimensions or margin')

    def observe_book_point(self, book_point_base_mm, tcp_base_mm, tool_rotation):
        rotation=np.asarray(tool_rotation,float)
        if (rotation.shape!=(3,3) or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6)
            or not np.isclose(np.linalg.det(rotation),1.,atol=1e-6)):
            raise ValueError('Invalid measured tool rotation')
        local=rotation.T@(vector(book_point_base_mm,'book point')-vector(tcp_base_mm,'TCP'))-self.center_tcp_mm
        along=float(np.dot(local,self.along_tcp));across=float(np.dot(local,self.across_tcp))
        return dict(along_mm=along,across_mm=across,signed_gap_mm=float(np.dot(local,self.normal_tcp)),
            inside_pad=bool(abs(along)<=self.length_mm/2-self.edge_margin_mm
                            and abs(across)<=self.width_mm/2-self.edge_margin_mm),
            contact_confirmed=False)
