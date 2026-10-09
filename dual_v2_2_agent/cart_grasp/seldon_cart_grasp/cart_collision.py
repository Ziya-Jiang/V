"""Nominal measured-cart collision primitives, placed using observed handle bars.

This replaces a solid cart rejection envelope with actual URDF openings. It is
not a certification of unseen payload or lower-body environment coverage.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .geometry import finite,Rejected
from .planning import ArmModel

class CartCollision:
    def __init__(self,urdf,observation):
        import fcl
        self.fcl=fcl;doc=ET.parse(urdf).getroot();axis=finite(observation['bar_axis'],(3,));normal=finite(observation['approach_normal'],(3,));axis/=np.linalg.norm(axis);normal-=np.dot(axis,normal)*axis;normal/=np.linalg.norm(normal);rot=np.column_stack([np.cross(axis,normal),axis,normal]);self.cart_pose=np.eye(4);self.cart_pose[:3,:3]=rot;self.cart_pose[:3,3]=np.mean(observation['handles'],axis=0)-normal*1.0575;self.objects=[]
        base=next(l for l in doc.findall('link') if l.get('name')=='cart_base')
        for c in base.findall('collision'):
            g=c.find('geometry');b=g.find('box');cylinder=g.find('cylinder')
            if b is not None:shape=fcl.Box(*np.fromstring(b.get('size'),sep=' '))
            elif cylinder is not None:shape=fcl.Cylinder(float(cylinder.get('radius')),float(cylinder.get('length')))
            else:raise Rejected('Unexpected cart collision geometry')
            t=self.cart_pose@ArmModel.origin(c);self.objects.append((c.get('name'),fcl.CollisionObject(shape,fcl.Transform(t[:3,:3],t[:3,3]))))
        self.urdf=str(urdf)
    def distance(self,obj):
        best=(1e6,None)
        for name,obstacle in self.objects:
            d=float(self.fcl.distance(obj,obstacle,self.fcl.DistanceRequest(),self.fcl.DistanceResult()))
            if d<best[0]:best=(d,name)
        return best
