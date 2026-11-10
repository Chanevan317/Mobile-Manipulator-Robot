import numpy as np, manifold3d as m
from stlio import read_stl
def servo_solid(path=__import__('os').path.join(__import__('os').path.dirname(__file__), 'sts3215_03a_v1.stl')):
    t=read_stl(path)*1000
    T=np.stack([t[...,0]-12.5,-t[...,1],-t[...,2]],-1).reshape(-1,3)   # CAD joint frame: shaft = Z, horn +Z, body -X
    u,inv=np.unique(np.round(T,4),axis=0,return_inverse=True); F=inv.reshape(-1,3)
    F=F[(F[:,0]!=F[:,1])&(F[:,1]!=F[:,2])&(F[:,0]!=F[:,2])]
    return m.Manifold(m.Mesh(vert_properties=u.astype(np.float32),tri_verts=F.astype(np.uint32)))
