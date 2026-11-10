import sys, os, math, json, numpy as np, xml.etree.ElementTree as ET
sys.path.insert(0,'.')
from stlio import read_stl
from lib import M, xf, mesh_arrays
import manifold3d as m3
def rpy(r,p,y):
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
def org(e):
    T=np.eye(4)
    if e is None: return T
    o=e.find('origin')
    if o is not None:
        T[:3,3]=[float(v) for v in o.get('xyz','0 0 0').split()]; T[:3,:3]=rpy(*[float(v) for v in o.get('rpy','0 0 0').split()])
    return T
def axrot(ax,a):
    ax=np.asarray(ax)/np.linalg.norm(ax); K=np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]])
    T=np.eye(4); T[:3,:3]=np.eye(3)+math.sin(a)*K+(1-math.cos(a))*K@K; return T
class Robot:
    def __init__(s,P,layout='prototype'):
        s.P=P; s.r=ET.parse(os.path.join(P,'urdf',f'mobile_manipulator_{layout}.urdf')).getroot()
        s.links={l.get('name'):l for l in s.r.findall('link')}; s.J=s.r.findall('joint')
        s.mats={m_.get('name'):[float(v) for v in m_.find('color').get('rgba').split()] for m_ in s.r.findall('material')}
        s.cache={}
    def fk(s,q):
        q=dict(q)
        for j in s.J:
            mi=j.find('mimic')
            if mi is not None: q[j.get('name')]=q.get(mi.get('joint'),0)*float(mi.get('multiplier','1'))+float(mi.get('offset','0'))
        W={'base_link':np.eye(4)}; pend=list(s.J)
        while pend:
            for j in list(pend):
                p=j.find('parent').get('link')
                if p in W:
                    T=W[p]@org(j); t=j.get('type'); a=q.get(j.get('name'),0.0)
                    ax=[float(v) for v in (j.find('axis').get('xyz') if j.find('axis') is not None else '1 0 0').split()]
                    if t in ('revolute','continuous'): T=T@axrot(ax,a)
                    elif t=='prismatic': T=T.copy(); T[:3,3]+=T[:3,:3]@(np.asarray(ax)*a)
                    W[j.find('child').get('link')]=T; pend.remove(j)
        return W
    def geom(s,g):
        mesh=g.find('mesh'); b=g.find('box'); c=g.find('cylinder'); sp=g.find('sphere')
        if mesh is not None:
            f=os.path.normpath(os.path.join(s.P,'urdf',mesh.get('filename')))
            if f not in s.cache:
                t=read_stl(f).reshape(-1,3); u,inv=np.unique(np.round(t,7),axis=0,return_inverse=True)
                s.cache[f]=M(m3.Mesh(vert_properties=u.astype(np.float32),tri_verts=inv.reshape(-1,3).astype(np.uint32)))
            return s.cache[f]
        if b is not None:
            x,y,z=[float(v) for v in b.get('size').split()]; return M.cube((x,y,z),True)
        if c is not None:
            return M.cylinder(float(c.get('length')),float(c.get('radius')),float(c.get('radius')),32,True)
        return M.sphere(float(sp.get('radius')),24)
    def solids(s,q,kind='visual'):
        W=s.fk(q); out=[]
        for nm,l in s.links.items():
            for v in l.findall(kind):
                g=s.geom(v.find('geometry')); T=W[nm]@org(v)
                col=(0.6,0.6,0.6)
                mt=v.find('material')
                if mt is not None: col=s.mats.get(mt.get('name'),[0.6]*4)[:3]
                out.append((nm,xf(g,T),col))
        return out
