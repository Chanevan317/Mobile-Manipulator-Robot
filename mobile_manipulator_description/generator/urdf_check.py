import sys, os, math, numpy as np, xml.etree.ElementTree as ET
sys.path.insert(0,'.')
from stlio import read_stl
P=sys.argv[1]
LAYOUTS=sys.argv[2:] or ['prototype']
for layout in LAYOUTS:
  for ros in ('','_ros'):
    U=os.path.join(P,'urdf',f'mobile_manipulator_{layout}{ros}.urdf'); print('==',os.path.basename(U))
    r=ET.parse(U).getroot()
    links={l.get('name'):l for l in r.findall('link')}; joints=r.findall('joint')
    child={}; errs=[]
    for j in joints:
        c=j.find('child').get('link'); p=j.find('parent').get('link')
        if c in child: errs.append('two parents '+c)
        child[c]=j
        if p not in links or c not in links: errs.append('missing link '+j.get('name'))
    roots=[l for l in links if l not in child]
    print('links',len(links),'joints',len(joints),'roots',roots)
    for nm,l in links.items():
        for g in l.iter('mesh'):
            fn=g.get('filename'); f=os.path.normpath(os.path.join(P,fn.split('mobile_manipulator_description/',1)[1]) if fn.startswith('package://') else os.path.join(P,'urdf',fn))
            if not os.path.exists(f): errs.append('missing mesh '+f)
        ine=l.find('inertial')
        if ine is not None:
            i=ine.find('inertia'); I=np.array([[float(i.get(a)) for a in r_] for r_ in (('ixx','ixy','ixz'),('ixy','iyy','iyz'),('ixz','iyz','izz'))])
            ev=np.linalg.eigvalsh(I); m=float(ine.find('mass').get('value'))
            if m<=0 or ev.min()<=0 or ev[2]>ev[0]+ev[1]+1e-12: errs.append(f'bad inertia {nm} {m} {ev}')
        elif l.find('visual') is not None or l.find('collision') is not None: errs.append('no inertial '+nm)
    names={j.get('name') for j in joints}
    for j in joints:
        mi=j.find('mimic')
        if mi is not None and mi.get('joint') not in names: errs.append('bad mimic')
    print('errors',errs[:20], len(errs))
    typ={}
    for j in joints: typ[j.get('type')]=typ.get(j.get('type'),0)+1
    print(typ)
