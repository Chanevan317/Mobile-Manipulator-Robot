import sys; sys.path.insert(0,'.')
from build import *
F=frames(); build_parts(F)
W=[(nm, xf(p['solid'],p['frame'])) for nm,p in arm.PARTS.items()]+[(b['name'],b['solid']) for b in bought(F)]
bbs=[w[1].bounding_box() for w in W]
FS=fasteners(); bad=0
for i,f in enumerate(FS):
    if f['kind']=='insert': 
        # insert model: shrink to check it sits fully in material: volume of insert outside its part
        # shell = the insert's own outside diameter + 1 mm of plastic, over its length (kit inserts, hw.SPEC od)
        host=dict(W)[f['part']]; sz=f['size']; ri=SPEC[sz]['ins']/2; L=f['length'] or SPEC[sz]['ins_d']-0.8
        shell=place(cyl(SPEC[sz]['od']/2+1.0,0.5,L)-cyl(ri+0.05,0,L+1),f['p'],f['d'])
        frac=(shell^host).volume()/shell.volume()
        if frac<0.97: print('insert wall thin',i,f['part'],f['note'],round(frac,3)); bad+=1
        continue
    s=fastener_solid(f); b=s.bounding_box()
    hits=[]
    for (nm,w),a in zip(W,bbs):
        if any(b[k]>a[k+3] or a[k]>b[k+3] for k in range(3)): continue
        v=(s^w).volume()
        if v>0.3: hits.append((nm,round(v,2)))
    if hits: print(i,f['part'],f['kind'],f['size'],f['length'],f['note'],hits); bad+=1
print('fasteners',len(FS),'flagged',bad)
