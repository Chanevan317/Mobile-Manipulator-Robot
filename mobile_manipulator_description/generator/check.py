import sys; sys.path.insert(0,'.')
from build import *
def world(F):
    build_parts(F)
    W=[(nm, xf(p['solid'],p['frame'])) for nm,p in arm.PARTS.items()]
    W+= [(b['name'], b['solid']) for b in bought(F)]
    return W
def clashes(W, tol=1.0, skip=()):
    out=[]
    bbs=[w[1].bounding_box() for w in W]
    for i in range(len(W)):
        for j in range(i+1,len(W)):
            a,b=bbs[i],bbs[j]
            if any(a[k]>b[k+3] or b[k]>a[k+3] for k in range(3)): continue
            v=(W[i][1]^W[j][1]).volume()
            if v>tol: out.append((W[i][0],W[j][0],round(v,2)))
    return out
if __name__=='__main__':
    F=frames()
    W=world(F)
    for nm,s in W[:21]:
        n=len(s.decompose())
        if n!=1: print('components',nm,n)
    for c in clashes(W, 0.05): print(c)
