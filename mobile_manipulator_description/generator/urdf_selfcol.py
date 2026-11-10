import sys; sys.path.insert(0,'.')
from urdf_fk import *
from lib import union
R=Robot(sys.argv[1], sys.argv[3] if len(sys.argv)>3 else 'prototype'); q=json.loads(sys.argv[2])
par={j.find('child').get('link'):j.find('parent').get('link') for j in R.J}
# links merged by fixed joints count as one body
fixedc={j.find('child').get('link') for j in R.J if j.get('type')=='fixed'}
def body(l):
    while l in fixedc: l=par[l]
    return l
import collections
S=collections.defaultdict(list)
for nm,s,c in R.solids(q,'collision'): S[body(nm)].append(s)
B={k:union(v) for k,v in S.items()}
bpar={body(k):body(v) for k,v in par.items() if body(k)!=body(v)}
names=list(B); bad=[]
for i in range(len(names)):
    for j in range(i+1,len(names)):
        a,b=names[i],names[j]
        if bpar.get(a)==b or bpar.get(b)==a: continue
        if 'roller' in a and 'roller' in b: continue
        v=(B[a]^B[b]).volume()*1e9
        if v>1.0: bad.append((a,b,round(v,1)))
print(bad)
