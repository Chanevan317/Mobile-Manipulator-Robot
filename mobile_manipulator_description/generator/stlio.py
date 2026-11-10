import numpy as np, struct
def read_stl(p):
    b=open(p,'rb').read()
    if b[:5]==b'solid' and b'facet' in b[:300]:
        v=[list(map(float,l.split()[1:4])) for l in b.decode().splitlines() if l.strip().startswith('vertex')]
        return np.array(v).reshape(-1,3,3)
    n=struct.unpack('<I',b[80:84])[0]
    a=np.frombuffer(b[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
    return a['v'].reshape(-1,3,3).astype(float)
def write_stl(p,V,F):
    t=V[F]; n=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]); l=np.linalg.norm(n,axis=1,keepdims=True); n=n/np.where(l==0,1,l)
    a=np.zeros(len(F),dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')])); a['n']=n; a['v']=t.reshape(-1,9)
    with open(p,'wb') as f: f.write(b'binary STL'.ljust(80,b' ')); f.write(struct.pack('<I',len(F))); f.write(a.tobytes())
