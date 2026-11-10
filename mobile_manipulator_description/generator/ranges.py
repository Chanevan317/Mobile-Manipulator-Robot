import sys; sys.path.insert(0,'.')
from build import *
from check import clashes
F0=frames(); build_parts(F0)
def key(nm):
    if nm.startswith('base'): return 'arm'
    if nm.startswith('upper'): return 'upper'
    if nm.startswith('forearm'): return 'fore'
    if nm.startswith(('wrist','pitch')): return 'wrist'
    if nm.startswith(('tilt','camera')): return 'tilt'
    return 'roll'
OFF={nm:(key(nm), np.linalg.inv(F0[key(nm)])@p['frame']) for nm,p in arm.PARTS.items()}
B0=bought(F0)
def bkey(nm):
    if 'shoulder' in nm or 'J1' in nm: return 'arm'
    if 'elbow' in nm or 'J2' in nm: return 'upper'
    if 'wrist_pan' in nm or 'J3' in nm: return 'fore'
    if 'pitch' in nm: return 'wrist'
    if 'roll_' in nm or 'wrist_cam' in nm: return 'tilt'
    return 'roll'
BOFF={b['name']:(bkey(b['name']), np.linalg.inv(F0[bkey(b['name'])]), b['solid']) for b in B0}
def world(F):
    W=[(nm, xf(arm.PARTS[nm]['solid'], F[k]@o)) for nm,(k,o) in OFF.items()]
    W+=[(nm, xf(s, F[k]@inv)) for nm,(k,inv,s) in BOFF.items()]
    return W
def scan(name, fn, angles):
    bad=[]
    for a in angles:
        c=clashes(world(fn(a)),0.5)
        if c: bad.append((a,c[:3]))
    return bad
if __name__=='__main__':
    which=sys.argv[1]
    A=range(-180,181,10)
    fn={'j1':lambda a:frames((a,0,0)),'j2':lambda a:frames((0,a,0)),'j3':lambda a:frames((0,0,a)),
        'pitch':lambda a:frames((0,0,0),a,0),'roll':lambda a:frames((0,0,0),0,a)}[which]
    if which=='pitch': A=range(-60,121,5)
    for a,c in scan(which,fn,A): print(a,c)
