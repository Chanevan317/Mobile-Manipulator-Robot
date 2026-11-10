import json, base64, numpy as np
from lib import mesh_arrays
def write_scene(items, path='view/scene.json', fov=30):
    parts=[]
    for it in items:
        V,F=(it['V'],it['F']) if 'V' in it else mesh_arrays(it['solid'])
        parts.append(dict(name=it['name'], c=list(it.get('color',(0.7,0.7,0.7))), op=it.get('op',1), metal=it.get('metal',0),
                          p=base64.b64encode(V[F].reshape(-1,3).astype(np.float32).tobytes()).decode()))
    json.dump(dict(parts=parts,fov=fov), open(path,'w'))
