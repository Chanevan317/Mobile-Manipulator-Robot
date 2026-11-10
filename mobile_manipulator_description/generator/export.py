"""Writers: binary STL, GLB (glTF 2.0), 3MF, OBJ. Meshes are (V float64 Nx3 in mm, F int Mx3)."""
import json, struct, zipfile, io
import numpy as np
from stlio import write_stl


def write_glb(path, items, scale=0.001):
    """items: list of dict(name, V, F, color=(r,g,b), metal=0..1). Output in metres (glTF convention), Y-up:
    our Z-up mm coordinates are converted with (x, z, -y)."""
    bin_ = bytearray(); bviews = []; accs = []; meshes = []; nodes = []; mats = []
    for it in items:
        V = np.asarray(it['V'], np.float64)
        F = np.asarray(it['F'], np.int64)
        tri = V[F].reshape(-1, 3)                                  # flat shading: unshared vertices
        n = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
        n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
        nrm = np.repeat(n, 3, axis=0)
        conv = lambda a: np.c_[a[:, 0], a[:, 2], -a[:, 1]]
        pos = (conv(tri) * scale).astype(np.float32); nrm = conv(nrm).astype(np.float32)
        for arr, typ in ((pos, 'VEC3'), (nrm, 'VEC3')):
            while len(bin_) % 4: bin_ += b'\0'
            off = len(bin_); bin_ += arr.tobytes()
            bviews.append(dict(buffer=0, byteOffset=off, byteLength=arr.nbytes, target=34962))
            a = dict(bufferView=len(bviews) - 1, componentType=5126, count=len(arr), type=typ)
            if arr is pos: a['min'] = pos.min(0).tolist(); a['max'] = pos.max(0).tolist()
            accs.append(a)
        c = list(it.get('color', (0.7, 0.7, 0.7)))
        mats.append(dict(name=it['name'], pbrMetallicRoughness=dict(baseColorFactor=c + [1.0], metallicFactor=it.get('metal', 0.0),
                                                                     roughnessFactor=0.55)))
        meshes.append(dict(name=it['name'], primitives=[dict(attributes=dict(POSITION=len(accs) - 2, NORMAL=len(accs) - 1),
                                                             material=len(mats) - 1)]))
        nodes.append(dict(name=it['name'], mesh=len(meshes) - 1))
    while len(bin_) % 4: bin_ += b'\0'
    gl = dict(asset=dict(version='2.0', generator='scara-arm cad'), scene=0,
              scenes=[dict(nodes=list(range(len(nodes))))], nodes=nodes, meshes=meshes, materials=mats,
              accessors=accs, bufferViews=bviews, buffers=[dict(byteLength=len(bin_))])
    js = json.dumps(gl).encode()
    while len(js) % 4: js += b' '
    with open(path, 'wb') as f:
        f.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(bin_)))
        f.write(struct.pack('<II', len(js), 0x4E4F534A)); f.write(js)
        f.write(struct.pack('<II', len(bin_), 0x004E4942)); f.write(bytes(bin_))


def write_3mf(path, items):
    """Multi-object 3MF (mm) with per-object colours and names."""
    res = ['<basematerials id="1">']
    for it in items:
        r, g, b = [int(max(0, min(255, round(v * 255)))) for v in it.get('color', (0.7, 0.7, 0.7))]
        res.append(f'<base name="{it["name"]}" displaycolor="#{r:02X}{g:02X}{b:02X}FF"/>')
    res.append('</basematerials>')
    build = []
    for i, it in enumerate(items):
        V = np.asarray(it['V']); F = np.asarray(it['F'])
        vs = ''.join(f'<vertex x="{x:.4f}" y="{y:.4f}" z="{z:.4f}"/>' for x, y, z in V)
        ts = ''.join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in F)
        res.append(f'<object id="{i + 2}" name="{it["name"]}" type="model" pid="1" pindex="{i}"><mesh><vertices>{vs}</vertices>'
                   f'<triangles>{ts}</triangles></mesh></object>')
        build.append(f'<item objectid="{i + 2}"/>')
    model = ('<?xml version="1.0" encoding="UTF-8"?><model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"><resources>' + ''.join(res) +
             '</resources><build>' + ''.join(build) + '</build></model>')
    ct = ('<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
          '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
          '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    with zipfile.ZipFile(path, 'w', zipfile.ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml', ct); z.writestr('_rels/.rels', rels); z.writestr('3D/3dmodel.model', model)


def write_obj(path, items):
    with open(path, 'w') as f, open(path[:-4] + '.mtl', 'w') as mt:
        f.write(f'mtllib {path.split("/")[-1][:-4]}.mtl\n'); base = 1
        for it in items:
            c = it.get('color', (0.7, 0.7, 0.7)); nm = it['name'].replace(' ', '_')
            mt.write(f'newmtl {nm}\nKd {c[0]:.3f} {c[1]:.3f} {c[2]:.3f}\n')
            f.write(f'o {nm}\nusemtl {nm}\n')
            for v in it['V']: f.write(f'v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n')
            for a, b, c_ in np.asarray(it['F']) + base: f.write(f'f {a} {b} {c_}\n')
            base += len(it['V'])


def write_step(path, items, author='scara-arm cad'):
    """AP214 STEP with one FACETED_BREP solid per item (exact triangle geometry, mm). Opens in Fusion 360,
    SolidWorks, FreeCAD, Onshape, Inventor as solid bodies."""
    import datetime
    L = []; n = [0]
    def e(s):
        n[0] += 1; L.append(f'#{n[0]}={s};'); return n[0]
    ctx_u = e("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))")
    ang = e("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))")
    sol = e("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())")
    unc = e(f"UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-04),#{ctx_u},'distance_accuracy_value','')")
    ctx = e(f"(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc}))"
            f"GLOBAL_UNIT_ASSIGNED_CONTEXT((#{ctx_u},#{ang},#{sol}))REPRESENTATION_CONTEXT('',''))")
    appc = e("APPLICATION_CONTEXT('automotive design')")
    e(f"APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{appc})")
    pctx = e(f"PRODUCT_CONTEXT('',#{appc},'mechanical')")
    pdc = e(f"PRODUCT_DEFINITION_CONTEXT('part definition',#{appc},'design')")
    o = e("CARTESIAN_POINT('',(0.,0.,0.))"); dz = e("DIRECTION('',(0.,0.,1.))"); dx = e("DIRECTION('',(1.,0.,0.))")
    ax = e(f"AXIS2_PLACEMENT_3D('',#{o},#{dz},#{dx})")
    for it in items:
        V = np.asarray(it['V']); F = np.asarray(it['F'])
        nm = it['name'].replace("'", '')
        base = n[0]
        for x, y, z in V:
            e(f"CARTESIAN_POINT('',({x:.5f},{y:.5f},{z:.5f}))")
        faces = []
        for a, b, c in F:
            pl = e(f"POLY_LOOP('',(#{base+1+a},#{base+1+b},#{base+1+c}))")
            fb = e(f"FACE_OUTER_BOUND('',#{pl},.T.)")
            faces.append(e(f"FACE('',(#{fb}))"))
        sh = e("CLOSED_SHELL('',(" + ','.join(f'#{f}' for f in faces) + "))")
        br = e(f"FACETED_BREP('{nm}',#{sh})")
        rep = e(f"FACETED_BREP_SHAPE_REPRESENTATION('{nm}',(#{ax},#{br}),#{ctx})")
        pr = e(f"PRODUCT('{nm}','{nm}','',(#{pctx}))")
        pdf = e(f"PRODUCT_DEFINITION_FORMATION('','',#{pr})")
        pd = e(f"PRODUCT_DEFINITION('design','',#{pdf},#{pdc})")
        pds = e(f"PRODUCT_DEFINITION_SHAPE('','',#{pd})")
        e(f"SHAPE_DEFINITION_REPRESENTATION(#{pds},#{rep})")
        c = it.get('color', (0.7, 0.7, 0.7))
        col = e(f"COLOUR_RGB('',{c[0]:.3f},{c[1]:.3f},{c[2]:.3f})")
        fsc = e(f"FILL_AREA_STYLE_COLOUR('',#{col})"); fas = e(f"FILL_AREA_STYLE('',(#{fsc}))")
        ssfa = e(f"SURFACE_STYLE_FILL_AREA(#{fas})"); ssu = e(f"SURFACE_SIDE_STYLE('',(#{ssfa}))")
        sus = e(f"SURFACE_STYLE_USAGE(.BOTH.,#{ssu})"); psa = e(f"PRESENTATION_STYLE_ASSIGNMENT((#{sus}))")
        e(f"STYLED_ITEM('',(#{psa}),#{br})")
    now = datetime.datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%S')
    head = ("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('faceted solids'),'2;1');\n"
            f"FILE_NAME('{path.split('/')[-1]}','{now}',('{author}'),(''),'','','');\n"
            "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n")
    with open(path, 'w') as f:
        f.write(head + '\n'.join(L) + '\nENDSEC;\nEND-ISO-10303-21;\n')
