"""Build the standalone integer model and separately preserved VCC kernels."""
from pathlib import Path
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
UP = ROOT / "upstream/vamp-vcc"
BUILD = ROOT / "build"


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]


def robot_description():
    tree = ET.parse(UP / "resources/panda/panda_spherized.urdf").getroot()
    links = [x.attrib['name'] for x in tree.findall('link')]
    joints, spheres = [], []
    for j in tree.findall('joint'):
        origin = j.find('origin')
        a = origin.attrib if origin is not None else {}
        rpy = list(map(float, a.get('rpy', '0 0 0').split()))
        xyz = list(map(float, a.get('xyz', '0 0 0').split()))
        limit = j.find('limit')
        index = int(j.attrib['name'][-1])-1 if j.attrib['type'] == 'revolute' else -1
        axis = j.find('axis')
        if index >= 0 and axis.attrib['xyz'] != '0 0 1':
            raise ValueError('v1 supports Panda z-axis revolute joints only')
        joints.append(dict(name=j.attrib['name'], parent=links.index(j.find('parent').attrib['link']),
                           child=links.index(j.find('child').attrib['link']), index=index, xyz=xyz,
                           rpy=rpy, rotation=rpy_matrix(rpy),
                           limit=[float(limit.attrib['lower']), float(limit.attrib['upper'])] if index >= 0 else None))
    for link in tree.findall('link'):
        for c in link.findall('collision'):
            sphere = c.find('geometry/sphere')
            if sphere is not None:
                origin = c.find('origin')
                xyz = list(map(float, origin.attrib.get('xyz', '0 0 0').split())) if origin is not None else [0., 0., 0.]
                spheres.append(dict(link=links.index(link.attrib['name']), xyz=xyz, radius=float(sphere.attrib['radius'])))
    srdf = ET.parse(UP / 'resources/panda/panda.srdf').getroot()
    disabled = {frozenset((x.attrib['link1'], x.attrib['link2'])) for x in srdf.findall('disable_collisions')}
    pairs = [(i, j) for i, a in enumerate(spheres) for j, b in enumerate(spheres[:i])
             if a['link'] != b['link'] and frozenset((links[a['link']], links[b['link']])) not in disabled]
    result = dict(links=links, joints=joints, spheres=spheres, self_pairs=pairs,
                  source_commit='96a0d556e9677bb91c755aed7d268fa2772035b9', finger_opening_m=0.065)
    return result


def generate():
    BUILD.mkdir(exist_ok=True)
    desc = robot_description()
    (BUILD/'robot_description.json').write_text(json.dumps(desc, indent=2)+'\n', encoding='utf-8')
    def ar(values):
        return '{'+','.join(str(v) for v in values)+'}'
    h = ['// Generated from pinned Panda URDF and SRDF. Do not edit.', '#pragma once',
         f'constexpr int NJ={len(desc["joints"])}, NL={len(desc["links"])}, NS={len(desc["spheres"])}, NP={len(desc["self_pairs"])};',
         'struct JointDesc {int parent,child,index; long long r[9],t[3]; double rf[9],tf[3];};',
         'struct SphereDesc {int link; long long p[3]; int radius; double pf[3],radiusf;};',
         'const JointDesc JOINTS[NJ]={']
    for j in desc['joints']:
        r = sum(j['rotation'], [])
        h.append('{'+','.join([str(j[k]) for k in ['parent','child','index']]+[ar([round(v*(1<<30)) for v in r]),ar([round(v*(1<<24)) for v in j['xyz']]),ar(r),ar(j['xyz'])])+'},')
    h += ['};','const SphereDesc SPHERES[NS]={']
    for s in desc['spheres']:
        h.append('{'+','.join([str(s['link']),ar([round(v*(1<<24)) for v in s['xyz']]),str(math.ceil(s['radius']*8192)),ar(s['xyz']),str(s['radius'])])+'},')
    h += ['};', 'const int SELF_PAIRS[NP][2]={'+','.join(ar(p) for p in desc['self_pairs'])+'};']
    limits = [next(j['limit'] for j in desc['joints'] if j['index'] == i) for i in range(7)]
    h += ['const int ANGLE_LO[7]='+ar([math.ceil(l[0]*(1<<24)) for l in limits])+';',
          'const int ANGLE_HI[7]='+ar([math.floor(l[1]*(1<<24)) for l in limits])+';',
          'const int ATAN_Q28[30]='+ar([round(math.atan(2.**-i)*(1<<28)) for i in range(30)])+';']
    (BUILD/'panda_generated.hh').write_text('\n'.join(h)+'\n', encoding='utf-8')
    source = (UP/'src/impl/vamp/robots/panda.hh').read_text(encoding='utf-8')
    begin = source.index('std::array<FloatVector<rake, 1>, 22> v;')
    end = source.index('\n        }', begin)
    body = source[begin:end].replace('std::array<FloatVector<rake, 1>, 22> v;', 'double v[22]{};').replace('std::array<FloatVector<rake, 1>, 236> y;', 'double y[236]{};')
    for axis, off in [('x',0),('y',1),('z',2),('r',3)]:
        body = body.replace(f'out.{axis}[i]', f'out[i*4+{off}]')
    (BUILD/'upstream_fk_scalar.hh').write_text('// Scalar extraction of pinned sphere_fk equations; double arithmetic.\ninline void upstream_fk_scalar(const double*x,double*out){\n'+body+'\n}\n', encoding='utf-8')
    return desc


def baseline_copy(unaligned_compat=False):
    dest = BUILD/'vcc_windows_include'
    src = UP/'src/impl'
    patches = []
    for path in src.rglob('*.hh'):
        text = path.read_text(encoding='utf-8')
        modified = text.replace('posix_memalign(', 'soc_compat_posix_memalign(').replace('aligned_alloc(', 'soc_compat_aligned_alloc(')
        modified = re.sub(r'(?<![\w:])(?:std::)?free\b', 'soc_compat_free', modified)
        if os.name == 'nt' and unaligned_compat:
            # GCC 8 MinGW's ABI supports stack alignment up to 16 bytes. Keep AVX2 math,
            # but use unaligned memory intrinsics and 16-byte C++ object alignment.
            modified = re.sub(r'alignas\(32\)', 'alignas(16)', modified)
            modified = modified.replace('Alignment = 32', 'Alignment = 16')
            modified = modified.replace('FloatVectorWidth * sizeof(FloatT)', '16') if path.name == 'vector.hh' else modified
            for op in ['load_ps','store_ps','load_si256','store_si256']:
                modified = modified.replace('_mm256_'+op, '_mm256_'+op.replace('_','u_',1))
        # Preserve original source; Windows alignment is changed only in this copy.
        out = dest/path.relative_to(src)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(modified, encoding='utf-8')
        if modified != text:
            patches.append(str(path.relative_to(src)).replace('\\','/'))
    return patches


def main():
    generate()
    compiler = shutil.which('g++')
    if not compiler:
        raise RuntimeError('g++ not found; install a C++17 compiler and rerun')
    flags = ['-std=c++17','-O2','-fno-fast-math','-ffp-contract=off','-shared','-static-libgcc','-static-libstdc++','-I',str(BUILD)]
    source_hash=hashlib.sha256((ROOT/'soc_model/core.cpp').read_bytes()+(BUILD/'panda_generated.hh').read_bytes()+(BUILD/'upstream_fk_scalar.hh').read_bytes()).hexdigest()
    dll = BUILD/(f'soc_model_{source_hash[:12]}.dll' if os.name == 'nt' else f'libsoc_model_{source_hash[:12]}.so')
    cmd = [compiler,*flags,str(ROOT/'soc_model/core.cpp'),'-o',str(dll)]
    if not dll.exists():subprocess.run(cmd, check=True)
    float_source=ROOT/'soc_model/float_morton.cpp'
    float_dll=BUILD/('float_morton.dll' if os.name=='nt' else 'libfloat_morton.so')
    if not float_dll.exists() or float_source.stat().st_mtime>float_dll.stat().st_mtime:
        subprocess.run([compiler,*flags,str(float_source),'-o',str(float_dll)],check=True)
    # LLVM in the pinned ziglang distribution correctly supports over-aligned AVX
    # stack objects on Windows; the older MinGW compiler remains sufficient for core.cpp.
    import importlib.util
    has_zig = importlib.util.find_spec('ziglang') is not None
    patches = baseline_copy(unaligned_compat=not has_zig)
    baseline_dll = BUILD/('vcc_baseline.dll' if os.name == 'nt' else 'libvcc_baseline.so')
    baseline_flags=[x for x in flags if x not in ('-static-libgcc','-static-libstdc++')]
    prefix=[sys.executable,'-m','ziglang','c++'] if has_zig else [compiler]
    baseline_cmd = [*prefix,*baseline_flags,'-DEIGEN_DONT_VECTORIZE','-mavx2','-mfma','-mbmi2','-Wno-ignored-attributes','-I',str(BUILD/'vcc_windows_include'),'-I',str(ROOT/'third_party/eigen'),'-I',str(ROOT/'third_party/nigh/src'),'-I',str(ROOT/'third_party/pdqsort'),str(ROOT/'soc_model/vcc_baseline.cpp'),'-o',str(baseline_dll)]
    environment=os.environ.copy()
    for name,folder in [('ZIG_GLOBAL_CACHE_DIR','zig_global_cache'),('ZIG_LOCAL_CACHE_DIR','zig_local_cache'),('TEMP','tmp'),('TMP','tmp')]:
        path=BUILD/folder;path.mkdir(exist_ok=True);environment[name]=str(path)
    baseline = subprocess.run(baseline_cmd, capture_output=True, text=True,encoding='utf-8',errors='replace',env=environment)
    (BUILD/'baseline_build.log').write_text(baseline.stdout+baseline.stderr, encoding='utf-8')
    report = dict(compiler=subprocess.check_output([compiler,'--version'],text=True).splitlines()[0],
                  core_command=cmd, core_binary=dll.relative_to(ROOT).as_posix(),core_source_sha256=source_hash,core_sha256=hashlib.sha256(dll.read_bytes()).hexdigest(),
                  baseline_command=baseline_cmd, baseline_built=baseline.returncode==0,
                  windows_compatibility_files=patches, baseline_log='build/baseline_build.log')
    (BUILD/'build_report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))
    if baseline.returncode:
        print(baseline.stderr[-5000:])
        raise RuntimeError('Original VCC kernel baseline did not compile')


if __name__ == '__main__':
    main()
