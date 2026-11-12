from pathlib import Path
import json,subprocess
root=Path(__file__).resolve().parents[1]
cmd=json.loads((root/'build/build_report.json').read_text())['baseline_command']
cmd=[x for x in cmd if x!='-shared']
cmd[cmd.index(str(root/'soc_model/vcc_baseline.cpp'))]=str(root/'tools/baseline_smoke.cpp')
cmd[cmd.index('-o')+1]=str(root/'build/baseline_smoke.exe');cmd.insert(1,'-g')
subprocess.run(cmd,check=True)
result=subprocess.run([r'C:\mingw64\bin\gdb.exe','--batch','-ex','run','-ex','bt','-ex','x/5i $pc','--args',str(root/'build/baseline_smoke.exe')],capture_output=True,text=True,timeout=60)
(root/'build/baseline_gdb.log').write_text(result.stdout+result.stderr,encoding='utf-8')
print((result.stdout+result.stderr)[-9000:])
