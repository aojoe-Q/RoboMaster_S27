"""Clean compile/link with recorded Keil flags; never flashes or opens the IDE.

Requires MDK-ARM/COD_H7_Template/project_project.dep and project.lnp from Keil.
All objects, linker outputs and logs go under tools/selftest/build/full_arm.
"""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import os
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
MDK = ROOT / "MDK-ARM"
OUT = ROOT / "tools/selftest/build/full_arm"
BIN = Path(os.environ.get("ARMCLANG_BIN", r"E:\keil5\ARM\ARMCLANG\bin"))
OUT.mkdir(parents=True, exist_ok=True)
dep = (MDK / "COD_H7_Template/project_project.dep").read_text(encoding="latin1")
entries = re.findall(r"^F \(([^)]+)\)\([^)]*\)\((.*?)\)(?=\r?\n)", dep, re.M | re.S)
sources = [(src, flags) for src, flags in entries if Path(src).suffix.lower() in (".c", ".s")]
link = (MDK / "COD_H7_Template/project.lnp").read_text()
objects = {}


def compile_one(entry):
    src, flags = entry
    flags = (flags.replace("ARM_MATH_CHECK-o", "ARM_MATH_CHECK -o")
             .replace("-c-fno-rtti", "-c -fno-rtti")
             .replace("-fshort-wchar-gdwarf-4", "-fshort-wchar -gdwarf-4"))
    match = re.search(r"-o\s+(\S+)", flags)
    if not match:
        raise RuntimeError("No output in recorded flags for " + src)
    old_obj = match[1]
    obj = OUT / Path(old_obj.replace("\\", "/")).name
    objects[old_obj.replace("/", "\\").lower()] = obj
    flags = flags[:match.start()] + flags[match.end():]
    flags = re.sub(r"\s-MD\b", "", flags)
    response = OUT / (obj.stem + ".rsp")
    response.write_text(flags + '\n-o "' + obj.as_posix() + '" "' + src.replace("\\", "/") + '"\n')
    result = subprocess.run([str(BIN / "armclang.exe"), "@" + str(response)],
                            cwd=MDK, capture_output=True)
    log = result.stdout + result.stderr
    (OUT / (obj.stem + ".log")).write_bytes(log)
    if result.returncode:
        print(src, log.decode(errors="replace"))
    return result.returncode, log.count(b"warning:")


with ThreadPoolExecutor(max_workers=4) as pool:
    results = list(pool.map(compile_one, sources))
errors = sum(code != 0 for code, _ in results)
print(f"Compiled {len(sources)} translation units: {errors} failed, "
      f"{sum(w for _, w in results)} compiler warnings")
if errors:
    sys.exit(1)

def replace_obj(match):
    name = match[1].lower()
    if name not in objects:
        raise RuntimeError("Missing fresh object: " + name)
    return '"' + objects[name].as_posix() + '"'

link = re.sub(r'"([^"\n]+\.o)"', replace_obj, link)
link = re.sub(r'--list\s+"[^"]+"', '--list "' + OUT.as_posix() + '/project.map"', link)
link = re.sub(r'-o\s+\S+\.axf', '-o "' + OUT.as_posix() + '/project.axf"', link)
(OUT / "project.lnp").write_text(link)
result = subprocess.run([str(BIN / "armlink.exe"), "--via", str(OUT / "project.lnp")],
                        cwd=MDK, capture_output=True)
(OUT / "link.log").write_bytes(result.stdout + result.stderr)
print((result.stdout + result.stderr).decode(errors="replace"))
if result.returncode == 0:
    subprocess.run([str(BIN / "fromelf.exe"), "--i32combined", "--output",
                    str(OUT / "project.hex"), str(OUT / "project.axf")], check=True)
    print("FULL ARM BUILD OK:", OUT / "project.axf")
sys.exit(result.returncode)
