r"""Compile the USB link sources with the flags Keil recorded in project_project.dep.

Needs a previous Keil build (for the .dep file) and ARMCLANG at E:\keil5. No link;
works while the Keil IDE is open.
"""
import os
import re
import subprocess
import sys
import json
import hashlib
from datetime import datetime

MDK = r"C:\Users\lenovo\Desktop\S27_TEST_2\MDK-ARM"
CC = r"E:\keil5\ARM\ARMCLANG\bin\armclang.exe"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "build", "armclang_obj")

dep = open(os.path.join(MDK, "COD_H7_Template", "project_project.dep"), encoding="latin-1", newline="").read()
line = [l for l in dep.split("\n") if l.startswith("F (..\\APP\\Arm_L_task.c)")][0].rstrip("\r")
flags = line[line.index(")(-xc") + 2: line.rindex(")")]
flags = (flags.replace("ARM_MATH_CHECK-o", "ARM_MATH_CHECK -o")
              .replace("-c-fno-rtti", "-c -fno-rtti")
              .replace("-fshort-wchar-gdwarf-4", "-fshort-wchar -gdwarf-4"))
args, skip = [], False
for a in flags.split():
    if skip:
        skip = False
        continue
    if a == "-o":
        skip = True
        continue
    if a == "-MD":
        continue
    args.append(a)

# Extract C files from .dep to ensure list matches actual build
dep_files = []
for line in dep.split("\n"):
    if line.startswith("F (.."):
        m = re.match(r"F \(([^)]+\.c)\)", line)
        if m:
            dep_files.append(m.group(1))

# Filter to project sources (APP, Components, Modules, BSP) and exclude archived files
excluded = [r"..\APP\Arm_R_task.c"]  # Archived files
files = [f for f in dep_files
         if any(x in f for x in [r"\APP\\", r"\Components\\", r"\Modules\\", r"\BSP\\"])
         and f not in excluded]

if not files:
    print("ERROR: No project C files found in .dep file")
    sys.exit(1)

print(f"Found {len(files)} project C files in .dep")
print(f"Excluded: {', '.join(excluded)}")

os.makedirs(OUT, exist_ok=True)

# Prepare build record
build_record = {
    "timestamp": datetime.now().isoformat(),
    "dep_file": os.path.join(MDK, "COD_H7_Template", "project_project.dep"),
    "compiler": CC,
    "flags": " ".join(args),
    "excluded_files": excluded,
    "files": [],
    "summary": {}
}

failed = 0
warnings_count = 0
for f in files:
    obj = os.path.join(OUT, os.path.basename(f)[:-2] + ".o")
    if os.path.exists(obj):
        os.remove(obj)

    # Compute source file hash
    src_path = os.path.join(MDK, f)
    src_hash = ""
    if os.path.exists(src_path):
        with open(src_path, "rb") as sf:
            src_hash = hashlib.sha256(sf.read()).hexdigest()[:16]

    r = subprocess.run([CC] + args + ["-Wall", "-Wextra", "-o", obj, f], cwd=MDK,
                       capture_output=True, text=True)
    if r.returncode: print(r.stdout + r.stderr)
    own = [l for l in (r.stdout + r.stderr).splitlines()
           if re.search(r"(warning|error):", l)
           and not re.search(r"Drivers|Middlewares|SystemView|FreeRTOS", l)]

    file_record = {
        "path": f,
        "hash": src_hash,
        "returncode": r.returncode,
        "obj_created": os.path.exists(obj),
        "warnings": own
    }
    build_record["files"].append(file_record)
    warnings_count += len(own)

    print("%-38s rc=%d %s" % (f, r.returncode, "obj ok" if os.path.exists(obj) else "NO OBJ"))
    for l in own[:10]:
        print("    " + l)
    failed += r.returncode != 0

build_record["summary"] = {
    "total_files": len(files),
    "failed": failed,
    "warnings": warnings_count
}

# Save build record
record_path = os.path.join(OUT, "build_record.json")
with open(record_path, "w") as f:
    json.dump(build_record, f, indent=2)
print(f"\nBuild record saved to: {record_path}")
print(f"Summary: {len(files)} files, {failed} failures, {warnings_count} warnings")

sys.exit(failed)
