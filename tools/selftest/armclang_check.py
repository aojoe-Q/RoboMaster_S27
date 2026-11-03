r"""Compile the USB link sources with the flags Keil recorded in project_project.dep.

Needs a previous Keil build (for the .dep file) and ARMCLANG at E:\keil5. No link;
works while the Keil IDE is open.
"""
import os
import re
import subprocess
import sys

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

files = [r"..\APP\Arm_USB_task.c", r"..\APP\arm_usb_protocol.c", r"..\APP\ring_buffer.c",
         r"..\Components\Device\Src\MiniPC.c", r"..\USB_DEVICE\App\usbd_cdc_if.c",
         r"..\Core\Src\freertos.c", r"..\APP\Compat\app_can.c", r"..\APP\Arm_L_task.c",
         r"..\APP\Moter_task.c", r"..\APP\Arm_R_task.c", r"..\APP\referee_usart_task.c"]
os.makedirs(OUT, exist_ok=True)
failed = 0
for f in files:
    obj = os.path.join(OUT, os.path.basename(f)[:-2] + ".o")
    if os.path.exists(obj):
        os.remove(obj)
    r = subprocess.run([CC] + args + ["-Wall", "-Wextra", "-o", obj, f], cwd=MDK,
                       capture_output=True, text=True)
    own = [l for l in (r.stdout + r.stderr).splitlines()
           if re.search(r"(warning|error):", l)
           and not re.search(r"Drivers|Middlewares|SystemView|FreeRTOS", l)]
    print("%-38s rc=%d %s" % (f, r.returncode, "obj ok" if os.path.exists(obj) else "NO OBJ"))
    for l in own[:10]:
        print("    " + l)
    failed += r.returncode != 0
sys.exit(failed)
