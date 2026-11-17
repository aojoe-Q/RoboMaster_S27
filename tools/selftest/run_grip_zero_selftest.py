"""Exercise the real gripper homing function with simulated feedback; no hardware."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / "APP/Arm_L_task.c").read_text(encoding="utf-8-sig")
header = (ROOT / "APP/Arm_L_task.h").read_text(encoding="utf-8-sig")
match = re.search(r"static void Arm_Grip_Stall_Zero\(void\)\s*\{", source)
assert match
end, depth = match.end(), 1
while depth:
    depth += (source[end] == "{") - (source[end] == "}")
    end += 1

parts = [r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
''']
for name in ("ArmGripZeroResult", "ArmGripZeroDiagnostics"):
    parts.append(re.search(r"typedef\s+(?:enum|struct)\s*\{[^}]*\}\s*" + name + r"\s*;", header).group())
parts += re.findall(r"^#define GRIP_ZERO_.*$", source, re.M)
parts.append(r'''
#define portTICK_PERIOD_MS 1U
#define HAL_OK 0
static const char *scenario;
static int hfdcan1;
static unsigned sent, samples;
static uint32_t now = 1000, sent_at;
typedef struct { float position; uint32_t update_tick, update_count; } joint_motor_measure_t;
static joint_motor_measure_t joint_L_motor[7];
static struct { struct { float Kp, Kd, send_set, torque; } Grip; } ArmData;
static struct { uint8_t startup_failed; } arm_output_diag;
volatile ArmGripZeroDiagnostics arm_grip_zero_diag;
static int is_case(const char *name) { return !strcmp(scenario, name); }
static uint32_t xTaskGetTickCount(void) { return now; }
static void Arm_PrepareAndPublish(void) {}
static void Arm_PublishCurrent(void) {}
static joint_motor_measure_t Arm_ReadGripFeedback(void) { return joint_L_motor[6]; }
static void osDelay(uint32_t ms) {
    now += ms;
    joint_motor_measure_t *f = &joint_L_motor[6];
    f->update_tick = is_case("stale") ? now - 41 : now;
    if (!is_case("duplicate")) f->update_count++;
    if (sent) {
        f->position = is_case("readback") ? 0.5f : 0.0f;
        return;
    }
    samples++;
    if (is_case("short_move")) f->position = 0.505f;
    else if (is_case("move_settle")) f->position = 0.5f + 0.03f * (samples < 3 ? samples : 3);
    else if (is_case("moving")) f->position += 0.03f;
    else if (is_case("creeping")) f->position += 0.0004f;
    /* Otherwise remain at the initial stop, with fresh feedback. */
}
static uint8_t Arm_Output_AllEnabled(void) {
    if (is_case("enable_lost") && now >= 1040) return 0;
    if (is_case("verify_lost") && sent) return 0;
    return 1;
}
static int canx_send_data(int *bus, uint16_t id, uint8_t *bytes, uint8_t n) {
    (void)bus;
    CHECK(id == 7 && n == 8 && bytes[7] == 0xfe);
    CHECK(ArmData.Grip.torque == 0);
    CHECK(now - 1000 >= GRIP_ZERO_STILL_CNT * GRIP_ZERO_STEP_MS);
    if (is_case("send_failed")) return 1;
    sent++;
    sent_at = now;
    return HAL_OK;
}
''')
parts.append(source[match.start():end])
parts.append(r'''
int main(int argc, char **argv) {
    CHECK(argc == 2);
    scenario = argv[1];
    joint_L_motor[6].position = 0.5f;
    joint_L_motor[6].update_tick = now;
    Arm_Grip_Stall_Zero();
    ArmGripZeroResult expected = GRIP_ZERO_OK;
    if (is_case("stale")) expected = GRIP_ZERO_FEEDBACK_STALE;
    if (is_case("enable_lost")) expected = GRIP_ZERO_ENABLE_LOST;
    if (is_case("moving") || is_case("creeping") || is_case("duplicate")) expected = GRIP_ZERO_STALL_TIMEOUT;
    if (is_case("send_failed")) expected = GRIP_ZERO_SEND_FAILED;
    if (is_case("readback")) expected = GRIP_ZERO_READBACK_TIMEOUT;
    if (is_case("verify_lost")) expected = GRIP_ZERO_VERIFY_ENABLE_LOST;
    CHECK(arm_grip_zero_diag.result == expected);
    CHECK(arm_output_diag.startup_failed == (expected != GRIP_ZERO_OK));
    CHECK(ArmData.Grip.torque == 0);
    CHECK(sent == (expected == GRIP_ZERO_OK || expected == GRIP_ZERO_READBACK_TIMEOUT || expected == GRIP_ZERO_VERIFY_ENABLE_LOST));
    if (is_case("stationary")) {
        CHECK(arm_grip_zero_diag.max_travel == 0);
        CHECK(sent_at == 1000 + GRIP_ZERO_STILL_CNT * GRIP_ZERO_STEP_MS + 8);
    }
    if (is_case("short_move")) CHECK(arm_grip_zero_diag.max_travel < 0.02f);
    if (is_case("move_settle")) CHECK(sent_at >= 1000 + (3 + GRIP_ZERO_STILL_CNT) * GRIP_ZERO_STEP_MS + 8);
    printf("%s: OK (result=%d)\n", scenario, (int)expected);
    return 0;
}
''')

with tempfile.TemporaryDirectory(prefix="grip_zero_selftest_") as folder:
    test_source = Path(folder) / "grip_zero.c"
    executable = Path(folder) / "grip_zero.exe"
    test_source.write_text("\n".join(parts), encoding="utf-8")
    compiler = shutil.which("gcc") or r"E:\mingw64\bin\gcc.exe"
    subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                    str(test_source), "-lm", "-o", str(executable)], check=True)
    for scenario in ("stationary", "short_move", "move_settle", "moving", "creeping",
                     "duplicate", "stale", "enable_lost", "send_failed", "readback", "verify_lost"):
        subprocess.run([str(executable), scenario], check=True, timeout=10)
print("GRIP ZERO SELFTEST OK")
