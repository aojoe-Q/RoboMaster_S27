"""Test the actual home-settling block with asynchronous motor feedback."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / "APP/Arm_L_task.c").read_text(encoding="utf-8-sig")
header = (ROOT / "APP/Arm_L_task.h").read_text(encoding="utf-8-sig")
start = source.index("        if (ramps_done) {")
end, depth = source.index("{", start) + 1, 1
while depth:
    depth += (source[end] == "{") - (source[end] == "}")
    end += 1
block = source[start:end]
parts = [r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
#define portTICK_PERIOD_MS 1U
static uint32_t now, settle_start, last_update_counts[7];
static uint8_t all_settled, feedback_fresh = 1, ramps_done = 1;
static float J1_J2_J3_J4_J5_J6_J7_init_flag = 1;
static struct { float position, velocity; uint32_t update_count, update_tick; } fb_snapshot[7];
static struct { uint8_t in_window, has_new_feedback; } arm_home_diag;
static uint32_t xTaskGetTickCount(void) { return now; }
''']
parts += re.findall(r"^#define (?:J[1-6]|grip)_zero_position[^\n]*", header, re.M)
parts += ["static void step(void) {", block, "}", r'''
int main(int argc, char **argv) {
    CHECK(argc == 2);
    const char *scenario = argv[1];
    const uint32_t periods[7] = {4, 8, 12, 4, 8, 12, 4};
    for (now = 0; now <= 300; now += 5) {
        feedback_fresh = 1;
        for (unsigned i = 0; i < 7; ++i) {
            uint32_t period = !strcmp(scenario, "normal") ? 4 : periods[i];
            fb_snapshot[i].update_count = now / period + 1;
            fb_snapshot[i].update_tick = now / period * period;
            fb_snapshot[i].position = 0;
            fb_snapshot[i].velocity = 0;
        }
        if (!strcmp(scenario, "stale") || !strcmp(scenario, "duplicate")) {
            fb_snapshot[6].update_count = 1;
            fb_snapshot[6].update_tick = 0;
            if (!strcmp(scenario, "stale") && now > 100) feedback_fresh = 0;
        }
        if (!strcmp(scenario, "position_reset") && now == 60) fb_snapshot[2].position = 0.06f;
        if (!strcmp(scenario, "velocity_reset") && now == 60) fb_snapshot[3].velocity = 0.11f;
        if (!strcmp(scenario, "out_of_position")) fb_snapshot[2].position = 0.06f;
        if (!strcmp(scenario, "moving")) fb_snapshot[3].velocity = 0.11f;
        if (!strcmp(scenario, "ramping")) ramps_done = 0;
        step();
        if (!J1_J2_J3_J4_J5_J6_J7_init_flag) {
            CHECK(now >= 100);
            CHECK(strcmp(scenario, "stale") && strcmp(scenario, "duplicate"));
            CHECK(strcmp(scenario, "out_of_position") && strcmp(scenario, "moving") && strcmp(scenario, "ramping"));
            if (!strcmp(scenario, "position_reset") || !strcmp(scenario, "velocity_reset")) CHECK(now >= 165);
            printf("%s: OK, settled at %u ms\n", scenario, now);
            return 0;
        }
    }
    CHECK(!strcmp(scenario, "stale") || !strcmp(scenario, "duplicate") ||
          !strcmp(scenario, "out_of_position") || !strcmp(scenario, "moving") || !strcmp(scenario, "ramping"));
    printf("%s: OK, did not settle\n", scenario);
    return 0;
}
''']
with tempfile.TemporaryDirectory(prefix="home_settle_") as folder:
    test_source = Path(folder) / "home_settle.c"
    executable = Path(folder) / "home_settle.exe"
    test_source.write_text("\n".join(parts), encoding="utf-8")
    compiler = shutil.which("gcc") or r"E:\mingw64\bin\gcc.exe"
    subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Wno-unused-variable",
                    str(test_source), "-lm", "-o", str(executable)], check=True)
    for scenario in ("normal", "asynchronous", "position_reset", "velocity_reset", "stale",
                     "duplicate", "out_of_position", "moving", "ramping"):
        subprocess.run([str(executable), scenario], check=True, timeout=10)
print("HOME SETTLE SELFTEST OK")
