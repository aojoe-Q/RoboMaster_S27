#!/usr/bin/env python3
"""快速验证 P0/P1/P2 修复的脚本"""

import os
import sys
import re

def check_file_exists(path, description):
    """检查文件是否存在"""
    if os.path.exists(path):
        print(f"✓ {description}")
        return True
    else:
        print(f"✗ {description} - 文件不存在: {path}")
        return False

def check_file_content(path, pattern, description):
    """检查文件内容是否包含特定模式"""
    try:
        with open(path, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
            if re.search(pattern, content):
                print(f"✓ {description}")
                return True
            else:
                print(f"✗ {description} - 未找到匹配模式")
                return False
    except Exception as e:
        print(f"✗ {description} - 读取失败: {e}")
        return False

def main():
    print("=== P0/P1/P2 修复快速验证 ===\n")

    results = []

    # 检查文档
    print("1. 文档检查")
    results.append(check_file_exists("docs/P0_P1_FIXES.md", "P0/P1 修复文档"))
    results.append(check_file_exists("docs/P2_ARMCLANG_CHECK_FIX.md", "P2 修复文档"))
    results.append(check_file_exists("docs/P0_P1_P2_SUMMARY.md", "修复总结文档"))
    results.append(check_file_exists("docs/VERIFICATION_CHECKLIST.md", "验证清单"))
    print()

    # 检查 P0-1: UART DMA 修复
    print("2. P0-1: UART DMA 修复")
    results.append(check_file_content(
        "BSP/Src/bsp_uart.c",
        r"huart->RxXferSize\s*=\s*DataLength\s*;",
        "RxXferSize = DataLength (去掉 *2)"
    ))
    results.append(check_file_content(
        "BSP/Src/bsp_uart.c",
        r"__HAL_DMA_SET_COUNTER\(huart->hdmarx\s*,\s*SBUS_RX_BUF_NUM\s*\)",
        "SBUS DMA 重装为 SBUS_RX_BUF_NUM"
    ))
    results.append(check_file_content(
        "BSP/Src/bsp_uart.c",
        r"__HAL_DMA_SET_COUNTER\(huart->hdmarx\s*,\s*IMAGE_TRANS_RX_LENGTH\s*\)",
        "图传 DMA 重装为 IMAGE_TRANS_RX_LENGTH"
    ))
    print()

    # 检查 P0-2: 图传解析边界检查
    print("3. P0-2: 图传解析边界检查")
    results.append(check_file_content(
        "Components/Device/Inc/Image_Transmission.h",
        r"void\s+Image_Transmission_Info_Update\s*\(\s*uint8_t\s*\*\s*Buff\s*,\s*uint16_t\s+Size\s*\)",
        "函数签名包含 Size 参数"
    ))
    results.append(check_file_content(
        "Components/Device/Src/Image_Transmission.c",
        r"if\s*\(\s*Size\s*<\s*FrameHeader_Length\s*\)",
        "最小长度检查"
    ))
    results.append(check_file_content(
        "Components/Device/Src/Image_Transmission.c",
        r"if\s*\(\s*total_frame_len\s*>\s*Size",
        "总帧长度边界检查"
    ))
    results.append(check_file_content(
        "BSP/Src/bsp_uart.c",
        r"Image_Transmission_Info_Update\s*\(\s*Image_Trans_MultiRx_Buff\[\d+\]\s*,\s*Size\s*\)",
        "调用点传入 Size"
    ))
    print()

    # 检查 P1: 遥控超时竞态修复
    print("4. P1: 遥控超时竞态修复")
    results.append(check_file_content(
        "Components/Device/Src/Remote_Control.c",
        r"#include\s+[\"<]FreeRTOS\.h[\">]",
        "包含 FreeRTOS.h"
    ))
    results.append(check_file_content(
        "Components/Device/Src/Remote_Control.c",
        r"taskENTER_CRITICAL\s*\(\s*\)",
        "使用 taskENTER_CRITICAL"
    ))
    results.append(check_file_content(
        "Components/Device/Src/Remote_Control.c",
        r"taskEXIT_CRITICAL\s*\(\s*\)",
        "使用 taskEXIT_CRITICAL"
    ))
    print()

    # 检查 P2: 编译检查脚本
    print("5. P2: 编译检查脚本")
    results.append(check_file_content(
        "tools/selftest/armclang_check.py",
        r"import\s+json",
        "导入 json 模块"
    ))
    results.append(check_file_content(
        "tools/selftest/armclang_check.py",
        r"import\s+hashlib",
        "导入 hashlib 模块"
    ))
    results.append(check_file_content(
        "tools/selftest/armclang_check.py",
        r"from\s+datetime\s+import\s+datetime",
        "导入 datetime"
    ))
    results.append(check_file_content(
        "tools/selftest/armclang_check.py",
        r"excluded\s*=\s*\[.*Arm_R_task\.c.*\]",
        "排除 Arm_R_task.c"
    ))
    results.append(check_file_content(
        "tools/selftest/armclang_check.py",
        r"build_record\s*=\s*\{",
        "创建构建记录"
    ))
    results.append(check_file_content(
        "tools/selftest/armclang_check.py",
        r"hashlib\.sha256",
        "计算文件哈希"
    ))
    print()

    # 统计结果
    total = len(results)
    passed = sum(results)
    failed = total - passed

    print("=" * 50)
    print(f"验证结果: {passed}/{total} 通过")

    if failed == 0:
        print("✓ 所有检查通过！")
        print("\n下一步:")
        print("  1. 运行编译测试: cd tools/selftest && python armclang_check.py")
        print("  2. 运行单元测试: python run_selftest.py && python run_control_selftest.py")
        print("  3. 如果测试通过，用 git status 检查改动再手动提交")
        return 0
    else:
        print(f"✗ {failed} 个检查失败")
        print("请检查上述失败项")
        return 1

if __name__ == "__main__":
    sys.exit(main())
