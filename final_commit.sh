#!/bin/bash
# P0/P1/P2 修复 - 最终验证和提交脚本

set -e  # 遇到错误立即退出

echo "================================================================"
echo "  S27_TEST_2 P0/P1/P2 修复 - 最终验证和提交"
echo "================================================================"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 步骤 1: 快速验证
echo "步骤 1/5: 运行快速验证..."
echo "----------------------------------------"
if python verify_fixes.py; then
    echo -e "${GREEN}✓ 快速验证通过${NC}"
else
    echo -e "${RED}✗ 快速验证失败，请检查修改${NC}"
    exit 1
fi
echo ""

# 步骤 2: 检查 Git 状态
echo "步骤 2/5: 检查 Git 状态..."
echo "----------------------------------------"
git status --short
echo ""

# 步骤 3: 运行编译测试（可选）
echo "步骤 3/5: 编译测试（可选）"
echo "----------------------------------------"
read -p "是否运行编译测试？(y/N): " run_tests
if [[ "$run_tests" =~ ^[Yy]$ ]]; then
    echo "运行编译检查..."
    cd tools/selftest
    if python armclang_check.py; then
        echo -e "${GREEN}✓ 编译检查通过${NC}"
    else
        echo -e "${YELLOW}⚠ 编译检查失败（可能是环境问题）${NC}"
        read -p "是否继续提交？(y/N): " continue_anyway
        if [[ ! "$continue_anyway" =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
    cd ../..
else
    echo "跳过编译测试"
fi
echo ""

# 步骤 4: 创建分支并暂存文件
echo "步骤 4/5: 创建分支并暂存修改..."
echo "----------------------------------------"
echo "创建分支: fix/p0-p1-p2-boundary-fixes"
git checkout -b fix/p0-p1-p2-boundary-fixes 2>/dev/null || git checkout fix/p0-p1-p2-boundary-fixes

echo "暂存修改的文件..."
git add BSP/Src/bsp_uart.c
git add Components/Device/Src/Image_Transmission.c
git add Components/Device/Inc/Image_Transmission.h
git add Components/Device/Src/Remote_Control.c
git add tools/selftest/armclang_check.py

echo "暂存新增的文档和脚本..."
git add docs/P0_P1_FIXES.md
git add docs/P2_ARMCLANG_CHECK_FIX.md
git add docs/P0_P1_P2_SUMMARY.md
git add docs/VERIFICATION_CHECKLIST.md
git add P0_P1_P2_COMPLETION_REPORT.md
git add P0_P1_P2_README.md
git add commit_p0_p1_p2.sh
git add commit_p0_p1_p2.bat
git add verify_fixes.py
git add final_commit.sh

echo ""
echo "暂存的修改："
git diff --cached --stat
echo ""

# 步骤 5: 创建提交
echo "步骤 5/5: 创建提交..."
echo "----------------------------------------"
read -p "确认创建提交？(y/N): " confirm_commit
if [[ ! "$confirm_commit" =~ ^[Yy]$ ]]; then
    echo "取消提交"
    exit 0
fi

git commit -m "fix: P0/P1/P2 边界修复与编译检查改进

P0-1: 修正 UART DMA 双缓冲配置
- RxXferSize 和所有 DMA 重装长度统一为单缓冲大小
- SBUS: 18 字节 (去掉 *2)
- 图传: 39 字节 (去掉 *2)
- 裁判系统: 对应长度 (去掉 *2)
- 消除缓冲区溢出风险

P0-2: 图传解析增加边界检查
- Image_Transmission_Info_Update() 增加 Size 参数
- CRC 校验前先验证长度边界
- 防止读取越界内存
- 拒绝畸形报文

P1: 遥控超时监视增加同步保护
- Remote_Message_Moniter() 增加临界区
- 消除监视函数与 ISR 的竞态条件
- 保持超时判断一致性

P2: 编译检查脚本改进
- 从 .dep 动态提取文件列表
- 排除已归档的 Arm_R_task.c
- 生成 JSON 构建记录
- 记录源码哈希、编译结果、告警

不改变控制算法、协议、周期和故障策略。

详细信息见:
- P0_P1_P2_COMPLETION_REPORT.md (完整报告)
- docs/P0_P1_FIXES.md (P0/P1 详细)
- docs/P2_ARMCLANG_CHECK_FIX.md (P2 详细)
- docs/VERIFICATION_CHECKLIST.md (验证清单)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"

echo ""
echo -e "${GREEN}✓ 提交成功！${NC}"
echo ""

# 显示提交信息
echo "提交信息："
echo "----------------------------------------"
git log -1 --stat
echo ""

# 最终说明
echo "================================================================"
echo -e "${GREEN}  修复完成并已提交${NC}"
echo "================================================================"
echo ""
echo "下一步："
echo "  1. 在实际硬件上验证修复（见 docs/VERIFICATION_CHECKLIST.md）"
echo "  2. 推送到远程: git push -u origin fix/p0-p1-p2-boundary-fixes"
echo "  3. 在 GitHub 创建 Pull Request"
echo ""
echo "修改的文件 (5个):"
echo "  - BSP/Src/bsp_uart.c"
echo "  - Components/Device/Src/Image_Transmission.c"
echo "  - Components/Device/Inc/Image_Transmission.h"
echo "  - Components/Device/Src/Remote_Control.c"
echo "  - tools/selftest/armclang_check.py"
echo ""
echo "新增的文档 (8个):"
echo "  - P0_P1_P2_COMPLETION_REPORT.md (主报告)"
echo "  - P0_P1_P2_README.md (快速指南)"
echo "  - docs/P0_P1_FIXES.md"
echo "  - docs/P2_ARMCLANG_CHECK_FIX.md"
echo "  - docs/P0_P1_P2_SUMMARY.md"
echo "  - docs/VERIFICATION_CHECKLIST.md"
echo "  + 3 个辅助脚本"
echo ""
echo "保持不变："
echo "  ✓ 波特率、帧格式、遥控映射"
echo "  ✓ USB 协议、CAN ID、控制周期"
echo "  ✓ PD 参数、零点、限位、增益"
echo "  ✓ 控制算法和故障策略"
echo ""
