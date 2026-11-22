#!/bin/bash
# P0/P1/P2 修复提交脚本

echo "=== P0/P1/P2 修复提交脚本 ==="
echo ""

# 显示当前状态
echo "1. 检查当前状态..."
git status --short

echo ""
echo "2. 检查修改的文件..."
echo "   - BSP/Src/bsp_uart.c"
echo "   - Components/Device/Src/Image_Transmission.c"
echo "   - Components/Device/Inc/Image_Transmission.h"
echo "   - Components/Device/Src/Remote_Control.c"
echo "   - tools/selftest/armclang_check.py"
echo "   - docs/*.md (新增)"

echo ""
echo "3. 创建分支 fix/p0-p1-p2-boundary-fixes"
git checkout -b fix/p0-p1-p2-boundary-fixes

echo ""
echo "4. 添加文件到暂存区..."
git add BSP/Src/bsp_uart.c
git add Components/Device/Src/Image_Transmission.c
git add Components/Device/Inc/Image_Transmission.h
git add Components/Device/Src/Remote_Control.c
git add tools/selftest/armclang_check.py
git add docs/P0_P1_FIXES.md
git add docs/P2_ARMCLANG_CHECK_FIX.md
git add docs/P0_P1_P2_SUMMARY.md
git add docs/VERIFICATION_CHECKLIST.md

echo ""
echo "5. 显示暂存的修改..."
git diff --cached --stat

echo ""
echo "6. 创建提交..."
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

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"

echo ""
echo "7. 显示提交信息..."
git log -1 --stat

echo ""
echo "=== 提交完成 ==="
echo ""
echo "下一步："
echo "  1. 运行验证测试（参见 docs/VERIFICATION_CHECKLIST.md）"
echo "  2. 如果测试通过，推送分支: git push -u origin fix/p0-p1-p2-boundary-fixes"
echo "  3. 在 GitHub 创建 Pull Request"
echo ""
