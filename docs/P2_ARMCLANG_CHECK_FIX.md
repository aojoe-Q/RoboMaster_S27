# P2: 编译检查脚本修复

修复日期：2026-11-11  
文件：`tools/selftest/armclang_check.py`

## 问题描述

编译检查脚本使用硬编码的文件列表，导致以下问题：
1. 引用了已归档的 `Arm_R_task.c`（已移至 `archive/`）
2. 硬编码列表与实际工程清单不同步
3. 没有记录编译参数、源码摘要和检查历史
4. 当文件清单变化时需要手工修改脚本

## 修复内容

### 1. 动态提取文件列表
- 从 `.dep` 文件动态提取所有 C 文件
- 过滤到项目源码（APP、Components、Modules、BSP）
- 排除已归档文件（可配置的排除列表）
- 检查时输出发现的文件数量和排除的文件

### 2. 添加构建记录
新增 JSON 格式的构建记录（`build_record.json`），包含：

```json
{
  "timestamp": "构建时间戳",
  "dep_file": ".dep 文件路径",
  "compiler": "编译器路径",
  "flags": "编译标志",
  "excluded_files": ["排除的文件列表"],
  "files": [
    {
      "path": "源文件路径",
      "hash": "SHA256摘要（前16位）",
      "returncode": "编译返回码",
      "obj_created": "目标文件是否生成",
      "warnings": ["告警列表"]
    }
  ],
  "summary": {
    "total_files": "总文件数",
    "failed": "失败数",
    "warnings": "告警总数"
  }
}
```

### 3. 源码完整性验证
- 为每个源文件计算 SHA256 哈希（前 16 位）
- 记录到 build_record.json
- 后续检查可以对比哈希判断源码是否变化

### 4. 改进的错误处理
- 检测 .dep 文件中无项目源文件的情况
- 明确报告排除了哪些文件
- 统计并报告总的告警数量

## 代码变更

### 新增导入
```python
import json
import hashlib
from datetime import datetime
```

### 动态文件提取
```python
# 从 .dep 提取所有 C 文件
dep_files = []
for line in dep.split("\n"):
    if line.startswith("F (.."):
        m = re.match(r"F \(([^)]+\.c)\)", line)
        if m:
            dep_files.append(m.group(1))

# 过滤并排除已归档文件
excluded = [r"..\APP\Arm_R_task.c"]
files = [f for f in dep_files
         if any(x in f for x in [r"\APP\\", r"\Components\\", r"\Modules\\", r"\BSP\\"])
         and f not in excluded]
```

### 构建记录生成
```python
# 为每个文件计算哈希
src_path = os.path.join(MDK, f)
src_hash = ""
if os.path.exists(src_path):
    with open(src_path, "rb") as sf:
        src_hash = hashlib.sha256(sf.read()).hexdigest()[:16]

# 记录编译结果
file_record = {
    "path": f,
    "hash": src_hash,
    "returncode": r.returncode,
    "obj_created": os.path.exists(obj),
    "warnings": own
}
build_record["files"].append(file_record)

# 保存记录
record_path = os.path.join(OUT, "build_record.json")
with open(record_path, "w") as f:
    json.dump(build_record, f, indent=2)
```

## 使用方法

### 运行检查
```bash
cd tools/selftest
python armclang_check.py
```

### 输出示例
```
Found 35 project C files in .dep
Excluded: ..\APP\Arm_R_task.c
..\APP\Arm_L_task.c                   rc=0 obj ok
..\APP\DM_motor.c                     rc=0 obj ok
...
Build record saved to: tools/selftest/build/armclang_obj/build_record.json
Summary: 35 files, 0 failures, 7 warnings
```

### 检查构建记录
```bash
cat tools/selftest/build/armclang_obj/build_record.json
```

## 添加排除文件

如果需要排除更多文件，修改 `excluded` 列表：

```python
excluded = [
    r"..\APP\Arm_R_task.c",      # 已归档
    r"..\APP\some_other_file.c"  # 其他需要排除的文件
]
```

## 后续改进建议

1. **增量编译**：利用哈希值判断源文件是否变化，只重新编译变化的文件
2. **历史对比**：保留多个 build_record，对比不同版本的告警变化
3. **CI 集成**：在 CI 流程中运行并报告告警趋势
4. **告警过滤**：配置告警白名单，只报告新增告警

## 验证

运行编译检查并验证：
1. 不再尝试编译 `Arm_R_task.c`
2. 成功编译所有当前工程中的项目源文件
3. 生成 `build_record.json` 记录
4. 退出码反映实际失败数量

## 影响

- **向后兼容**：输出格式和退出码保持不变
- **功能增强**：自动同步工程清单，记录构建历史
- **维护改善**：添加/移除文件时无需修改脚本
