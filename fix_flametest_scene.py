#!/usr/bin/env python3
"""修复 USD 场景中的绝对路径并生成 lab_flametest_v17.usd（已废弃）

⚠️ 历史脚本：本脚本是 v17 时代的转换工具（v17fix -> v17.usd），
现规范文件已是 assets/scenes/c_flame/c1_flame_wire_solid/c1_flame_wire_solid.usd
（由 fix_flametest_v17.py 产出并重命名）。本脚本仅作历史保留，输出到 legacy 文件名
lab_flametest_v17.usd，不覆盖 c1_flame_wire_solid.usd。

用法（在服务器或本地仓库根目录执行）：
    python fix_flametest_scene.py

原理：
  v17fix 是 USDA 文本文件，其中包含本地绝对路径（E:/浙江大学/...）
  这些路径在服务器上无法解析，导致贴图/MDL 丢失 → 红色背景
  本脚本将绝对路径替换为相对路径（../../base/lab_001/ → 从
  c_flame/c1_flame_wire_solid/ 指向 scenes/base/lab_001/）
  然后输出为 lab_flametest_v17.usd
"""
import os
import re
import sys

def main():
    # 定位场景目录（新分类结构：scenes/c_flame/c1_flame_wire_solid/）
    script_dir = os.path.dirname(os.path.abspath(__file__))
    scene_dir = os.path.join(script_dir, 'assets', 'scenes', 'c_flame', 'c1_flame_wire_solid')

    v17fix = os.path.join(scene_dir, 'lab_flametest.usd.v17fix')
    v17_usd = os.path.join(scene_dir, 'lab_flametest_v17.usd')
    
    if not os.path.exists(v17fix):
        print(f"ERROR: v17fix file not found: {v17fix}")
        sys.exit(1)
    
    print(f"Reading: {v17fix}")
    with open(v17fix, 'r', encoding='utf-8') as f:
        text = f.read()
    
    # 定义需要替换的绝对路径前缀（正斜杠和反斜杠两种形式）
    # lab_001 的贴图和材质路径：E:/.../chemistry_lab/lab_001/ -> ../../base/lab_001/
    prefixes = [
        # 正斜杠形式（asset 引用 @...@ 中的路径）
        (r'E:/[^@]*?/chemistry_lab/lab_001/', '../../base/lab_001/'),
        # 反斜杠形式（注释中的路径）
        (r'E:\\[^@]*?\\chemistry_lab\\lab_001\\', '../../base/lab_001/'),
        # 通用：任何 E: 开头的本地路径指向 chemistry_lab 下的子目录
        (r'E:/[^@]*?/chemistry_lab/', '../../scenes/'),
        (r'E:\\[^@]*?\\chemistry_lab\\', '../../scenes/'),
    ]
    
    original_text = text
    total_replacements = 0
    
    for pattern, replacement in prefixes:
        # 统计替换次数
        matches = re.findall(pattern, text)
        if matches:
            count = len(matches)
            total_replacements += count
            print(f"  Pattern: {pattern[:50]}... -> {replacement} ({count} matches)")
            text = re.sub(pattern, replacement, text)
    
    # 验证：检查是否还有残留的 E: 路径
    remaining = re.findall(r'E:[/\\][^\s@]+', text)
    if remaining:
        print(f"\nWARNING: {len(remaining)} absolute paths still remain:")
        for r in remaining[:5]:
            print(f"  {r}")
        if len(remaining) > 5:
            print(f"  ... and {len(remaining) - 5} more")
    else:
        print(f"\nOK: No absolute paths remaining")
    
    # 写入 v17.usd
    print(f"\nWriting: {v17_usd}")
    with open(v17_usd, 'w', encoding='utf-8') as f:
        f.write(text)
    
    print(f"Done! Total replacements: {total_replacements}")
    print(f"Scene file: {v17_usd}")
    
    # 验证文件大小
    orig_size = os.path.getsize(v17fix)
    new_size = os.path.getsize(v17_usd)
    print(f"Size: {orig_size} -> {new_size} bytes (diff={orig_size - new_size})")

if __name__ == '__main__':
    main()
