# -*- coding: utf-8 -*-
"""
提交包打包脚本
============================================================
从开发仓库清理生成最终线上提交ZIP包。

打包前清理:
  1. 删除 .git/ .gitignore
  2. 删除 dataset/ 样本文件夹
  3. 删除报告Word草稿、调研笔记、临时测试脚本
  4. 删除 scripts/ (打包脚本本身不提交)
  5. 删除 sim_test_log/ 原始日志(保留汇总表)
  6. 删除冗余日志、视频录屏

打包操作:
  选中外层文件夹 -> 右键压缩为zip
  不要直接选中 src docs 等内层文件打包

用法:
  python scripts/pack_submission.py --team "XX小队"
  python scripts/pack_submission.py --team "XX小队" --output D:/

交付物标准 (参赛手册 4.5):
  - 完整Python算法代码包(仅允许单模块Agent文件)
  - 算法依赖文件
  - 技术报告(>=2000字)
  - 可选: 仿真训练评分过程文件、推演演示视频

负责人: 队长(成员5)
"""

import argparse
import os
import shutil
import zipfile


def clean_dev_repo(src_dir, dst_dir):
    """清理开发仓库，生成提交包目录"""
    # TODO: 复制并清理文件
    pass


def pack_zip(src_dir, zip_path):
    """打包为ZIP文件"""
    # TODO: 压缩为zip
    pass


def main():
    parser = argparse.ArgumentParser(description="HF2026 提交包打包脚本")
    parser.add_argument("--team", type=str, required=True, help="队伍名称")
    parser.add_argument("--output", type=str, default=".", help="输出目录")
    args = parser.parse_args()

    print(f"[HF2026] 开始打包: {args.team}")
    # TODO: 执行打包流程
    print(f"[HF2026] 打包完成")


if __name__ == "__main__":
    main()
