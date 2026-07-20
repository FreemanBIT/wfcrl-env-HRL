"""
OpenFAST 输出通道注入工具
=========================
内部模块，负责向各风机模块（ElastoDyn / ServoDyn / InflowWind）的
输入文件中自动添加 OutList 通道。

FAST.Farm 的 .outb 文件仅包含已在各模块 OutList 中注册的通道。
若 .fst 顶层 OutList 没有对应条目，即使 .fstf 中指定了通道名，
输出的 .outb 也不包含该数据。本模块确保测量通道被正确注入到每个
风机子模块的输入文件中。

用法
----
# (内部使用，不直接导出)
from wfcrl.engine._outlist import _set_fast_scalar, _inject_outlist_channels

_inject_outlist_channels(
    module_path="/path/to/ElastoDyn.dat",
    channels=["YawPzn", "BldPitch1", "RotSpeed"],
)

_set_fast_scalar(
    path="/path/to/ElastoDyn.dat",
    key="NacYaw",
    value="0.0",
)
"""

from __future__ import annotations

import os
import re
from typing import List, Optional


# =========================================================================
# 各模块应写入的默认 OutList 通道
# =========================================================================

# ElastoDyn 通道（偏航、变桨、转速、叶根载荷）
ED_OUTS = ["YawPzn", "BldPitch1", "RotSpeed", "RootMIP1", "RootMOoP1", "RootMzb1"]

# ServoDyn 通道（功率、转矩）
SRV_OUTS = ["GenPwr", "GenTq"]

# InflowWind 通道（风速分量）
IFW_OUTS = ["Wind1VelX", "Wind1VelY", "Wind1VelZ"]


# =========================================================================
# _set_fast_scalar — 修改 OpenFAST 输入文件中的标量参数
# =========================================================================

def _set_fast_scalar(path: str, key: str, value: str) -> None:
    """在 OpenFAST 输入文件中就地修改一个标量参数（按注释列的参数名匹配）。

    OpenFAST 行格式为：`   <value>   <KEY>   - description`。
    本函数按第二列的 KEY 精确匹配该行，替换第一列的值，保留其余内容与 CRLF。
    仅改**首个**匹配行。用原文本编辑而非 FASTInputFile.write()，
    以免丢失文件里已追加的 OutList 通道。

    Parameters
    ----------
    path : str
        输入文件路径（如 ElastoDyn.dat）。
    key : str
        参数名（第二列，如 "NacYaw"）。
    value : str
        新值字符串（替换第一列）。
    """
    if not os.path.exists(path):
        return
    with open(path, "rb") as f:
        raw = f.read()
    text = raw.decode("ascii", errors="replace").replace("\r\n", "\n")
    lines = text.split("\n")
    for i, line in enumerate(lines):
        toks = line.split()
        if len(toks) >= 2 and toks[1] == key:
            idx0 = line.find(toks[0])
            new_line = line[:idx0] + value + line[idx0 + len(toks[0]):]
            lines[i] = new_line
            break
    out = "\n".join(lines).encode("ascii", errors="replace")
    out = out.replace(b"\r\n", b"\n").replace(b"\n", b"\r\n")
    with open(path, "wb") as f:
        f.write(out)


# =========================================================================
# _inject_outlist_channels — 向模块输入文件注入 OutList 通道
# =========================================================================

def _inject_outlist_channels(module_path: str, channels: List[str]) -> None:
    """把 channels 注入某模块输入文件的 OutList 段（若尚未存在）。

    在包含 'OutList' 的行之后、'END' 段之前插入 '"Ch"  Ch' 行。
    已存在的通道不重复添加。保持 CRLF。

    Parameters
    ----------
    module_path : str
        模块输入文件路径（如 ElastoDyn.dat, ServoDyn.dat, InflowWind.dat）。
    channels : List[str]
        待注入的通道名列表（如 ["YawPzn", "BldPitch1"]）。
    """
    if not os.path.exists(module_path):
        return
    with open(module_path, "rb") as f:
        raw = f.read()
    text = raw.decode("ascii", errors="replace")
    text = text.replace("\r\n", "\n")
    lines = text.split("\n")

    # 找 OutList 段头：该行的**参数名列**（第一个 token，或紧跟数值后的 token）
    # 是 'OutList'。特征：包含独立 token 'OutList'，且**不是**以引号开头的通道行，
    # 也**不是** END 行。
    outlist_idx = None
    for i, l in enumerate(lines):
        s = l.strip()
        if s.startswith('"') or s.upper().startswith("END"):
            continue
        toks = s.split()
        if toks and toks[0] == "OutList":
            outlist_idx = i
            break
    if outlist_idx is None:
        return  # 该模块没有 OutList 段，跳过

    # 找该段的 END（第一个以 END 开头的行，在 outlist_idx 之后）
    end_idx = None
    for i in range(outlist_idx + 1, len(lines)):
        if lines[i].strip().upper().startswith("END"):
            end_idx = i
            break
    if end_idx is None:
        end_idx = len(lines)

    # 现有通道名集合（该段内已引用的）
    existing = set()
    for i in range(outlist_idx + 1, end_idx):
        s = lines[i].strip()
        if s.startswith('"'):
            nm = s.split('"')
            if len(nm) >= 2:
                existing.add(nm[1].strip())

    # 待插入的新通道
    to_add = [c for c in channels if c not in existing]
    if not to_add:
        return
    insert_lines = [f'"{c}"    {c}' for c in to_add]
    lines = lines[:end_idx] + insert_lines + lines[end_idx:]

    out = "\n".join(lines).encode("ascii", errors="replace")
    out = out.replace(b"\r\n", b"\n").replace(b"\n", b"\r\n")
    with open(module_path, "wb") as f:
        f.write(out)
