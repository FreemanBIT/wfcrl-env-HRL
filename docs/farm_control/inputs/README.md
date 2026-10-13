# FAST.Farm WFC 场控接口 — 上游输入文件

本目录保存上游提供的原始输入（只读参考，勿改）：

| 文件 | 说明 | sha256 |
|---|---|---|
| FASTFarm_WFC_Bridge_Development_Plan_v1_1.md | 开发计划 v1.1（上游原始版）| 00aebbc8d59c79671e98f68350c3d29bec59429368d1cb408aa46aa5df6fa62e |
| FASTFarm_WFC_Interface_ICD_v1_1.csv | 接口控制文档 v1.1（30 信号）| 7c2e4acc881c5b4ac2110bfa65b70eec80c04dbc2dc89e7a3773e6e8757a6441 |

## 当前实施基线

- **实施计划 v2**（决策修订版，含 7 条 DR + Phase 0-6 + 门禁 G0-G6）：
  见上级目录 `../BRIDGE_DEV_PLAN_v2.md`（commit a55a68d）。
- 决策来源：场控接口项目实施决策（DR-1..DR-9），已全部纳入 v2。
- M0 进度跟踪：`../../../induction_vs_yaw_study/docs/EXECUTION_LOG.md`（M0 节）。

## 状态

- [x] 2026-10-13 保存上游输入 + v2 计划落库（commit a55a68d）
- [ ] M0 剩余：0.1 ROSCO fork diff、0.3 多机隔离、0.5 实时核心拆分设计
- [ ] M1 Bridge 实时核心（G1）→ 提交 Bridge 接口设计供审查