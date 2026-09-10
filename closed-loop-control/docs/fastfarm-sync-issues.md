> [历史诊断笔记] 本文记录早期文件桥同步问题排查。当前 ZMQ 架构（FCR_FARM_CONTROL）下
> 时序同步由 CommandStore seq/TTL + wait_step 处理，本文档仅供回溯。
# FAST.Farm 闭环控制验证中的同步问题总结

## 背景