/*
 * yaw_adapter.c — 偏航协议适配器（DR-5）
 *
 * 正式通道（DR-5）：绝对机舱目标方位 yaw_target_heading_rad（OpenFAST 内部
 *   约定，CCW+，0=+X）。Bridge 核心只认绝对目标。
 * 旧离线协议（兼容适配器）：yaw_delta_rad（相对生效瞬间机舱位置的增量，
 *   CW+，罗盘方向）。适配器把 delta 转成绝对目标后喂入核心。
 *
 * 转换（与 angle_convention 语义一致，仅此一处做换算）：
 *   delta_internal = -yaw_delta                 （CW+ → CCW+ 取反）
 *   target         = wrap_pm_pi(heading_now + delta_internal)
 *
 * 注意：RT 侧（DR-1）不得再次反号；绝对目标通道不经本适配器。
 */
#include "fcr_bridge_core.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double wrap_pm_pi_local(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

/* 旧协议：delta_rad（CW+，罗盘）→ 绝对目标（内部约定）。
 * heading_now_rad 为动作生效瞬间机舱方位（内部约定）。 */
double fcr_yaw_adapter_delta_to_target(double yaw_delta_rad, double heading_now_rad) {
    double delta_internal = -yaw_delta_rad;      /* CW+ → CCW+ */
    return wrap_pm_pi_local(heading_now_rad + delta_internal);
}

/* 校验绝对目标合法性（有限且在合理范围）。 */
int fcr_yaw_adapter_validate_abs(double target_rad) {
    if (!isfinite(target_rad)) return -1;
    if (target_rad < -10.0 * M_PI || target_rad > 10.0 * M_PI) return -2;
    return 0;
}
