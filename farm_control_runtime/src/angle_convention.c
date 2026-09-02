/*
 * angle_convention.c — 角度与单位转换的唯一实现（Phase 1）
 *
 * 全仓库只允许在本文件做坐标系单位转换：
 *   - 罗盘顺时针（CW+）与 OpenFAST 内部逆时针（CCW+，0=+X）之间的换算；
 *   - wrap 到 (-PI, PI] 或 [0, 2*PI)；
 *   - 度/弧度互转；
 *   - 有限性检查。
 */
#include "fcr_internal.h"
#include <math.h>

#define FCR_PI 3.141592653589793238462643383279502884

double fcr_wrap_pm_pi(double a)
{
    /* wrap 到 (-PI, PI] */
    while (a >  FCR_PI) a -= 2.0 * FCR_PI;
    while (a <= -FCR_PI) a += 2.0 * FCR_PI;
    return a;
}

double fcr_wrap_0_2pi(double a)
{
    double w = fmod(a, 2.0 * FCR_PI);
    if (w < 0.0) w += 2.0 * FCR_PI;
    return w;
}

double fcr_deg2rad(double d) { return d * (FCR_PI / 180.0); }
double fcr_rad2deg(double r) { return r * (180.0 / FCR_PI); }

double fcr_cw_delta_to_internal(double delta_cw_rad)
{
    /* 罗盘（顺时针，北→东）增量 → OpenFAST 内部（逆时针，+X→+Y）增量：取反 */
    return -delta_cw_rad;
}

double fcr_compute_yaw_target(double heading_now_rad, double delta_cw_rad)
{
    double delta_internal = fcr_cw_delta_to_internal(delta_cw_rad);
    return fcr_wrap_pm_pi(heading_now_rad + delta_internal);
}

int fcr_is_finite(double x)
{
    return isfinite(x) != 0;
}
