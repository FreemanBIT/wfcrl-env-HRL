/*
 * fcr_bridge_core.c — ROSCO Bridge 实时核心实现（M1）
 *
 * 10ms 步进 fcr_bridge_core_step() 为纯内存运算（无锁/socket/malloc/等待），
 * 满足 DR-4 红线。诱导映射语义与现 InductionSupervisor 对齐（DR-4 复用），
 * 但独立成核心以便 RT Fast Task 直接挂载。
 *
 * 控制分配（DR-3）：
 *   Mode1 输出 R_pitch/R_speed/R_torque 三个比例，按 alloc_w_* 权重分配
 *   降载量；默认（未配置权重/和为 0）退化为纯 r_pitch 通道（R_power）。
 *   禁止未分配就同时施加多个降载量：仅当权重被显式配置时才多通道输出。
 *
 * Ct 估计（DR-7）：
 *   ct_est = 2*T/(rho*A*V^2)。V 取盘面扰动风时若 use_model_corr=1 则做
 *   模型修正（v_disk_ref_corr 缩放，修正扰动风偏高导致的 Ct 低估/高估）；
 *   经典归一化要求修正后再算（不直接用原始扰动盘面风）。
 *
 * 偏航（DR-5）：正式通道为绝对目标 yaw_target_heading_rad（内部约定）。
 *   legacy delta 协议由 yaw_adapter.c 转成绝对目标后喂入本核心。
 */
#include "fcr_bridge_core.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

struct FcrBridgeCore {
    uint32_t n_turbines;
    FcrBridgeIndConfig ind;
    FcrBridgeCtConfig  ct;
    FcrBridgeYawConfig yaw;
    FcrBridgeTurbineState t[FCR_BRIDGE_MAX_TURBINES];
};

static FcrBridgeIndConfig default_ind(void) {
    FcrBridgeIndConfig c;
    c.a0 = 0.3333333333333333;
    c.a_min = 0.05;
    /* WIN-12: 物理可达上限 = a0 (Cp 峰值在 a=1/3)；
     * 更高诱导需要 Ct 继续上升但 pitch 降载方向相反，标 UNACHIEVABLE */
    c.a_max = 0.3333333333333333;
    c.ct_coef = 1.0;
    c.power_cap = 1.0;
    c.min_pitch_max_rad = 0.30;
    return c;
}

static FcrBridgeCtConfig default_ct(void) {
    FcrBridgeCtConfig c;
    c.air_density = 1.225;
    c.rotor_area = M_PI * 63.0 * 63.0;   /* NREL 5MW R=63 */
    c.gen_eff = 0.944;
    c.v_disk_ref_corr = 1.0;
    c.use_model_corr = 1;
    return c;
}

static FcrBridgeYawConfig default_yaw(void) {
    FcrBridgeYawConfig c;
    c.deadband_rad = 2.0 * DEG2RAD;
    return c;
}

FcrBridgeCore *fcr_bridge_core_create(uint32_t n_turbines,
                                      const FcrBridgeIndConfig *ind,
                                      const FcrBridgeCtConfig *ct,
                                      const FcrBridgeYawConfig *yaw) {
    FcrBridgeCore *c;
    if (n_turbines == 0 || n_turbines > FCR_BRIDGE_MAX_TURBINES) return NULL;
    c = (FcrBridgeCore *)calloc(1, sizeof(FcrBridgeCore));
    if (!c) return NULL;
    c->n_turbines = n_turbines;
    c->ind = ind ? *ind : default_ind();
    c->ct = ct ? *ct : default_ct();
    c->yaw = yaw ? *yaw : default_yaw();
    if (c->ct.rotor_area <= 0.0) c->ct.rotor_area = default_ct().rotor_area;
    if (c->ct.air_density <= 0.0) c->ct.air_density = 1.225;
    if (c->ct.gen_eff <= 0.0 || c->ct.gen_eff > 1.0) c->ct.gen_eff = 0.944;
    return c;
}

void fcr_bridge_core_destroy(FcrBridgeCore *core) {
    if (core) free(core);
}

/* ---- 诱导映射纯函数（可单测）---- */
double fcr_bridge_mode1_power_ratio(double a, double a0) {
    double pe, pe0;
    if (a0 <= 0.0 || a0 >= 1.0) a0 = 1.0 / 3.0;
    if (!isfinite(a) || a <= 0.0 || a >= 1.0) return 1.0; /* 安全默认 */
    pe = a * (1.0 - a) * (1.0 - a);
    pe0 = a0 * (1.0 - a0) * (1.0 - a0);
    if (pe0 <= 0.0) return 1.0;
    return pe / pe0;
}

double fcr_bridge_mode2_ct_ref(double a) {
    /* Ct_ref = 4a(1-a)：理想动量理论（带 ct_coef 标定，在 step 内乘） */
    if (!isfinite(a) || a <= 0.0 || a >= 1.0) return 4.0 / 9.0; /* a=1/3 */
    return 4.0 * a * (1.0 - a);
}

/* WIN-10: bilinear interpolation of Ct(lam, beta) table */
static double ct_bilinear_interp(const double *lamv, const double *betav,
                                 const double *ctv, int n_lam, int n_beta,
                                 double lam, double beta) {
    int i, j;
    double fl, fb, ct00, ct01, ct10, ct11, ct0, ct1;
    /* clamp to table bounds */
    if (lam < lamv[0]) lam = lamv[0];
    if (lam > lamv[n_lam-1]) lam = lamv[n_lam-1];
    if (beta < betav[0]) beta = betav[0];
    if (beta > betav[n_beta-1]) beta = betav[n_beta-1];
    /* find lam bracket */
    for (i = 0; i < n_lam - 1; ++i)
        if (lam >= lamv[i] && lam <= lamv[i+1]) break;
    if (i >= n_lam - 1) i = n_lam - 2;
    fl = (lam - lamv[i]) / (lamv[i+1] - lamv[i]);
    /* find beta bracket */
    for (j = 0; j < n_beta - 1; ++j)
        if (beta >= betav[j] && beta <= betav[j+1]) break;
    if (j >= n_beta - 1) j = n_beta - 2;
    fb = (beta - betav[j]) / (betav[j+1] - betav[j]);
    /* bilinear interpolation */
    ct00 = ctv[i * n_beta + j];
    ct01 = ctv[i * n_beta + j + 1];
    ct10 = ctv[(i+1) * n_beta + j];
    ct11 = ctv[(i+1) * n_beta + j + 1];
    ct0 = ct00 + fb * (ct01 - ct00);
    ct1 = ct10 + fb * (ct11 - ct10);
    return ct0 + fl * (ct1 - ct0);
}

/* WIN-10: find beta by scanning with bilinear interpolation */
int fcr_bridge_mode2_find_beta(const FcrBridgeCmd *cmd, double lam,
                               double ct_ref, double *beta_deg_out) {
    int j;
    double best = 1e30, best_beta = 0.0, ct;
    const double *lamv, *betav, *ctv;
    if (!cmd || !beta_deg_out) return -1;
    if (cmd->aero_n_lam <= 0 || cmd->aero_n_beta <= 0) return -2;
    lamv = cmd->aero_lam_ptr ? cmd->aero_lam_ptr : cmd->aero_lam_small;
    betav = cmd->aero_beta_deg_ptr ? cmd->aero_beta_deg_ptr : cmd->aero_beta_deg_small;
    ctv = cmd->aero_ct_ptr ? cmd->aero_ct_ptr : cmd->aero_ct_small;
    /* scan beta axis with bilinear interpolation at current lambda */
    for (j = 0; j < cmd->aero_n_beta; ++j) {
        ct = ct_bilinear_interp(lamv, betav, ctv, cmd->aero_n_lam, cmd->aero_n_beta,
                                lam, betav[j]);
        double d = fabs(ct - ct_ref);
        if (d < best) { best = d; best_beta = betav[j]; }
    }
    if (best > 0.5) return -4; /* table coverage insufficient */
    *beta_deg_out = best_beta;
    return 0;
}

/* ---- 慢路径：写入命令快照 ---- */
int fcr_bridge_core_set_command(FcrBridgeCore *core, const FcrBridgeCmd *cmd) {
    FcrBridgeTurbineState *st;
    if (!core || !cmd) return -1;
    if (cmd->turbine_id < 1 || cmd->turbine_id > (int32_t)core->n_turbines) return -2;
    st = &core->t[cmd->turbine_id - 1];
    st->cmd = *cmd;
    st->cmd_valid = 1;
    return 0;
}

/* ---- 快路径：喂测量 ---- */
int fcr_bridge_core_set_meas(FcrBridgeCore *core, const FcrBridgeMeas *m) {
    FcrBridgeTurbineState *st;
    if (!core || !m) return -1;
    if (m->turbine_id < 1 || m->turbine_id > (int32_t)core->n_turbines) return -2;
    st = &core->t[m->turbine_id - 1];
    st->meas = *m;
    st->meas_valid = 1;
    return 0;
}

/* ---- 10ms 步进（纯内存；DR-4 红线合规）---- */
int fcr_bridge_core_step(FcrBridgeCore *core, int32_t turbine_id,
                         double sim_time_s, FcrRoscoExternalSetpoint *out) {
    FcrBridgeTurbineState *st;
    FcrBridgeCmd *cmd;
    FcrBridgeMeas *me;
    if (!core || !out) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)core->n_turbines) return -2;
    st = &core->t[turbine_id - 1];
    cmd = &st->cmd;
    me = &st->meas;
    {
        FcrRoscoExternalSetpoint sp;
        double pr, lam = 0.0, ct_ref = 0.0, beta_deg = 0.0;
        uint32_t cmd_active;

        memset(&sp, 0, sizeof(sp));
        sp.farm_control_enable = 1;
        sp.valid = 1;
        sp.yaw_enable = 0;
        sp.induction_enable = 0;

        if (!st->cmd_valid && !st->meas_valid) goto done;

        /* ---- 命令 TTL / stale（safety 语义）---- */
        cmd_active = st->cmd_valid;
        if (cmd_active && cmd->induction_ttl_s > 0.0 &&
            sim_time_s - cmd->source_time_s > cmd->induction_ttl_s) {
            /* 诱导命令过期：回退额定 */
            cmd_active = 0;
            st->cmd_stale = 1;
        }
        if (cmd_active && cmd->yaw_ttl_s > 0.0 &&
            sim_time_s - cmd->source_time_s > cmd->yaw_ttl_s) {
            cmd_active = 0;
            st->cmd_stale = 1;
        }

        /* ---- 偏航（DR-5 绝对目标；锁存）---- */
        if (cmd_active && st->cmd_valid && cmd->yaw_valid) {
            if (cmd->yaw_seq != st->yaw_seq_applied) {
                /* 新 seq：锁存绝对目标（OpenFAST 内部约定，无 delta 转换） */
                st->yaw_target_heading_rad = cmd->yaw_target_heading_rad;
                st->yaw_seq_applied = cmd->yaw_seq;
            }
            sp.yaw_enable = 1;
            sp.yaw_seq = st->yaw_seq_applied;
            sp.yaw_target_heading_rad = st->yaw_target_heading_rad;
        }

        /* ---- 诱导（DR-3 Mode 0/1/2/3）---- */
        if (cmd_active && st->cmd_valid && cmd->induction_valid) {
            double a = cmd->induction_ref;
            uint32_t mode = cmd->induction_mode;

            if (mode == 0) {
                /* Mode 0：原 ROSCO，所有外部 ratio=1（旁路） */
                sp.induction_enable = 0;
                st->induction_seq_applied = cmd->induction_seq;
            } else if (mode == 1) {
                /* WIN-12: 物理可达区域检查 — a > a0 不可达 */
                if (!isfinite(a) || a < core->ind.a_min) {
                    st->induction_fallback = 1;
                    st->fallback_active = 1;
                    sp.command_status_flags |= 0x4; /* FALLBACK */
                    pr = 1.0;
                } else if (a > core->ind.a0) {
                    /* WIN-12: 目标诱导高于基线 — 物理不可达。
                     * Cp 在 a=1/3 达峰，更高 a 需要 Ct 继续上升，
                     * 但 pitch 增大会降 Ct，方向矛盾。 */
                    st->induction_fallback = 1;
                    st->fallback_active = 1;
                    sp.command_status_flags |= 0x4;  /* FALLBACK */
                    sp.command_status_flags |= 0x100; /* UNACHIEVABLE */
                    pr = 1.0;
                } else {
                    st->induction_fallback = 0;
                    pr = fcr_bridge_mode1_power_ratio(a, core->ind.a0);
                    if (pr > core->ind.power_cap) pr = core->ind.power_cap;
                    if (pr < 0.05) pr = 0.05;
                }
                st->power_ratio = pr;
                st->speed_ref_ratio = pow(pr, 1.0 / 3.0);
                st->torque_limit_ratio = pow(pr, 2.0 / 3.0);
                st->induction_seq_applied = cmd->induction_seq;
                sp.induction_enable = 1;
                sp.induction_seq = cmd->induction_seq;
                /* 控制分配（DR-3）：默认全走 r_pitch；显式权重时按权分配 */
                {
                    double wsum = cmd->alloc_w_pitch + cmd->alloc_w_speed
                                + cmd->alloc_w_torque;
                    double w0 = cmd->alloc_w_pitch;
                    double w1 = cmd->alloc_w_speed;
                    double w2 = cmd->alloc_w_torque;
                    if (wsum <= 0.0) {
                        w0 = 1.0; w1 = 0.0; w2 = 0.0; /* 默认纯 r_pitch */
                    } else {
                        w0 /= wsum; w1 /= wsum; w2 /= wsum;
                    }
                    /* r_pitch/r_speed/r_torque 按权重分配（0 权重=通道不输出） */
                    sp.power_ratio = (w0 > 0.0) ? 1.0 + w0 * (pr - 1.0) : 1.0;
                    sp.speed_ref_ratio = (w1 > 0.0) ? 1.0 + w1 * (st->speed_ref_ratio - 1.0) : 1.0;
                    sp.torque_limit_ratio = (w2 > 0.0) ? 1.0 + w2 * (st->torque_limit_ratio - 1.0) : 1.0;
                    /* r_pitch 实际是 power_ratio 通道（FCR_ApplyReferences 用 power_ratio） */
                    st->min_pitch_rad = 0.0; /* Mode1 不用最小桨距约束 */
                }
            } else if (mode == 3) {
                /* WIN-15: Mode 3 显式不支持 — 反馈闭环接口，RT 阶段实现。
                 * 不静默走 Mode1，不误导试验。 */
                st->induction_fallback = 1;
                st->fallback_active = 1;
                sp.command_status_flags |= 0x4;   /* FALLBACK */
                sp.command_status_flags |= 0x100;  /* UNSUPPORTED */
                st->power_ratio = 1.0;
                st->speed_ref_ratio = 1.0;
                st->torque_limit_ratio = 1.0;
                st->min_pitch_rad = 0.0;
                sp.induction_enable = 0;  /* 不生效 */
            } else if (mode == 2) {
                /* Mode 2：Ct 前馈 → 桨距（min_pitch_ref 或 PRC） */
                if (!isfinite(a) || a < core->ind.a_min || a > core->ind.a0) {
                    /* a > a0: Ct 反查找不到（Ct 在 a=1/3 最大），标 UNACHIEVABLE */
                    st->induction_fallback = 1;
                    st->fallback_active = 1;
                    sp.command_status_flags |= 0x4;
                    if (isfinite(a) && a > core->ind.a0)
                        sp.command_status_flags |= 0x100; /* UNACHIEVABLE */
                    ct_ref = fcr_bridge_mode2_ct_ref(core->ind.a0) * core->ind.ct_coef;
                } else {
                    st->induction_fallback = 0;
                    ct_ref = fcr_bridge_mode2_ct_ref(a) * core->ind.ct_coef;
                }
                st->ct_ref = ct_ref;
                st->power_ratio = 1.0; /* Ct 前馈不直接改功率比例 */
                st->speed_ref_ratio = 1.0;
                st->torque_limit_ratio = 1.0;
                st->induction_seq_applied = cmd->induction_seq;
                sp.induction_enable = 1;
                sp.induction_seq = cmd->induction_seq;
                /* 由 Ct(lambda,beta) 反解 beta_target → min_pitch_ref */
                if (me->valid && me->rotor_speed_rad_s > 0.0 && me->disk_wind_speed_mps > 0.5) {
                    lam = me->rotor_speed_rad_s * (cmd->aero_blade_radius_m > 0.0
                          ? cmd->aero_blade_radius_m : 63.0) / me->disk_wind_speed_mps;
                }
                if (fcr_bridge_mode2_find_beta(cmd, lam, ct_ref, &beta_deg) == 0) {
                    double bp = beta_deg * DEG2RAD;
                    if (bp < 0.0) bp = 0.0;
                    if (bp > core->ind.min_pitch_max_rad) bp = core->ind.min_pitch_max_rad;
                    st->min_pitch_rad = bp;
                    sp.min_pitch_rad = bp;
                } else {
                    st->min_pitch_rad = 0.0;
                    sp.min_pitch_rad = 0.0;
                }
                sp.power_ratio = 1.0;
                sp.speed_ref_ratio = 1.0;
                sp.torque_limit_ratio = 1.0;
                /* Mode 3 桩：pitch_offset 留 0（未来闭环） */
            }
            if (st->induction_fallback) st->fallback_active = 1;
            else st->fallback_active = 0;
        }

        /* ---- Ct 估计（DR-7 模型修正）----
         * 归一化风速优先级（严格语义分层）：
         *   1) ambient_upstream_wind_mps（上游参考风）> 0.5 → 用之（最正确）
         *   2) 否则 disk_wind_speed_mps（盘面扰动风）> 0.5 → 用之 + 模型修正
         *   3) 否则 Ct 无效。
         * 禁止直接用扰动盘面风做经典归一化而不做修正。 */
        if (me->valid) {
            double V = 0.0;
            if (me->ambient_upstream_wind_mps > 0.5) {
                V = me->ambient_upstream_wind_mps;
            } else if (me->disk_wind_speed_mps > 0.5) {
                V = me->disk_wind_speed_mps;
                if (core->ct.use_model_corr && core->ct.v_disk_ref_corr > 0.0 &&
                    fabs(core->ct.v_disk_ref_corr - 1.0) > 1e-9) {
                    V = me->disk_wind_speed_mps / core->ct.v_disk_ref_corr;
                }
            }
            if (V > 0.5 && me->rotor_thrust_n > 0.0) {
                st->ct_est = 2.0 * me->rotor_thrust_n
                           / (core->ct.air_density * core->ct.rotor_area * V * V);
                st->ct_est_valid = 1;
            } else {
                st->ct_est_valid = 0;
            }
        }

        /* ---- 汇总 setpoint ---- */
        sp.ct_ref = st->ct_ref;
        sp.min_pitch_rad = st->min_pitch_rad;
        sp.power_ratio = (sp.induction_enable && sp.power_ratio > 0.0) ? sp.power_ratio : 1.0;
        sp.speed_ref_ratio = (sp.induction_enable && sp.speed_ref_ratio > 0.0) ? sp.speed_ref_ratio : 1.0;
        sp.torque_limit_ratio = (sp.induction_enable && sp.torque_limit_ratio > 0.0) ? sp.torque_limit_ratio : 1.0;
        if (st->fallback_active) sp.command_status_flags |= 0x4;

    done:
        *out = sp;
    }
    return 0;
}

int fcr_bridge_core_get(FcrBridgeCore *core, int32_t turbine_id,
                        FcrBridgeTurbineState *out) {
    if (!core || !out) return -1;
    if (turbine_id < 1 || turbine_id > (int32_t)core->n_turbines) return -2;
    *out = core->t[turbine_id - 1];
    return 0;
}
