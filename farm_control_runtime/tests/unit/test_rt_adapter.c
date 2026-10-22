/*
 * test_rt_adapter.c — RT 平台接入层测试（要求 §3 生命周期 / §5 时间 / §6 状态 / §14 数据）
 *
 * 覆盖：
 *   - 生命周期：configure 校验、重复 init 拒绝、未 init 调 step、reset 后 run→reset→run 可重复
 *   - 时间：非法/回退时间上下文拒绝；SCADA 边界由计数器（100×10ms=1s）
 *   - 状态：NOT_READY / INVALID_INPUT / TIMEOUT（边沿）/ OK
 *   - 数据：NaN 命令被拒绝（INVALID_VALUE + INVALID_INPUT）
 */
#include "test_framework.h"
#include "fcr_rt.h"
#include <string.h>
#include <math.h>

static FcrRtConfig mk_cfg(uint32_t n)
{
    FcrRtConfig c;
    memset(&c, 0, sizeof(c));
    c.n_turbines = n;
    c.dt_high_s = 0.01;
    c.dt_low_s = 1.0;
    c.dt_scada_s = 1.0;
    c.default_yaw_ttl_s = 60.0;
    c.default_induction_ttl_s = 5.0;
    c.stale_threshold_s = 5.0;
    c.flow_stale_threshold_s = 3.0;
    c.bridge_enable = 0;
    c.bridge_induction_mode = 1;
    return c;
}

static void put_state(FcrRtHandle *h, int tid, double heading)
{
    FcrRoscoRawState st;
    memset(&st, 0, sizeof(st));
    st.turbine_id = tid;
    st.valid = 1;
    st.nacelle_heading_rad = heading;
    st.gen_power_w = 2.0e6;
    st.rotor_speed_rad_s = 1.2;
    ASSERT(fcr_rt_set_state(h, tid, &st) == 0);
}

static void mk_yaw_cmd(FarmCommandFrame *fr, int tid, double target,
                       uint64_t seq, double basis)
{
    memset(fr, 0, sizeof(*fr));
    fr->protocol_version = FCR_PROTOCOL_VERSION;
    fr->n_turbines = (uint32_t)tid;
    fr->command_frame_seq = 1;
    fr->turbines[tid - 1].control_enable = 1;
    fr->turbines[tid - 1].control_mode = FCR_CONTROL_MODE_PRC;
    fr->turbines[tid - 1].command_basis_sim_time_s = basis;
    fr->turbines[tid - 1].yaw_valid = 1;
    fr->turbines[tid - 1].yaw_seq = seq;
    fr->turbines[tid - 1].yaw_target_heading_rad = target;
    fr->turbines[tid - 1].yaw_ttl_s = 60.0;
}

static void mk_ind_cmd(FarmCommandFrame *fr, int tid, double a,
                       uint64_t seq, double basis, double ttl)
{
    memset(fr, 0, sizeof(*fr));
    fr->protocol_version = FCR_PROTOCOL_VERSION;
    fr->n_turbines = (uint32_t)tid;
    fr->command_frame_seq = 1;
    fr->turbines[tid - 1].control_enable = 1;
    fr->turbines[tid - 1].control_mode = FCR_CONTROL_MODE_PRC;
    fr->turbines[tid - 1].command_basis_sim_time_s = basis;
    fr->turbines[tid - 1].induction_valid = 1;
    fr->turbines[tid - 1].induction_seq = seq;
    fr->turbines[tid - 1].induction_ref = a;
    fr->turbines[tid - 1].induction_ttl_s = ttl;
}

/* 第 k 次 10 ms 调用（k 从 0 起）：n_low=k/100, n_fast_low=k%100 */
static FcrRtStatus step_k(FcrRtHandle *h, int k, FcrRtStatusInfo *si)
{
    FcrRtTimeCtx t;
    memset(&t, 0, sizeof(t));
    t.n_low = k / 100;
    t.n_fast_low = k % 100;
    t.dt_low = 1.0;
    t.dt_high = 0.01;
    t.n_high_low = 100;
    t.t_low = (double)t.n_low + (double)t.n_fast_low * 0.01;
    return fcr_rt_step(h, &t, si);
}

/* ---- 配置校验（要求 §3.1 / §13 invalid init）---- */
static void test_configure_validation(void)
{
    FcrRtHandle *h = fcr_rt_create();
    FcrRtConfig c;
    ASSERT(h != NULL);

    /* 未 configure 就 init → 拒绝 */
    ASSERT(fcr_rt_init(h) == -2);

    c = mk_cfg(0);
    ASSERT(fcr_rt_configure(h, &c) == -2);          /* n_turbines=0 */

    c = mk_cfg(2);
    c.dt_high_s = 0.0;
    ASSERT(fcr_rt_configure(h, &c) == -2);          /* dt_high=0 */

    c = mk_cfg(2);
    c.dt_scada_s = 0.015;                            /* 1.5 个 10ms 步 → 非整数 */
    ASSERT(fcr_rt_configure(h, &c) == -2);

    c = mk_cfg(2);
    c.dt_scada_s = 0.5;                              /* 50 步，可整除 1s */
    ASSERT(fcr_rt_configure(h, &c) == 0);

    c = mk_cfg(2);
    ASSERT(fcr_rt_configure(h, &c) == 0);
    ASSERT(fcr_rt_init(h) == 0);
    ASSERT(fcr_rt_init(h) == -3);                    /* 重复 init 拒绝 */
    fcr_rt_destroy(h);
}

/* ---- run before init（要求 §13）---- */
static void test_run_before_init(void)
{
    FcrRtHandle *h = fcr_rt_create();
    FcrRtStatusInfo si;
    FcrRtTimeCtx t;
    ASSERT(h != NULL);
    memset(&t, 0, sizeof(t));
    t.dt_low = 1.0;
    t.dt_high = 0.01;
    ASSERT(fcr_rt_step(h, &t, &si) == FCR_RT_NOT_READY);
    fcr_rt_destroy(h);
}

/* ---- 时间上下文校验（要求 §14 数据/§5 时间）---- */
static void test_step_time_validation(void)
{
    FcrRtHandle *h = fcr_rt_create();
    FcrRtConfig c = mk_cfg(2);
    FcrRtStatusInfo si;
    FcrRtTimeCtx t;
    ASSERT(fcr_rt_configure(h, &c) == 0);
    ASSERT(fcr_rt_init(h) == 0);

    ASSERT(fcr_rt_step(h, NULL, &si) == FCR_RT_INVALID_INPUT);
    ASSERT(si.error_source == FCR_RT_SRC_TIME);

    memset(&t, 0, sizeof(t));
    t.n_low = -1;
    t.dt_low = 1.0;
    ASSERT(fcr_rt_step(h, &t, &si) == FCR_RT_INVALID_INPUT);

    /* 正常步进后时间回退 → 拒绝 */
    ASSERT(step_k(h, 200, &si) != FCR_RT_INVALID_INPUT);
    memset(&t, 0, sizeof(t));
    t.n_low = 0; t.t_low = 0.0; t.dt_low = 1.0;
    ASSERT(fcr_rt_step(h, &t, &si) == FCR_RT_INVALID_INPUT);
    ASSERT(si.error_code == 5);
    fcr_rt_destroy(h);
}

/* ---- SCADA 边界：total_step 跨 100 步（=1 s）出帧（要求 §5.2）---- */
static void test_scada_boundary_counter(void)
{
    FcrRtHandle *h = fcr_rt_create();
    FcrRtConfig c = mk_cfg(1);
    FcrRtStatusInfo si;
    FarmStateFrame out;
    int k;
    ASSERT(fcr_rt_configure(h, &c) == 0);
    ASSERT(fcr_rt_init(h) == 0);
    put_state(h, 1, 0.0);

    /* k=0..99（t=0.00..0.99）无 1 s 帧 */
    for (k = 0; k <= 99; ++k) {
        ASSERT(step_k(h, k, &si) == FCR_RT_OK);
        ASSERT(fcr_rt_get_output(h, &out) == 0);
    }
    /* k=100（n_low=1, t=1.00）→ 产生 1 s 帧，采样时刻 = 1.00 */
    ASSERT(step_k(h, 100, &si) == FCR_RT_OK);
    ASSERT(fcr_rt_get_output(h, &out) == 1);
    ASSERT(out.protocol_version == FCR_PROTOCOL_VERSION);
    ASSERT(out.n_turbines == 1);
    ASSERT_NEAR(out.state_sample_sim_time_s, 1.00, 1e-9);
    /* 取走后不重复返回 */
    ASSERT(fcr_rt_get_output(h, &out) == 0);
    /* 下一 1 s 边界（k=200） */
    for (k = 101; k <= 199; ++k) {
        step_k(h, k, &si);
        ASSERT(fcr_rt_get_output(h, &out) == 0);
    }
    step_k(h, 200, &si);
    ASSERT(fcr_rt_get_output(h, &out) == 1);
    ASSERT_NEAR(out.state_sample_sim_time_s, 2.00, 1e-9);
    fcr_rt_destroy(h);
}

/* ---- A1：Mock Scheduler 1000 周期（要求 §15 / M3）----
 * DT_BASE=10 ms、DT_Low=1 s、DT_SCADA=1 s；运行 1000 个 BASE 周期（t=0.01..10.00）。
 * 检查：step 调用 1000 次；SCADA 出帧 10 次（t=1..10）；无重复/漏调；命令生效时刻明确。
 */
static void test_mock_scheduler_1000_cycles(void)
{
    FcrRtHandle *h = fcr_rt_create();
    FcrRtConfig c = mk_cfg(1);
    FcrRtStatusInfo si;
    FarmStateFrame out;
    int k, n_calls = 0, n_out = 0, n_dup = 0;
    int n_cmd = 0;
    double first_t = -1.0, last_t = -1.0;
    double out_t[16];
    FcrRoscoExternalSetpoint sp;
    FarmCommandFrame fr;

    ASSERT(fcr_rt_configure(h, &c) == 0);
    ASSERT(fcr_rt_init(h) == 0);

    /* 1-based 周期：k=1..1000，t=k*0.01（0.01..10.00），n_low=k/100, n_fast=k%100 */
    for (k = 1; k <= 1000; ++k) {
        double t = (double)k * 0.01;
        int n_low = k / 100;
        int n_fast = k % 100;
        FcrRtTimeCtx ctx;
        FcrRtStatus st;

        memset(&ctx, 0, sizeof(ctx));
        ctx.n_low = n_low;
        ctx.n_fast_low = n_fast;
        ctx.dt_low = 1.0;
        ctx.dt_high = 0.01;
        ctx.n_high_low = 100;
        ctx.t_low = t;

        /* 调度器每 10 ms 搬运一次平台状态 */
        put_state(h, 1, 0.0);

        /* 调度器每 1 s（SCADA 边界）把 RCP 新命令送入（真实 RCP 行为） */
        if ((k % 100) == 0) {
            mk_yaw_cmd(&fr, 1, 0.1, (uint64_t)(++n_cmd), t);
            ASSERT(fcr_rt_set_command(h, &fr) == 0);
        }

        st = fcr_rt_step(h, &ctx, &si);
        ASSERT(st == FCR_RT_OK);          /* 命令每 1 s 刷新 → 不超龄 */
        n_calls++;
        if (first_t < 0.0) first_t = t;
        last_t = t;

        /* 输出：仅在 SCADA 边界产生；取走后不得重复 */
        {
            int got = fcr_rt_get_output(h, &out);
            if (got == 1) {
                ASSERT(n_out < 16);
                out_t[n_out++] = out.state_sample_sim_time_s;
            }
            /* 同一时刻再取一次必须为空（无重复帧） */
            if (got == 1 && fcr_rt_get_output(h, &out) == 1) n_dup++;
        }
    }

    ASSERT(n_calls == 1000);
    ASSERT(n_cmd == 10);            /* RCP 每 1 s 一次 → 10 次 */
    ASSERT(n_out == 10);            /* 1000/100 = 10 次场控交换 */
    ASSERT(n_dup == 0);             /* 无重复帧 */
    ASSERT_NEAR(first_t, 0.01, 1e-9);
    ASSERT_NEAR(last_t, 10.00, 1e-9);
    for (k = 0; k < 10; ++k)        /* 出帧时刻 = 1.00, 2.00, ..., 10.00 */
        ASSERT_NEAR(out_t[k], (double)(k + 1), 1e-9);

    /* 最后一条命令（k=1000, seq=10）已生效 */
    ASSERT(fcr_rt_get_setpoint(h, 1, &sp) == 0);
    ASSERT(sp.external_yaw_enable == 1);
    ASSERT(sp.yaw_seq == 10);
    fcr_rt_destroy(h);
}

/* ---- A3：RT 接口版本（要求 §4.1）---- */
static void test_rt_abi_version(void)
{
    ASSERT(FCR_RT_ABI_VERSION >= 1u);
    ASSERT(fcr_rt_get_version() == FCR_RT_ABI_VERSION);
}

/* ---- 命令落地 + setpoint（S01→S08）---- */
static void test_command_to_setpoint(void)
{
    FcrRtHandle *h = fcr_rt_create();
    FcrRtConfig c = mk_cfg(2);
    FcrRtStatusInfo si;
    FarmCommandFrame fr;
    FcrRoscoExternalSetpoint sp;

    ASSERT(fcr_rt_configure(h, &c) == 0);
    ASSERT(fcr_rt_init(h) == 0);
    put_state(h, 1, 0.0);

    mk_yaw_cmd(&fr, 1, 0.1, 1, 0.0);
    ASSERT(fcr_rt_set_command(h, &fr) == 0);
    ASSERT(step_k(h, 0, &si) == FCR_RT_OK);

    ASSERT(fcr_rt_get_setpoint(h, 1, &sp) == 0);
    ASSERT(sp.external_yaw_enable == 1);
    ASSERT(sp.yaw_seq == 1);
    ASSERT_NEAR(sp.yaw_target_heading_rad, 0.1, 1e-12);
    fcr_rt_destroy(h);
}

/* ---- NaN 命令被拒绝（G-02；要求 §4.1/§14）---- */
static void test_reject_nan_command(void)
{
    FcrRtHandle *h = fcr_rt_create();
    FcrRtConfig c = mk_cfg(2);
    FcrRtStatusInfo si;
    FarmCommandFrame fr;
    FcrRoscoExternalSetpoint sp;

    ASSERT(fcr_rt_configure(h, &c) == 0);
    ASSERT(fcr_rt_init(h) == 0);
    put_state(h, 1, 0.0);

    mk_yaw_cmd(&fr, 1, 0.0 / 0.0, 1, 0.0);   /* NaN 偏航目标 */
    ASSERT(fcr_rt_set_command(h, &fr) == 0);
    ASSERT(step_k(h, 0, &si) == FCR_RT_INVALID_INPUT);
    ASSERT(si.error_source == FCR_RT_SRC_COMMAND);
    ASSERT(si.error_code == 10);
    /* 通道未被锁存（不生效） */
    ASSERT(fcr_rt_get_setpoint(h, 1, &sp) == 0);
    ASSERT(sp.external_yaw_enable == 0);

    /* 恢复合法命令后状态回到 OK 且生效 */
    mk_yaw_cmd(&fr, 1, 0.1, 2, 0.01);
    ASSERT(fcr_rt_set_command(h, &fr) == 0);
    ASSERT(step_k(h, 1, &si) == FCR_RT_OK);
    ASSERT(fcr_rt_get_setpoint(h, 1, &sp) == 0);
    ASSERT(sp.external_yaw_enable == 1);
    fcr_rt_destroy(h);
}

/* ---- TTL 到期：边沿报 TIMEOUT，随后 OK（D3 释放外部控制）---- */
static void test_ttl_timeout_edge(void)
{
    FcrRtHandle *h = fcr_rt_create();
    FcrRtConfig c = mk_cfg(2);
    FcrRtStatusInfo si;
    FarmCommandFrame fr;
    FcrRoscoExternalSetpoint sp;

    ASSERT(fcr_rt_configure(h, &c) == 0);
    ASSERT(fcr_rt_init(h) == 0);
    put_state(h, 1, 0.0);

    /* 诱导 ttl=1s，basis=0 */
    mk_ind_cmd(&fr, 1, 0.20, 1, 0.0, 1.0);
    ASSERT(fcr_rt_set_command(h, &fr) == 0);
    ASSERT(step_k(h, 0, &si) == FCR_RT_OK);
    ASSERT(fcr_rt_get_setpoint(h, 1, &sp) == 0);
    ASSERT(sp.induction_enable == 1);

    /* t=2s（k=200）超过 TTL 1s → 本步报 TIMEOUT */
    ASSERT(step_k(h, 200, &si) == FCR_RT_TIMEOUT);
    ASSERT(si.error_source == FCR_RT_SRC_COMMAND);
    ASSERT(fcr_rt_get_setpoint(h, 1, &sp) == 0);
    ASSERT(sp.induction_enable == 0);          /* D3：已释放外部控制 */

    /* 下一步不再重复报 TIMEOUT */
    ASSERT(step_k(h, 201, &si) == FCR_RT_OK);
    fcr_rt_destroy(h);
}

/* ---- Reset：run → reset → run 可重复（要求 §3.3/§13）---- */
static void test_reset_repeatable(void)
{
    FcrRtHandle *h = fcr_rt_create();
    FcrRtConfig c = mk_cfg(2);
    FcrRtStatusInfo si;
    FarmCommandFrame fr;
    FcrRoscoExternalSetpoint sp;
    FarmStateFrame out;

    ASSERT(fcr_rt_configure(h, &c) == 0);
    ASSERT(fcr_rt_init(h) == 0);
    put_state(h, 1, 0.0);

    /* 第一轮：下发并生效 */
    mk_yaw_cmd(&fr, 1, 0.1, 1, 0.0);
    ASSERT(fcr_rt_set_command(h, &fr) == 0);
    ASSERT(step_k(h, 0, &si) == FCR_RT_OK);
    ASSERT(fcr_rt_get_setpoint(h, 1, &sp) == 0);
    ASSERT(sp.external_yaw_enable == 1);

    /* reset：状态归零 */
    ASSERT(fcr_rt_reset(h) == 0);
    ASSERT(fcr_rt_get_setpoint(h, 1, &sp) == 0);
    ASSERT(sp.external_yaw_enable == 0);
    ASSERT(sp.yaw_seq == 0);
    ASSERT(sp.induction_enable == 0);
    ASSERT(fcr_rt_get_output(h, &out) == 0);

    /* 第二轮：同样的命令应再次生效（seq 从 1 重新开始） */
    put_state(h, 1, 0.0);
    mk_yaw_cmd(&fr, 1, 0.1, 1, 0.0);
    ASSERT(fcr_rt_set_command(h, &fr) == 0);
    ASSERT(step_k(h, 0, &si) == FCR_RT_OK);
    ASSERT(fcr_rt_get_setpoint(h, 1, &sp) == 0);
    ASSERT(sp.external_yaw_enable == 1);
    ASSERT(sp.yaw_seq == 1);

    /* 1 s 边界仍按 reset 后的计数器重新计时 */
    ASSERT(fcr_rt_get_output(h, &out) == 0);
    fcr_rt_destroy(h);
}

/* ---- stop 后 step → NOT_READY ---- */
static void test_stop_then_step(void)
{
    FcrRtHandle *h = fcr_rt_create();
    FcrRtConfig c = mk_cfg(1);
    FcrRtStatusInfo si;
    ASSERT(fcr_rt_configure(h, &c) == 0);
    ASSERT(fcr_rt_init(h) == 0);
    ASSERT(step_k(h, 0, &si) == FCR_RT_OK);
    ASSERT(fcr_rt_stop(h) == 0);
    ASSERT(step_k(h, 1, &si) == FCR_RT_NOT_READY);
    fcr_rt_destroy(h);
}

/* ---- A2：执行时预算与统计（要求 §5.4）---- */
static void test_step_perf_budget(void)
{
    FcrRtHandle *h = fcr_rt_create();
    FcrRtConfig c = mk_cfg(1);
    FcrRtStatusInfo si;
    FcrRtPerf perf;

    /* 预算关闭（0）：不报 WARNING，统计仍工作 */
    c.max_step_us = 0.0;
    ASSERT(fcr_rt_configure(h, &c) == 0);
    ASSERT(fcr_rt_init(h) == 0);
    put_state(h, 1, 0.0);
    ASSERT(step_k(h, 0, &si) == FCR_RT_OK);
    ASSERT(fcr_rt_get_perf(h, &perf) == 0);
    ASSERT(perf.n_steps == 1);
    ASSERT(perf.n_over_budget == 0);
    ASSERT(perf.last_us >= 0.0);
    ASSERT(perf.max_us >= perf.last_us - 1e-9);
    fcr_rt_destroy(h);

    /* 预算极小（0.0001 µs）→ 必超预算：状态 WARNING + 计数，控制输出不变 */
    h = fcr_rt_create();
    c = mk_cfg(1);
    c.max_step_us = 0.0001;
    ASSERT(fcr_rt_configure(h, &c) == 0);
    ASSERT(fcr_rt_init(h) == 0);
    put_state(h, 1, 0.0);
    ASSERT(step_k(h, 0, &si) == FCR_RT_WARNING);
    ASSERT(si.error_source == FCR_RT_SRC_INTERNAL);
    ASSERT(si.error_code == 50);
    ASSERT(fcr_rt_get_perf(h, &perf) == 0);
    ASSERT(perf.n_over_budget >= 1);
    fcr_rt_destroy(h);
}

/* ---- B3 D0：非关键数据缺失不阻断运行（AWAE 未注入 → 合法空帧）---- */
static void test_d0_awae_absent_continues(void)
{
    FcrRtHandle *h = fcr_rt_create();
    FcrRtConfig c = mk_cfg(1);
    FcrRtStatusInfo si;
    FcrAWAEObservationFrame obs;
    FcrAWAECatalogFrame cat;

    ASSERT(fcr_rt_configure(h, &c) == 0);
    ASSERT(fcr_rt_init(h) == 0);
    put_state(h, 1, 0.0);

    /* 不注入 AWAE 目录/流场：模块照常运行（D0），输出合法空帧（不伪造点） */
    ASSERT(step_k(h, 0, &si) == FCR_RT_OK);
    ASSERT(fcr_rt_get_awae(h, &obs, &cat) == 0);
    ASSERT(cat.n_points == 0);
    ASSERT(obs.n_observations == 0);
    ASSERT(cat.protocol_version == FCR_PROTOCOL_VERSION);
    fcr_rt_destroy(h);
}

/* ---- B3 D2：control_enable=0 → 回退标称（基础控制器）---- */
static void test_d2_control_disable_fallback(void)
{
    FcrRtHandle *h = fcr_rt_create();
    FcrRtConfig c = mk_cfg(2);
    FcrRtStatusInfo si;
    FarmCommandFrame fr;
    FcrRoscoExternalSetpoint sp;

    ASSERT(fcr_rt_configure(h, &c) == 0);
    ASSERT(fcr_rt_init(h) == 0);
    put_state(h, 1, 0.0);

    /* 先下发有效诱导命令 → 生效 */
    mk_ind_cmd(&fr, 1, 0.20, 1, 0.0, 30.0);
    ASSERT(fcr_rt_set_command(h, &fr) == 0);
    ASSERT(step_k(h, 0, &si) == FCR_RT_OK);
    ASSERT(fcr_rt_get_setpoint(h, 1, &sp) == 0);
    ASSERT(sp.induction_enable == 1);
    ASSERT(sp.r_pitch < 1.0);

    /* control_enable=0 → 回退标称（D2） */
    mk_ind_cmd(&fr, 1, 0.20, 2, 0.01, 30.0);
    fr.turbines[0].control_enable = 0;
    ASSERT(fcr_rt_set_command(h, &fr) == 0);
    ASSERT(step_k(h, 1, &si) == FCR_RT_OK);
    ASSERT(fcr_rt_get_setpoint(h, 1, &sp) == 0);
    ASSERT(sp.induction_enable == 0);
    ASSERT(sp.external_yaw_enable == 0);
    ASSERT(sp.farm_control_enable == 0);       /* 外部场控总使能关闭 */
    ASSERT((sp.command_status_flags & FCR_CMD_FLAG_CONTROL_DISABLED) != 0);
    fcr_rt_destroy(h);
}

/* ---- B4：Inf 命令值被拒绝（与 NaN 同策略）---- */
static void test_reject_inf_command(void)
{
    FcrRtHandle *h = fcr_rt_create();
    FcrRtConfig c = mk_cfg(2);
    FcrRtStatusInfo si;
    FarmCommandFrame fr;
    FcrRoscoExternalSetpoint sp;

    ASSERT(fcr_rt_configure(h, &c) == 0);
    ASSERT(fcr_rt_init(h) == 0);
    put_state(h, 1, 0.0);

    mk_yaw_cmd(&fr, 1, (double)INFINITY, 1, 0.0);
    ASSERT(fcr_rt_set_command(h, &fr) == 0);
    ASSERT(step_k(h, 0, &si) == FCR_RT_INVALID_INPUT);
    ASSERT(fcr_rt_get_setpoint(h, 1, &sp) == 0);
    ASSERT(sp.external_yaw_enable == 0);       /* 未锁存 */

    /* 诱导 Inf 同样拒绝 */
    mk_ind_cmd(&fr, 1, (double)INFINITY, 1, 0.0, 30.0);
    ASSERT(fcr_rt_set_command(h, &fr) == 0);
    ASSERT(step_k(h, 1, &si) == FCR_RT_INVALID_INPUT);
    ASSERT(fcr_rt_get_setpoint(h, 1, &sp) == 0);
    ASSERT(sp.induction_enable == 0);
    fcr_rt_destroy(h);
}

/* ---- B4：越界诱导（a>1/3）→ 不采纳/回退额定（D2）---- */
static void test_out_of_range_induction_fallback(void)
{
    FcrRtHandle *h = fcr_rt_create();
    FcrRtConfig c = mk_cfg(2);
    FcrRtStatusInfo si;
    FarmCommandFrame fr;
    FcrRoscoExternalSetpoint sp;

    ASSERT(fcr_rt_configure(h, &c) == 0);
    ASSERT(fcr_rt_init(h) == 0);
    put_state(h, 1, 0.0);

    /* a=0.9 远超 a_max(1/3)：物理不可达 → 回退额定（r_pitch=1）+ FALLBACK */
    mk_ind_cmd(&fr, 1, 0.9, 1, 0.0, 30.0);
    ASSERT(fcr_rt_set_command(h, &fr) == 0);
    ASSERT(step_k(h, 0, &si) == FCR_RT_OK);
    ASSERT(fcr_rt_get_setpoint(h, 1, &sp) == 0);
    ASSERT_NEAR(sp.r_pitch, 1.0, 1e-12);
    ASSERT((sp.command_status_flags & FCR_CMD_FLAG_FALLBACK_ACTIVE) != 0);
    fcr_rt_destroy(h);
}

/* ---- B4：版本/规模不匹配被拒绝（要求 §4.1）---- */
static void test_reject_version_and_size_mismatch(void)
{
    FcrRtHandle *h = fcr_rt_create();
    FcrRtConfig c = mk_cfg(2);
    FarmCommandFrame fr;

    ASSERT(fcr_rt_configure(h, &c) == 0);
    ASSERT(fcr_rt_init(h) == 0);

    /* 协议版本不匹配 */
    mk_yaw_cmd(&fr, 1, 0.1, 1, 0.0);
    fr.protocol_version = 1;                 /* 旧版本 */
    ASSERT(fcr_rt_set_command(h, &fr) == -2);

    /* n_turbines 超出配置规模 */
    mk_yaw_cmd(&fr, 1, 0.1, 1, 0.0);
    fr.n_turbines = 3;                       /* 配置仅 2 台 */
    ASSERT(fcr_rt_set_command(h, &fr) == -2);

    /* 合法帧被接受 */
    mk_yaw_cmd(&fr, 1, 0.1, 1, 0.0);
    ASSERT(fcr_rt_set_command(h, &fr) == 0);
    fcr_rt_destroy(h);
}

void test_rt_adapter_register(void)
{
    RUN_TEST("rt_configure_validation", test_configure_validation);
    RUN_TEST("rt_run_before_init", test_run_before_init);
    RUN_TEST("rt_step_time_validation", test_step_time_validation);
    RUN_TEST("rt_scada_boundary_counter", test_scada_boundary_counter);
    RUN_TEST("rt_mock_scheduler_1000_cycles", test_mock_scheduler_1000_cycles);
    RUN_TEST("rt_abi_version", test_rt_abi_version);
    RUN_TEST("rt_step_perf_budget", test_step_perf_budget);
    RUN_TEST("rt_command_to_setpoint", test_command_to_setpoint);
    RUN_TEST("rt_reject_nan_command", test_reject_nan_command);
    RUN_TEST("rt_ttl_timeout_edge", test_ttl_timeout_edge);
    RUN_TEST("rt_reset_repeatable", test_reset_repeatable);
    RUN_TEST("rt_stop_then_step", test_stop_then_step);
    RUN_TEST("rt_d0_awae_absent_continues", test_d0_awae_absent_continues);
    RUN_TEST("rt_d2_control_disable_fallback", test_d2_control_disable_fallback);
    RUN_TEST("rt_reject_inf_command", test_reject_inf_command);
    RUN_TEST("rt_out_of_range_induction_fallback", test_out_of_range_induction_fallback);
    RUN_TEST("rt_reject_version_and_size_mismatch", test_reject_version_and_size_mismatch);
}
