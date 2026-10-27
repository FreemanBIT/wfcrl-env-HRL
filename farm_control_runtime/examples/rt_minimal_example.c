/*
 * rt_minimal_example.c — 场控模块最小调度器接入示例（WSL/Linux 与 Windows 均可编译）
 *
 * 演示《要求》§2.2/§3/§5/§6 的完整调用序列：
 *   create → configure → init → (每 10 ms) set_state/step → (每 1 s) set_command/get_output
 *   → reset → stop → destroy
 *
 * 编译（在 farm_control_runtime/ 下，与库同编）：
 *   gcc -std=gnu11 -O2 -DMB_STUB -Iinclude -Isrc -Ibridge -Iawae \
 *       -Ioffline/harness -Ioffline/provider -Itransport/modbus \
 *       examples/rt_minimal_example.c \
 *       src/angle_convention.c src/command_store.c src/state_store.c src/watchdog.c \
 *       src/runtime.c src/rosco_api.c src/signal_statistics.c src/state_aggregator.c \
 *       src/induction_supervisor.c src/yaw_action_manager.c src/bridge_runtime_adapter.c \
 *       src/fcr_rt_adapter.c bridge/fcr_bridge_core.c bridge/yaw_adapter.c \
 *       bridge/rt_snapshot.c awae/observation_manager.c awae/wind_mapper.c \
 *       awae/awae_frame_builder.c offline/provider/offline_fastfarm_provider.c \
 *       offline/harness/shm_command_source_stub.c \
 *       -lpthread -lm -o rt_minimal_example
 *
 * 或先构建静态库再链接（推荐）：
 *   cmake -S . -B build-linux && cmake --build build-linux
 *   gcc -std=gnu11 -Iinclude examples/rt_minimal_example.c \
 *       build-linux/libfarm_control_runtime.a -lpthread -lm -o rt_minimal_example
 *   ./rt_minimal_example
 */
#include <stdio.h>
#include <string.h>

#include "fcr_rt.h"

#define N_TURBINES 2
#define DT_HIGH    0.01   /* 调度器周期 10 ms */
#define DT_LOW     1.0    /* FAST.Farm 主步 1 s */
#define DT_SCADA   1.0    /* 场控交换 1 s */

static void feed_state(FcrRtHandle *h, double t)
{
    int tid;
    for (tid = 1; tid <= N_TURBINES; ++tid) {
        FcrRoscoRawState st;
        memset(&st, 0, sizeof(st));
        st.turbine_id = tid;
        st.valid = 1;
        st.source_time_s = t;
        st.nacelle_heading_rad = 0.0;   /* 例：已对准 */
        st.gen_power_w = 2.0e6;
        st.rotor_speed_rad_s = 1.2;
        st.hub_wind_speed_mps = 8.0;
        if (fcr_rt_set_state(h, tid, &st) != 0)
            printf("[warn] set_state T%d failed\n", tid);
    }
}

static void send_rcp_command(FcrRtHandle *h, int sec)
{
    FarmCommandFrame fr;
    memset(&fr, 0, sizeof(fr));
    fr.protocol_version = FCR_PROTOCOL_VERSION;
    fr.command_frame_seq = (uint64_t)sec;
    fr.n_turbines = N_TURBINES;

    /* T1：诱导 a=0.20（降载）；T2：不下诱导命令（保持标称） */
    fr.turbines[0].control_enable = 1;
    fr.turbines[0].control_mode = FCR_CONTROL_MODE_PRC;
    fr.turbines[0].command_basis_sim_time_s = (double)sec;
    fr.turbines[0].induction_valid = 1;
    fr.turbines[0].induction_seq = (uint64_t)sec;
    fr.turbines[0].induction_ref = 0.20;
    fr.turbines[0].induction_ttl_s = 5.0;

    fr.turbines[1].control_enable = 1;
    fr.turbines[1].control_mode = FCR_CONTROL_MODE_PRC;
    fr.turbines[1].command_basis_sim_time_s = (double)sec;

    if (fcr_rt_set_command(h, &fr) != 0)
        printf("[warn] set_command failed at t=%d s\n", sec);
}

int main(void)
{
    FcrRtConfig cfg;
    FcrRtHandle *h;
    FcrRtStatusInfo si;
    FcrRtPerf perf;
    FcrRoscoExternalSetpoint sp;
    int k, outputs = 0, warnings = 0;

    printf("fcr_rt ABI version = %u\n", (unsigned)fcr_rt_get_version());
    printf("FCR_RT_ABI_VERSION = %u\n", (unsigned)FCR_RT_ABI_VERSION);

    /* ---------- 1) 配置 ---------- */
    memset(&cfg, 0, sizeof(cfg));
    cfg.n_turbines = N_TURBINES;
    cfg.dt_high_s = DT_HIGH;
    cfg.dt_low_s = DT_LOW;
    cfg.dt_scada_s = DT_SCADA;
    cfg.default_yaw_ttl_s = 60.0;
    cfg.default_induction_ttl_s = 5.0;
    cfg.stale_threshold_s = 5.0;
    cfg.flow_stale_threshold_s = 3.0;
    cfg.bridge_enable = 0;              /* 0=legacy 诱导监督器；1=bridge core */
    cfg.bridge_induction_mode = 1;
    cfg.max_step_us = 1000.0;           /* 10 ms 周期内的执行时预算 */

    /* ---------- 2) 生命周期 ---------- */
    h = fcr_rt_create();
    if (!h) { printf("[FAIL] create\n"); return 1; }
    if (fcr_rt_configure(h, &cfg) != 0) { printf("[FAIL] configure\n"); fcr_rt_destroy(h); return 1; }
    if (fcr_rt_init(h) != 0) { printf("[FAIL] init\n"); fcr_rt_destroy(h); return 1; }

    /* ---------- 3) 主循环：1000 × 10 ms（10 s）---------- */
    for (k = 1; k <= 1000; ++k) {
        FcrRtTimeCtx t;
        FcrRtStatus st;
        FarmStateFrame out;

        memset(&t, 0, sizeof(t));
        t.n_low = k / 100;              /* DT_Low 步号（权威） */
        t.n_fast_low = k % 100;         /* 本 DT_Low 内的 10 ms 子步 */
        t.dt_low = DT_LOW;
        t.dt_high = DT_HIGH;
        t.n_high_low = 100;
        t.t_low = (double)k * DT_HIGH;

        /* 调度器搬运平台状态（每 10 ms） */
        feed_state(h, t.t_low);

        /* 调度器在 1 s 边界送入 RCP 命令 */
        if ((k % 100) == 0) send_rcp_command(h, k / 100);

        /* 每 10 ms 调用一次；返回值用于快速判断，si 含详情 */
        memset(&si, 0, sizeof(si));
        st = fcr_rt_step(h, &t, &si);
        if (st != FCR_RT_OK) {
            ++warnings;
            if (warnings <= 5)
                printf("[%s] t=%.2f src=%u code=%d msg=%s\n",
                       fcr_rt_status_str(st), t.t_low, si.error_source,
                       (int)si.error_code, si.message);
        }

        /* 调度器在 1 s 边界取走状态帧（发给 RCP） */
        if (fcr_rt_get_output(h, &out) == 1) {
            ++outputs;
            printf("[output] t=%.2f s seq=%llu n_turbines=%u\n",
                   out.state_sample_sim_time_s,
                   (unsigned long long)out.state_frame_seq, out.n_turbines);
        }

        if (st == FCR_RT_FATAL) break;
    }

    /* ---------- 4) 读取 setpoint（交给单机控制/ROSCO）---------- */
    if (fcr_rt_get_setpoint(h, 1, &sp) == 0) {
        printf("T1 setpoint: induction_enable=%u seq=%llu r_pitch=%.4f r_speed=%.4f\n",
               sp.induction_enable, (unsigned long long)sp.induction_seq,
               sp.r_pitch, sp.r_speed);
    }

    /* ---------- 5) 执行时统计 ---------- */
    if (fcr_rt_get_perf(h, &perf) == 0) {
        printf("perf: steps=%llu last=%.1f us max=%.1f us avg=%.1f us over_budget=%llu\n",
               (unsigned long long)perf.n_steps, perf.last_us, perf.max_us,
               perf.avg_us, (unsigned long long)perf.n_over_budget);
    }
    printf("summary: 1000 steps, %d SCADA frames, %d warnings\n", outputs, warnings);

    /* ---------- 6) reset 可重复性（要求 §3.3）---------- */
    if (fcr_rt_reset(h) != 0) printf("[warn] reset failed\n");
    else {
        FcrRoscoExternalSetpoint sp2;
        fcr_rt_get_setpoint(h, 1, &sp2);
        printf("after reset: induction_enable=%u seq=%llu (期望 0/0)\n",
               sp2.induction_enable, (unsigned long long)sp2.induction_seq);
    }

    /* ---------- 7) 收尾 ---------- */
    fcr_rt_stop(h);
    fcr_rt_destroy(h);
    printf("done\n");
    return 0;
}
