
#include "fcr_internal.h"
#include <stdio.h>
#include <assert.h>

int main() {
    FcrCommandStore *cs = fcr_command_store_create(3, 60.0, 5.0, 5.0, 1.0);
    FcrTurbineCommand cmd;
    FcrRoscoExternalSetpoint sp;
    double heading[3] = {0.0, 0.0, 0.0};
    
    // Simulate T2 yaw command (+10 deg = 0.1745 rad)
    memset(&cmd, 0, sizeof(cmd));
    cmd.yaw_valid = 1;
    cmd.yaw_seq = 1;
    cmd.yaw_delta_rad = 0.1745;
    cmd.yaw_ttl_s = 1.0e6;
    cmd.source_time_s = 1.0;  // > 0 for WIN-05 validation
    
    FarmCommandFrame frame;
    memset(&frame, 0, sizeof(frame));
    frame.protocol_version = FCR_PROTOCOL_VERSION;
    frame.n_turbines = 3;
    frame.turbines[1] = cmd;  // T2
    
    int rc = fcr_command_store_accept_frame(cs, &frame);
    printf("accept_frame rc=%d\n", rc);
    
    // First refresh
    fcr_command_store_refresh(cs, 0.0, heading, 1);
    fcr_command_store_get_setpoint(cs, 2, &sp);
    printf("After 1st refresh: T2 yaw_enable=%d yaw_seq=%llu yaw_valid=%d latched=%d\n",
           sp.yaw_enable, (unsigned long long)sp.yaw_seq, 
           cs->t[1].yaw_valid, cs->t[1].yaw_target_latched);
    
    // Second refresh (heading now valid)
    fcr_command_store_refresh(cs, 0.01, heading, 1);
    fcr_command_store_get_setpoint(cs, 2, &sp);
    printf("After 2nd refresh: T2 yaw_enable=%d yaw_seq=%llu yaw_valid=%d latched=%d target=%.4f\n",
           sp.yaw_enable, (unsigned long long)sp.yaw_seq,
           cs->t[1].yaw_valid, cs->t[1].yaw_target_latched, sp.yaw_target_heading_rad);
    
    // Debug counters
    uint32_t ya, ia, yl, il;
    fcr_command_store_debug_counters(&ya, &ia, &yl, &il);
    printf("Counters: yaw_accept=%u ind_accept=%u yaw_latch=%u\n", ya, ia, yl);
    
    fcr_command_store_destroy(cs);
    return 0;
}
