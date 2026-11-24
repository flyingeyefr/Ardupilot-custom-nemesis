#include "Copter.h"

bool ModeCustom::init(bool ignore_checks)
{
    if(copter.position_ok()) {
        pos_control->NE_set_max_speed_accel_m(pos_control->get_vel_estimate_NED_ms().xy().length(), BRAKE_MODE_DECEL_RATE_MSS);
        pos_control->NE_set_correction_speed_accel_m(pos_control->get_vel_estimate_NED_ms().xy().length(), BRAKE_MODE_DECEL_RATE_MSS);

        if(!pos_control->NE_is_active()) {
            pos_control->NE_init_controller();
        }
    }

    pos_control->D_set_max_speed_accel_m(BRAKE_MODE_SPEED_Z_MS, BRAKE_MODE_SPEED_Z_MS, BRAKE_MODE_DECEL_RATE_MSS);
    pos_control->D_set_correction_speed_accel_m(BRAKE_MODE_SPEED_Z_MS, BRAKE_MODE_SPEED_Z_MS, BRAKE_MODE_DECEL_RATE_MSS);

    if(!pos_control->D_is_active()) {
        pos_control->D_init_controller();
    }
    
    fs_start_time_ms = AP_HAL::millis();

    target_alt_m = pos_control->get_pos_estimate_U_m() + 30.0f;

    return true;
}

void ModeCustom::run()
{
    if(!copter.failsafe.radio){
        RC_Channel *chn10 = RC_Channels::rc_channel(9);

        if(chn10->get_radio_in() > 1200){
            copter.set_mode(Mode::Number::STABILIZE, ModeReason::RADIO_FAILSAFE_RECOVERY);
            return;
        } 
    }

    if(AP_HAL::millis() - fs_start_time_ms >= 60000) {
        copter.arming.disarm(AP_Arming::Method::TERMINATION);
        return;
    }

    if (!motors->armed()) {
        make_safe_ground_handling();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    
    float target_roll_cd = 0.0f;
    float target_pitch_cd = 0.0f;

    if (copter.position_ok()) {
        if (!pos_control->NE_is_active()) {
            pos_control->NE_init_controller();
        }
        Vector2f target_vel_xy(0.0f, 0.0f);
        Vector2f target_accel_xy(0.0f, 0.0f);
        pos_control->input_vel_accel_NE_cm(target_vel_xy, target_accel_xy);
        pos_control->NE_update_controller();

        target_roll_cd = pos_control->get_roll_cd();
        target_pitch_cd = pos_control->get_pitch_cd();
    }

    float current_alt_m = pos_control->get_pos_estimate_U_m();
    float target_climb_speed_cms = 0.0f;

    if (current_alt_m < (target_alt_m - 1.0f)) {
        target_climb_speed_cms = 150.0f; 
    } else if (current_alt_m > (target_alt_m + 1.0f)) {
        target_climb_speed_cms = -80.0f; 
    }

    pos_control->D_set_pos_target_from_climb_rate_cms(target_climb_speed_cms);
    pos_control->D_update_controller();

    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll_cd, target_pitch_cd, 0.0f);
}