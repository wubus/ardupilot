#pragma once

/// @file    AC_AttitudeControl_Sub.h
/// @brief   ArduSub attitude control library

#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsMulticopter.h>

// default angle controller PID gains
// (Sub-specific defaults for parent class)
#define AC_ATC_SUB_ANGLE_P             6.0f
#define AC_ATC_SUB_ACCEL_Y_MAX         110000.0f

// default rate controller PID gains
#define AC_ATC_SUB_RATE_RP_P           0.135f
#define AC_ATC_SUB_RATE_RP_I           0.090f
#define AC_ATC_SUB_RATE_RP_D           0.0036f
#define AC_ATC_SUB_RATE_RP_IMAX        0.444f
#define AC_ATC_SUB_RATE_RP_FILT_HZ     30.0f
#define AC_ATC_SUB_RATE_YAW_P          0.180f
#define AC_ATC_SUB_RATE_YAW_I          0.018f
#define AC_ATC_SUB_RATE_YAW_D          0.0f
#define AC_ATC_SUB_RATE_YAW_IMAX       0.222f
#define AC_ATC_SUB_RATE_YAW_FILT_HZ    5.0f

#define MAX_YAW_ERROR                  radians(5)

class AC_AttitudeControl_Sub : public AC_AttitudeControl {
public:
    AC_AttitudeControl_Sub(AP_AHRS_View &ahrs, const AP_MultiCopter &aparm, AP_MotorsMulticopter& motors);

    // empty destructor to suppress compiler warning
    virtual ~AC_AttitudeControl_Sub() {}

    // pid accessors
    AC_PID& get_rate_roll_pid() override { return _pid_rate_roll; }
    AC_PID& get_rate_pitch_pid() override { return _pid_rate_pitch; }
    AC_PID& get_rate_yaw_pid() override { return _pid_rate_yaw; }
    const AC_PID& get_rate_roll_pid() const override { return _pid_rate_roll; }
    const AC_PID& get_rate_pitch_pid() const override { return _pid_rate_pitch; }
    const AC_PID& get_rate_yaw_pid() const override { return _pid_rate_yaw; }

    // Update Alt_Hold angle maximum
    void update_althold_lean_angle_max(float throttle_in) override;

    // Set output throttle
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

    // calculate total body frame throttle required to produce the given earth frame throttle
	float get_throttle_boosted(float throttle_in);

    // set desired throttle vs attitude mixing (actual mix is slewed towards this value over 1~2 seconds)
    //  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
    //  has no effect when throttle is above hover throttle
    void set_throttle_mix_min() override { _throttle_rpy_mix_desired = _thr_mix_min; }
    void set_throttle_mix_man() override { _throttle_rpy_mix_desired = _thr_mix_man; }
    void set_throttle_mix_max(float ratio) override { _throttle_rpy_mix_desired = _thr_mix_max; }

    // are we producing min throttle?
    bool is_throttle_mix_min() const override { return (_throttle_rpy_mix < 1.25f*_thr_mix_min); }

    // run lowest level body-frame rate controller and send outputs to the motors
    void rate_controller_run(int32_t target_pitch_angle = 0, bool wing_deploy = true, uint32_t tsld = 1000) override;

    // sanity check parameters.  should be called once before take-off
    void parameter_sanity_check() override;

    // set the PID notch sample rates
    void set_notch_sample_rate(float sample_rate) override;

    // This function ensures that the ROV reaches the target orientation with the desired yaw rate
    void input_euler_angle_roll_pitch_slew_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, float slew_yaw);

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // update_throttle_rpy_mix - updates thr_low_comp value towards the target
    void update_throttle_rpy_mix();

    // get maximum value throttle can be raised to based on throttle vs attitude prioritisation
    float get_throttle_avg_max(float throttle_in);

    AP_MotorsMulticopter& _motors_multi;
    AC_PID                _pid_rate_roll;
    AC_PID                _pid_rate_pitch;
    AC_PID                _pid_rate_yaw;

    AP_Float              _thr_mix_man;     // throttle vs attitude control prioritisation used when using manual throttle (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_min;     // throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_max;     // throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
};
