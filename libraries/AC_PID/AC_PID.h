#pragma once

/// @file	AC_PID.h
/// @brief	Generic PID algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include <Filter/SlewLimiter.h>
#include <Filter/NotchFilter.h>
#include <Filter/AP_Filter.h>

#define AC_PID_TFILT_HZ_DEFAULT  0.0f   // default input filter frequency
#define AC_PID_EFILT_HZ_DEFAULT  0.0f   // default input filter frequency
#define AC_PID_DFILT_HZ_DEFAULT  20.0f   // default input filter frequency
#define AC_PID_RESET_TC          0.16f   // Time constant for integrator reset decay to zero

#include "AP_PIDInfo.h"

/// @class	AC_PID
/// @brief	Copter PID control class
class AC_PID {
public:

    struct Defaults {
        float p;
        float i;
        float d;
        float ff;
        float imax;
        float filt_T_hz;
        float filt_E_hz;
        float filt_D_hz;
        float srmax;
        float srtau;
        float dff;
        float p_fw;
        float i_fw;
        float d_fw;
    };

    // Constructor for PID
    AC_PID(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz,
           float initial_srmax=0, float initial_srtau=1.0, float initial_dff=0, float initial_p_fw=0.13, float initial_i_fw=0.0, float initial_d_fw=0.0, float initial_p_wf=0.13, float initial_i_wf=0.0, float initial_d_wf=0.0);
    AC_PID(const AC_PID::Defaults &defaults) :
        AC_PID(
            defaults.p,
            defaults.i,
            defaults.d,
            defaults.ff,
            defaults.imax,
            defaults.filt_T_hz,
            defaults.filt_E_hz,
            defaults.filt_D_hz,
            defaults.srmax,
            defaults.srtau,
            defaults.dff,
            defaults.p,
            defaults.i,
            defaults.d
            )
        { }

    CLASS_NO_COPY(AC_PID);

    //  update_all - set target and measured inputs to PID controller and calculate outputs
    //  target and error are filtered
    //  the derivative is then calculated and filtered
    //  the integral is then updated based on the setting of the limit flag
    float update_all(float target, float measurement, float dt, bool limit = false, float boost = 1.0f, int32_t pitch_angle_target = 0, bool wing_deploy = true, uint32_t tsld = 1000);

    //  update_error - set error input to PID controller and calculate outputs
    //  target is set to zero and error is set and filtered
    //  the derivative then is calculated and filtered
    //  the integral is then updated based on the setting of the limit flag
    //  Target and Measured must be set manually for logging purposes.
    // todo: remove function when it is no longer used.
    float update_error(float error, float dt, bool limit = false);

    // get_pid - get results from pid controller
    float get_p() const;
    float get_i() const;
    float get_d() const;
    float get_ff() const;

    float update_p_gain(int32_t pitch_angle_target, bool wing_deploy, uint32_t tsld); // function for implementing gain scheduling. I will need pilot pitch input as an argument probably?
    float update_d_gain(int32_t pitch_angle_target, bool wing_deploy, uint32_t tsld);
    float update_i_gain(int32_t pitch_angle_target, bool wing_deploy, uint32_t tsld);

    // reset_I - reset the integrator
    void reset_I();

    // reset_filter - input filter will be reset to the next value provided to set_input()
    void reset_filter() {
        _flags._reset_filter = true;
    }

    // load gain from eeprom
    void load_gains();

    // save gain to eeprom
    void save_gains();

    /// operator function call for easy initialisation
    void operator()(float p_val, float i_val, float d_val, float ff_val, float imax_val, float input_filt_T_hz, float input_filt_E_hz, float input_filt_D_hz, float dff_val=0);

    // get accessors
    const AP_Float &kP() const { return _kp; }
    AP_Float &kP() { return _kp; }
    AP_Float &kI() { return _ki; }
    AP_Float &kD() { return _kd; }
    AP_Float &kIMAX() { return _kimax; }
    AP_Float &kPDMAX() { return _kpdmax; }
    AP_Float &ff() { return _kff;}
    AP_Float &filt_T_hz() { return _filt_T_hz; }
    AP_Float &filt_E_hz() { return _filt_E_hz; }
    AP_Float &filt_D_hz() { return _filt_D_hz; }
    AP_Float &slew_limit() { return _slew_rate_max; }
    AP_Float &kDff() { return _kdff; }
    AP_Float &kP_fw() { return _kp_fw; }
    AP_Float &kI_fw() { return _ki_fw; }
    AP_Float &kD_fw() { return _kd_fw; }
    AP_Float &kP_wf() { return _kp_wf; }
    AP_Float &kI_wf() { return _ki_wf; }
    AP_Float &kD_wf() { return _kd_wf; }

    float imax() const { return _kimax.get(); }
    float pdmax() const { return _kpdmax.get(); }

    float get_filt_T_alpha(float dt) const;
    float get_filt_E_alpha(float dt) const;
    float get_filt_D_alpha(float dt) const;

    // set accessors
    void kP(const float v) { _kp.set(v); }
    void kI(const float v) { _ki.set(v); }
    void kD(const float v) { _kd.set(v); }
    void ff(const float v) { _kff.set(v); }
    void imax(const float v) { _kimax.set(fabsf(v)); }
    void pdmax(const float v) { _kpdmax.set(fabsf(v)); }
    void filt_T_hz(const float v);
    void filt_E_hz(const float v);
    void filt_D_hz(const float v);
    void slew_limit(const float v);
    void kDff(const float v) { _kdff.set(v); }
    void kP_fw(const float v) { _kp_fw.set(v); }
    void kI_fw(const float v) { _ki_fw.set(v); }
    void kD_fw(const float v) { _kd_fw.set(v); }

    // set the desired and actual rates (for logging purposes)
    void set_target_rate(float target) { _pid_info.target = target; }
    void set_actual_rate(float actual) { _pid_info.actual = actual; }

    // integrator setting functions
    void set_integrator(float i);
    void relax_integrator(float integrator, float dt, float time_constant);

    // set slew limiter scale factor
    void set_slew_limit_scale(int8_t scale) { _slew_limit_scale = scale; }

    // return current slew rate of slew limiter. Will return 0 if SMAX is zero
    float get_slew_rate(void) const { return _slew_limiter.get_slew_rate(); }

    const AP_PIDInfo& get_pid_info(void) const { return _pid_info; }

    void set_notch_sample_rate(float);

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

protected:

    //  update_i - update the integral
    //  if the limit flag is set the integral is only allowed to shrink
    void update_i(float dt, bool limit, int32_t pitch_angle_target = 0, bool wing_fold = false, uint32_t tsld = 1000);

    // parameters
    AP_Float _kp;
    AP_Float _ki;
    AP_Float _kd;
    AP_Float _kff;
    AP_Float _kimax;
    AP_Float _kpdmax;
    AP_Float _filt_T_hz;         // PID target filter frequency in Hz
    AP_Float _filt_E_hz;         // PID error filter frequency in Hz
    AP_Float _filt_D_hz;         // PID derivative filter frequency in Hz
    AP_Float _slew_rate_max;
    AP_Float _kdff;
    AP_Float _kp_fw;
    AP_Float _ki_fw;
    AP_Float _kd_fw;
    AP_Float _kp_wf;
    AP_Float _kd_wf;
    AP_Float _ki_wf;

#if AP_FILTER_ENABLED
    AP_Int8 _notch_T_filter;
    AP_Int8 _notch_E_filter;
#endif

    // the time constant tau is not currently configurable, but is set
    // as an AP_Float to make it easy to make it configurable for a
    // single user of AC_PID by adding the parameter in the param
    // table of the parent class. It is made public for this reason
    AP_Float _slew_rate_tau;

    SlewLimiter _slew_limiter{_slew_rate_max, _slew_rate_tau};

    // flags
    struct ac_pid_flags {
        bool _reset_filter :1; // true when input filter should be reset during next call to set_input
        bool _I_set :1; // true if if the I terms has been set externally including zeroing
    } _flags;

    // internal variables
    float _integrator;        // integrator value
    float _target;            // target value to enable filtering
    float _error;             // error value to enable filtering
    float _derivative;        // derivative value to enable filtering
    int8_t _slew_limit_scale;
    float _target_derivative; // target derivative value to enable dff
#if AP_FILTER_ENABLED
    NotchFilterFloat* _target_notch;
    NotchFilterFloat* _error_notch;
#endif

    AP_PIDInfo _pid_info;

private:
    const float default_kp;
    const float default_ki;
    const float default_kd;
    const float default_kff;
    const float default_kdff;
    const float default_kimax;
    const float default_filt_T_hz;
    const float default_filt_E_hz;
    const float default_filt_D_hz;
    const float default_slew_rate_max;
    const float default_kp_fw;
    const float default_ki_fw;
    const float default_kd_fw;
    const float default_kp_wf;
    const float default_ki_wf;
    const float default_kd_wf;
};
