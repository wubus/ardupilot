/// @file	AC_PID.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PID.h"

#define AC_PID_DEFAULT_NOTCH_ATTENUATION 40

const AP_Param::GroupInfo AC_PID::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P", 0, AC_PID, _kp, default_kp),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("I", 1, AC_PID, _ki, default_ki),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D", 2, AC_PID, _kd, default_kd),

    // 3 was for uint16 IMAX

    // @Param: FF
    // @DisplayName: FF FeedForward Gain
    // @Description: FF Gain which produces an output value that is proportional to the demanded input
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FF", 4, AC_PID, _kff, default_kff),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("IMAX", 5, AC_PID, _kimax, default_kimax),

    // 6 was for float FILT

    // 7 is for float ILMI and FF

    // index 8 was for AFF

    // @Param: FLTT
    // @DisplayName: PID Target filter frequency in Hz
    // @Description: Target filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTT", 9, AC_PID, _filt_T_hz, default_filt_T_hz),

    // @Param: FLTE
    // @DisplayName: PID Error filter frequency in Hz
    // @Description: Error filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTE", 10, AC_PID, _filt_E_hz, default_filt_E_hz),

    // @Param: FLTD
    // @DisplayName: PID Derivative term filter frequency in Hz
    // @Description: Derivative filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTD", 11, AC_PID, _filt_D_hz, default_filt_D_hz),

    // @Param: SMAX
    // @DisplayName: Slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("SMAX", 12, AC_PID, _slew_rate_max, default_slew_rate_max),

    // @Param: PDMX
    // @DisplayName: PD sum maximum
    // @Description: The maximum/minimum value that the sum of the P and D term can output
    // @User: Advanced
    AP_GROUPINFO("PDMX", 13, AC_PID, _kpdmax, 0),

    // @Param: D_FF
    // @DisplayName: PID Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D_FF", 14, AC_PID, _kdff, default_kdff),

#if AP_FILTER_ENABLED
    // @Param: NTF
    // @DisplayName: PID Target notch filter index
    // @Description: PID Target notch filter index
    // @Range: 1 8
    // @User: Advanced
    AP_GROUPINFO("NTF", 15, AC_PID, _notch_T_filter, 0),

    // @Param: NEF
    // @DisplayName: PID Error notch filter index
    // @Description: PID Error notch filter index
    // @Range: 1 8
    // @User: Advanced
    AP_GROUPINFO("NEF", 16, AC_PID, _notch_E_filter, 0),
#endif

    // @Param: P_FW
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P_FW", 17, AC_PID, _kp_fw, default_kp),

    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("I_FW", 18, AC_PID, _ki_fw, default_ki),

    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D_FW", 19, AC_PID, _kd_fw, default_kd),

    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P_WF", 20, AC_PID, _kp_wf, default_kp),

    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("I_WF", 21, AC_PID, _ki_wf, default_ki),

    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D_WF", 22, AC_PID, _kd_wf, default_kd),

    AP_GROUPEND
};

// Constructor
AC_PID::AC_PID(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz,
               float initial_srmax, float initial_srtau, float initial_dff, float initial_p_fw, float initial_i_fw, float initial_d_fw, float initial_p_wf, float initial_i_wf, float initial_d_wf) :
    default_kp(initial_p),
    default_ki(initial_i),
    default_kd(initial_d),
    default_kff(initial_ff),
    default_kimax(initial_imax),
    default_filt_T_hz(initial_filt_T_hz),
    default_filt_E_hz(initial_filt_E_hz),
    default_filt_D_hz(initial_filt_D_hz),
    default_slew_rate_max(initial_srmax),
    default_kdff(initial_dff),
    default_kp_fw(initial_p),
    default_ki_fw(initial_i),
    default_kd_fw(initial_d),
    default_kp_wf(initial_p),
    default_ki_wf(initial_i),
    default_kd_wf(initial_d)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    // this param is not in the table, so its default is no loaded in the call above
    _slew_rate_tau.set(initial_srtau);

    // reset input filter to first value received
    _flags._reset_filter = true;

    memset(&_pid_info, 0, sizeof(_pid_info));

    // slew limit scaler allows for plane to use degrees/sec slew
    // limit
    _slew_limit_scale = 1;
}

// filt_T_hz - set target filter hz
void AC_PID::filt_T_hz(float hz)
{
    _filt_T_hz.set(fabsf(hz));
}

// filt_E_hz - set error filter hz
void AC_PID::filt_E_hz(float hz)
{
    _filt_E_hz.set(fabsf(hz));
}

// filt_D_hz - set derivative filter hz
void AC_PID::filt_D_hz(float hz)
{
    _filt_D_hz.set(fabsf(hz));
}

// slew_limit - set slew limit
void AC_PID::slew_limit(float smax)
{
    _slew_rate_max.set(fabsf(smax));
}

void AC_PID::set_notch_sample_rate(float sample_rate)
{
#if AP_FILTER_ENABLED
    if (_notch_T_filter == 0 && _notch_E_filter == 0) {
        return;
    }

    if (_notch_T_filter != 0) {
        if (_target_notch == nullptr) {
            _target_notch = new NotchFilterFloat();
        }
        AP_Filter* filter = AP::filters().get_filter(_notch_T_filter);
        if (filter != nullptr && !filter->setup_notch_filter(*_target_notch, sample_rate)) {
            delete _target_notch;
            _target_notch = nullptr;
            _notch_T_filter.set(0);
        }
    }

    if (_notch_E_filter != 0) {
        if (_error_notch == nullptr) {
            _error_notch = new NotchFilterFloat();
        }
        AP_Filter* filter = AP::filters().get_filter(_notch_E_filter);
        if (filter != nullptr && !filter->setup_notch_filter(*_error_notch, sample_rate)) {
            delete _error_notch;
            _error_notch = nullptr;
            _notch_E_filter.set(0);
        }
    }
#endif
}

//  update_all - set target and measured inputs to PID controller and calculate outputs
//  target and error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated based on the setting of the limit flag
float AC_PID::update_all(float target, float measurement, float dt, bool limit, float boost, int32_t pitch_angle_target, bool wing_deploy, uint32_t tsld)
{
    // don't process inf or NaN
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    // reset input filter to value received
    _pid_info.reset = _flags._reset_filter;
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _target = target;
        _error = _target - measurement;
        _derivative = 0.0f;
        _target_derivative = 0.0f;
#if AP_FILTER_ENABLED
        if (_target_notch != nullptr) {
            _target_notch->reset();
            _target = _target_notch->apply(_target);
        }
        if (_error_notch != nullptr) {
            _error_notch->reset();
            _error = _error_notch->apply(_error);
        }
#endif
    } else {
        float error_last = _error;
        float target_last = _target;
        float error = _target - measurement;
#if AP_FILTER_ENABLED
        // apply notch filters before FTLD/FLTE to avoid shot noise
        if (_target_notch != nullptr) {
            target = _target_notch->apply(target);
        }
        if (_error_notch != nullptr) {
            error = _error_notch->apply(error);
        }
#endif
        _target += get_filt_T_alpha(dt) * (target - _target);
        _error += get_filt_E_alpha(dt) * (error - _error);

        // calculate and filter derivative
        if (is_positive(dt)) {
            float derivative = (_error - error_last) / dt;
            _derivative += get_filt_D_alpha(dt) * (derivative - _derivative);
            _target_derivative = (_target - target_last) / dt;
        }
    }

    // update I term
    update_i(dt, limit, pitch_angle_target);

    float P_out = (_error * update_p_gain(pitch_angle_target, wing_deploy, tsld));
    float D_out = (_derivative * update_d_gain(pitch_angle_target, wing_deploy, tsld));

    // calculate slew limit modifier for P+D
    _pid_info.Dmod = _slew_limiter.modifier((_pid_info.P + _pid_info.D) * _slew_limit_scale, dt);
    _pid_info.slew_rate = _slew_limiter.get_slew_rate();

    P_out *= _pid_info.Dmod;
    D_out *= _pid_info.Dmod;

    // boost output if required
    P_out *= boost;
    D_out *= boost;

    _pid_info.PD_limit = false;
    // Apply PD sum limit if enabled
    if (is_positive(_kpdmax)) {
        const float PD_sum_abs = fabsf(P_out + D_out);
        if (PD_sum_abs > _kpdmax) {
            const float PD_scale = _kpdmax / PD_sum_abs;
            P_out *= PD_scale;
            D_out *= PD_scale;
            _pid_info.PD_limit = true;
        }
    }

    _pid_info.target = _target;
    _pid_info.actual = measurement;
    _pid_info.error = _error;
    _pid_info.P = P_out;
    _pid_info.D = D_out;
    _pid_info.FF = _target * _kff;
    _pid_info.DFF = _target_derivative * _kdff;

    return P_out + D_out + _integrator;
}

//  update_error - set error input to PID controller and calculate outputs
//  target is set to zero and error is set and filtered
//  the derivative then is calculated and filtered
//  the integral is then updated based on the setting of the limit flag
//  Target and Measured must be set manually for logging purposes.
// todo: remove function when it is no longer used.
float AC_PID::update_error(float error, float dt, bool limit)
{
    // don't process inf or NaN
    if (!isfinite(error)) {
        return 0.0f;
    }

    // Reuse update all code path, zero target and pass negative error as measurement
    // Passing as measurement bypasses any target filtering to maintain behaviour
    // Negate as update all calculates error as target - measurement
    _target = 0.0;
    const float output = update_all(0.0, -error, dt, limit);

    // Make sure logged target and actual are still 0 to maintain behaviour
    _pid_info.target = 0.0;
    _pid_info.actual = 0.0;

    return output;
}

//  update_i - update the integral
//  If the limit flag is set the integral is only allowed to shrink
void AC_PID::update_i(float dt, bool limit, int32_t pitch_angle_target, bool wing_fold, uint32_t tsld)
{
    if (!is_zero(_ki) && is_positive(dt)) {
        // Ensure that integrator can only be reduced if the output is saturated
        if (!limit || ((is_positive(_integrator) && is_negative(_error)) || (is_negative(_integrator) && is_positive(_error)))) {
            _integrator += ((float)_error * update_i_gain(pitch_angle_target, wing_fold, tsld)) * dt;
            _integrator = constrain_float(_integrator, -_kimax, _kimax);
        }
    } else {
        _integrator = 0.0f;
    }
    _pid_info.I = _integrator;
    _pid_info.limit = limit;

    // Set I set flag for logging and clear
    _pid_info.I_term_set = _flags._I_set;
    _flags._I_set = false;
}

float AC_PID::update_p_gain(int32_t pat, bool wd, uint32_t tsld) // pat = pitch angle target
{
    float kp_h;
    float t_ = tsld / 1000.0f;
    float k_scalar = constrain_float(t_, 0, 1);

    if (wd) {
        kp_h = k_scalar*_kp + (1-k_scalar)*_kp_wf;
    }
    else {
        kp_h = k_scalar*_kp_wf + (1-k_scalar)*_kp;
    }
    if (abs(pat) <= 2500) {
        return kp_h;
    }
    else if (abs(pat) >= 7000) {
        return _kp_fw;
    }
    else {
        return kp_h + (_kp_fw - kp_h) * (abs(pat) - 2500) / 4500;
    }
}

float AC_PID::update_d_gain(int32_t pat, bool wd, uint32_t tsld) // pat = pitch angle target
{
    float kd_h;
    float t_ = tsld / 1000.0f;
    float k_scalar = constrain_float(t_, 0, 1);

    if (wd) {
        kd_h = k_scalar*_kd + (1-k_scalar)*_kd_wf;
    }
    else {
        kd_h = k_scalar*_kd_wf + (1-k_scalar)*_kd;
    }
    if (abs(pat) <= 2500) {
        return kd_h;
    }
    else if (abs(pat) >= 7000) {
        return _kd_fw;
    }
    else {
        return kd_h + (_kd_fw - kd_h) * (abs(pat) - 2500) / 4500;
    }
}

float AC_PID::update_i_gain(int32_t pat, bool wd, uint32_t tsld) // pat = pitch angle target
{
    float ki_h;
    float t_ = tsld / 1000.0f;
    float k_scalar = constrain_float(t_, 0, 1);

    if (wd) {
        ki_h = k_scalar*_ki + (1-k_scalar)*_ki_wf;
    }
    else {
        ki_h = k_scalar*_ki_wf + (1-k_scalar)*_ki;
    }
    if (abs(pat) <= 2500) {
        return ki_h;
    }
    else if (abs(pat) >= 7000) {
        return _ki_fw;
    }
    else {
        return ki_h + (_ki_fw - ki_h) * (abs(pat) - 2500) / 4500;
    }
}

float AC_PID::get_p() const
{
    return _pid_info.P;
}

float AC_PID::get_i() const
{
    return _integrator;
}

float AC_PID::get_d() const
{
    return _pid_info.D;
}

float AC_PID::get_ff() const
{
    return  _pid_info.FF + _pid_info.DFF;
}

void AC_PID::reset_I()
{
    _flags._I_set = true;
    _integrator = 0.0;
}

// load original gains from eeprom, used by autotune to restore gains after tuning
void AC_PID::load_gains()
{
    _kp.load();
    _ki.load();
    _kd.load();
    _kff.load();
    _filt_T_hz.load();
    _filt_E_hz.load();
    _filt_D_hz.load();
}

// save original gains to eeprom, used by autotune to save gains before tuning
void AC_PID::save_gains()
{
    _kp.save();
    _ki.save();
    _kd.save();
    _kff.save();
    _filt_T_hz.save();
    _filt_E_hz.save();
    _filt_D_hz.save();
}

/// Overload the function call operator to permit easy initialisation
void AC_PID::operator()(float p_val, float i_val, float d_val, float ff_val, float imax_val, float input_filt_T_hz, float input_filt_E_hz, float input_filt_D_hz, float dff_val)
{
    _kp.set(p_val);
    _ki.set(i_val);
    _kd.set(d_val);
    _kff.set(ff_val);
    _kimax.set(fabsf(imax_val));
    _filt_T_hz.set(input_filt_T_hz);
    _filt_E_hz.set(input_filt_E_hz);
    _filt_D_hz.set(input_filt_D_hz);
    _kdff.set(dff_val);
}

// get_filt_T_alpha - get the target filter alpha
float AC_PID::get_filt_T_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_T_hz);
}

// get_filt_E_alpha - get the error filter alpha
float AC_PID::get_filt_E_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_E_hz);
}

// get_filt_D_alpha - get the derivative filter alpha
float AC_PID::get_filt_D_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_D_hz);
}

void AC_PID::set_integrator(float integrator)
{
    _flags._I_set = true;
    _integrator = constrain_float(integrator, -_kimax, _kimax);
}

void AC_PID::relax_integrator(float integrator, float dt, float time_constant)
{
    integrator = constrain_float(integrator, -_kimax, _kimax);
    if (is_positive(dt)) {
        _flags._I_set = true;
        _integrator = _integrator + (integrator - _integrator) * (dt / (dt + time_constant));
    }
}
