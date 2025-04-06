/// @file	AC_P.cpp
/// @brief	Generic P algorithm

#include <AP_Math/AP_Math.h>
#include "AC_P.h"

const AP_Param::GroupInfo AC_P::var_info[] = {
    // @Param: P
    // @DisplayName: PI Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P",    0, AC_P, _kp, default_kp),
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P_FW",    1, AC_P, _kp_fw, default_kp_fw),
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P_WF",    2, AC_P, _kp_wf, default_kp_wf),
    AP_GROUPEND
};

float AC_P::get_p(float error) const
{
    return (float)error * _kp;
}

void AC_P::load_gains()
{
    _kp.load();
}

void AC_P::save_gains()
{
    _kp.save();
}

float AC_P::update_p_gain(int32_t pat, bool wd, uint32_t tsld) // pat = pitch angle target
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