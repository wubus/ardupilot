#pragma once

/// @file	AC_PD.h
/// @brief	Generic P controller with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>

/// @class	AC_P
/// @brief	Object managing one P controller
class AC_P {
public:

    /// Constructor for P that saves its settings to EEPROM
    ///
    /// @note	PIs must be named to avoid either multiple parameters with the
    ///			same name, or an overly complex constructor.
    ///
    /// @param  initial_p       Initial value for the P term.
    ///
    AC_P(const float &initial_p = 0.0f) :
        default_kp(initial_p),
        default_kp_fw(initial_p),
        default_kp_wf(initial_p)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    CLASS_NO_COPY(AC_P);

    /// Iterate the P controller, return the new control value
    ///
    /// Positive error produces positive output.
    ///
    /// @param error	The measured error value
    /// @param dt		The time delta in milliseconds (note
    ///					that update interval cannot be more
    ///					than 65.535 seconds due to limited range
    ///					of the data type).
    ///
    /// @returns		The updated control output.
    ///
    float       get_p(float error) const;
    float       update_p_gain(int32_t pat, bool wd, uint32_t tsld);

    /// Load gain properties
    ///
    void        load_gains();

    /// Save gain properties
    ///
    void        save_gains();

    /// @name	parameter accessors
    //@{

    /// Overload the function call operator to permit relatively easy initialisation
    void operator() (const float p) { _kp.set(p); }

    // accessors
    AP_Float    &kP() { return _kp; }
    const AP_Float &kP() const { return _kp; }
    AP_Float    &kP_fw() { return _kp_fw; }
    void        kP(const float v) { _kp.set(v); }
    void        kP_fw(const float v) {_kp_fw.set(v);}

    static const struct AP_Param::GroupInfo        var_info[];

private:
    AP_Float        _kp;
    AP_Float        _kp_fw;
    AP_Float        _kp_wf;
    const float default_kp;
    const float default_kp_fw;
    const float default_kp_wf;
};
