#include "mode.h"
#include "Plane.h"

#if HAL_QUADPLANE_ENABLED

void ModeLaunch::update()
{
    plane.nav_roll_cd = 0;
    // const float pitch_input = (float)plane.channel_pitch->get_control_in() / plane.channel_pitch->get_range();
    switch (phase) {
        case LaunchPhase::Disarmed:
        case LaunchPhase::LaunchDetected:  // should nominally be with default
        case LaunchPhase::PreLaunch:  // should nominally be with default
        default:
            plane.nav_pitch_cd = constrain_float(ahrs.pitch_sensor,-8500,8500);
            break;            
        case LaunchPhase::Stabilized:
            plane.nav_pitch_cd = constrain_float(ahrs.pitch_sensor,-8500,8500);
            break;
        case LaunchPhase::Flare:
            plane.nav_pitch_cd = constrain_float(ahrs.pitch_sensor,-8500,8500);
            break;
        case LaunchPhase::WingsDeploy:
            plane.nav_pitch_cd = constrain_float(wd_initial_pitch - (AP_HAL::millis() - wd_time) * wd_pitch_rate, -4500, 4500);
            break;
        case LaunchPhase::Switch:
            plane.nav_pitch_cd = 300.0f; // 3 degrees nose up
            break;
    }
}

void ModeLaunch::run()
{
    
    // logic for what the aircraft should be doing in each phase of launch

    if (!plane.arming.is_armed()) {
        phase = LaunchPhase::Disarmed;
    }

    else if (phase == LaunchPhase::Disarmed && plane.arming.is_armed()) {    // armed yet? if yes, proceed in PreLaunch
        gcs().send_text(MAV_SEVERITY_INFO,"Launch mode: PreLaunch");
        phase = LaunchPhase::PreLaunch;
    }

    else if (phase == LaunchPhase::PreLaunch && launch_detected()) {         // launch detected yet? if yes, proceed in LaunchDetected
        gcs().send_text(MAV_SEVERITY_INFO,"Launch mode: LaunchDetected");
        launch_time = AP_HAL::millis();
        phase = LaunchPhase::LaunchDetected;
    }

    else if (phase == LaunchPhase::LaunchDetected) {                         // wait for .1 seconds after launch detection, then switch to Stabilized
        // wait for stabilization
        if (AP_HAL::millis() - launch_time > 480) { // 550
            gcs().send_text(MAV_SEVERITY_INFO,"Launch mode: Flare");
            phase = LaunchPhase::Flare;
        }
    }

    else if (phase == LaunchPhase::Stabilized) {                             // wait for 1 seconds of stabilization, then flare
        // wait for flare
        if (AP_HAL::millis() - launch_time > 480) {   // 1325
            gcs().send_text(MAV_SEVERITY_INFO,"Launch mode: Flare");
            phase = LaunchPhase::Flare;
        }
    }
    
    else if (phase == LaunchPhase::Flare) {                          // give flare .225 seconds, then deploy wings
        // give brief time to flare
        if (AP_HAL::millis() - launch_time > 580) {  // 1575
            gcs().send_text(MAV_SEVERITY_INFO,"Launch mode: WingsDeploy");
            phase = LaunchPhase::WingsDeploy;
            plane.wing_deploy = true;
            plane.wing_deploy_start = AP_HAL::millis();
            plane.millis_since_wing_deploy = 0;
            wd_initial_pitch = constrain_float(ahrs.pitch_sensor,-8500,8500);
            wd_time = AP_HAL::millis();
            wd_pitch_rate = (wd_initial_pitch - 300.0f) / 900.0f;  // place 2
        }
    }

    else if (phase == LaunchPhase::WingsDeploy) { // it takes about 900 ms to fully deploy wings (safe estimate)
        // wait for switch to next mode             // 0.5 + 1.0 + 0.3 + 0.90 = 2.75 seconds
        if (AP_HAL::millis() - launch_time > 1480) { // 2475 THERE ARE 2 PLACES WHERE THIS TIME IS USED - MAKE SURE THEY ARE CONSISTENT 2970
            gcs().send_text(MAV_SEVERITY_INFO,"Launch mode: Switch");        // once wings are deployed, time to switch to FBWA
            phase = LaunchPhase::Switch;
            // switch to FBWA
        }
    }

    else if (phase == LaunchPhase::Switch) {
        // switch to FBWA
        plane.set_mode(plane.mode_fbwa, ModeReason::LAUNCH_COMPLETED);
        gcs().send_text(MAV_SEVERITY_INFO,"Launch complete, switching to FBWA");
        phase = LaunchPhase::Disarmed; // reset phase
    }


    // running controllers based on phase of launch

    switch (phase) {
        case LaunchPhase::Disarmed:
            quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            break;
        case LaunchPhase::PreLaunch:
            // quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED); // changed this temporarily for 1dof test
            // quadplane.hold_stabilize(0.5f);     // THIS SHOULD NOT BE THERE IN THE FINAL VERSION !!!!!
            quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
            attitude_control->set_throttle_out(0, false, 0);
            // quadplane.relax_attitude_control();
            // plane.stabilize_pitch();            // THIS SHOULD NOT BE THERE IN THE FINAL VERSION !!!!!
            // SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 1908);  // scaled value is out of 4500 (+/-)
            break;
        case LaunchPhase::LaunchDetected:
            // quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED); // changed this temporarily for 1dof test
            // quadplane.hold_stabilize(0.1f);
            quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
            attitude_control->set_throttle_out(0.1f, false, 0);
            plane.stabilize_roll();
            plane.stabilize_pitch();
            break;
        case LaunchPhase::Stabilized:
            quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
            // quadplane.hold_stabilize(0.1f);  // 50% throttle // should *not* be using pitch control at this time            
            plane.stabilize_roll();
            plane.stabilize_pitch();
            break;
        case LaunchPhase::Flare:
            // quadplane.hold_stabilize(0.1f);  // 50% throttle // should *not* be using pitch control at this time            
            plane.stabilize_roll();
            plane.stabilize_pitch();
            // SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 2725);  // scaled value out of 4500 (PWM = 2725*350/4500) (+/-)
            break;
        case LaunchPhase::WingsDeploy:
            // quadplane.hold_stabilize(0.55f);  // 50% throttle // *should* be using pitch control at this time
            plane.stabilize_pitch();
            plane.stabilize_roll();
            break;
        case LaunchPhase::Switch:  // if we don't successfully switch to FBWA, at least keep flying
            // quadplane.hold_stabilize(0.55f); // 50% throttle // *should* be using pitch control at this time
            plane.stabilize_pitch();
            plane.stabilize_roll();
            break;
        default:
            // should never get here
            break;
    }

}


bool ModeLaunch::launch_detected() {
    if (plane.manual_launch_triggered) {
        plane.manual_launch_triggered = false;
        return true;
    }
    return fabsf(AP::ahrs().get_accel_ef().x) >= 3 * GRAVITY_MSS;
}



float ModeLaunch::get_throttle_by_launch_phase() const {
    switch (phase) {
        case LaunchPhase::Disarmed:
            return 0.0f;
        case LaunchPhase::PreLaunch:
            return 0.1f;
        case LaunchPhase::LaunchDetected:
            return 0.1f; // 20% throttle
        case LaunchPhase::Stabilized:
            return 0.1f; // 50% throttle
        case LaunchPhase::Flare:
            return 0.1f; // 40% throttle
        case LaunchPhase::WingsDeploy:
            return 0.55f; // 55% throttle
        case LaunchPhase::Switch:
            return 0.55f; // 55% throttle
        default:
            return 0.55f; // should never get here
    }
}

#endif