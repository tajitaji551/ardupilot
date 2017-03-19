#include "Copter.h"
#define SHAKE_WAIT_COUNT 2000 // 5000ms
#define SHAKE_THRESHOLD 95 // threshold

// shake_init - initialise shake controller
bool Copter::shake_init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to use shake to start
    return false;
#endif

    // do not enter the mode when already armed or when flying
    if (motors->armed()) {
        return false;
    }

    // init state
    shake_state.stage = Shake_Disarmed;
    shake_state.prev_stage = Shake_Disarmed;
    shake_state.nextmode_attempted = false;
    shake_state.last_stage_changed_ms = AP_HAL::millis();

    shake_detection.shake_wait_count = SHAKE_WAIT_COUNT;
    shake_detection.shake_detected_count = 0;

    return true;
}

// runs the shake to start controller
// should be called at 100hz or more
void Copter::shake_run()
{
	float velocityLen = inertial_nav.get_velocity().length();
    /* Shake state Machine
    Shake_Disarmed - motors are off
    Shake_Detecting_1 - the shake has been detected and copter was armed
    Shake_Armed -  motors are on and we are waiting for next shake
    Shake_Detecting_2 - the shake has been detected and the copter change
    Shake_HgtStabilise - the copter is kept level and  height is stabilised about the target height
    Shake_PosHold - the copter is kept at a constant position and height

    According above state machine, ShakeMode have to shake twice.
    */
    if ((shake_state.stage == Shake_Detecting_1 || shake_state.stage == Shake_Detecting_2)
        && velocityLen > SHAKE_THRESHOLD) {
    	shake_detection.shake_detected_count++;
    }
    uint32_t now = AP_HAL::millis();
    if (shake_state.stage == Shake_Disarmed && !motors->armed() && !motors->get_interlock()) {
        gcs_send_text(MAV_SEVERITY_INFO,"waiting for shake1");
        shake_state.stage = Shake_Detecting_1;
    } else if (shake_state.stage == Shake_Detecting_1 && shake_detection.shake_detected_count > SHAKE_THRESHOLD) {
        gcs_send_text(MAV_SEVERITY_INFO,"shake1 detected - motor armed");
        shake_state.stage = Shake_Armed;
        shake_detection.shake_detected_count = 0;
        shake_detection.shake_wait_count = SHAKE_WAIT_COUNT;
    } else if (shake_state.stage == Shake_Armed && motors->armed()) {
        gcs_send_text(MAV_SEVERITY_INFO,"waiting for shake2");
        shake_state.stage = Shake_Detecting_2;
    } else if (shake_state.stage == Shake_Detecting_2 && shake_detection.shake_detected_count > SHAKE_THRESHOLD) {
        gcs_send_text(MAV_SEVERITY_INFO,"shake2 detected - controlling height");
        shake_state.stage = Shake_HgtStabilize;

        // initialize vertical speed and acceleration limits
		// use brake mode values for rapid response
		pos_control->set_speed_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z);
		pos_control->set_accel_z(BRAKE_MODE_DECEL_RATE);

		pos_control->set_alt_target(inertial_nav.get_altitude() + 150);

		// set the initial velocity of the height controller demand to the measured velocity if it is going up
		// if it is going down, set it to zero to enforce a very hard stop
		pos_control->set_desired_velocity_z(fmaxf(inertial_nav.get_velocity_z(),0.0f));

		// Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
		set_auto_armed(true);
    } else if (shake_state.stage == Shake_HgtStabilize && pos_control->get_alt_error() < 50.0f) {
        gcs_send_text(MAV_SEVERITY_INFO,"shake2 detected - controlling position");
        shake_state.stage = Shake_PosHold;

        // initialise the loiter target to the curent position and velocity
		wp_nav->init_loiter_target();

		// Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
		set_auto_armed(true);
    } else if (shake_state.stage == Shake_PosHold && pos_control->get_horizontal_error() < 50.0f) {
        if (!shake_state.nextmode_attempted) {
            set_mode(BRAKE, MODE_REASON_SHAKE_COMPLETE);
            switch (g2.shake_nextmode) {
                case AUTO:
                case GUIDED:
                case RTL:
                case LAND:
                case BRAKE:
                    set_mode((control_mode_t)g2.throw_nextmode.get(), MODE_REASON_THROW_COMPLETE);
                    break;
                default:
                    // do nothing
                    break;
            }
            shake_state.nextmode_attempted = true;
        }
    }

    // shake waiting timeout
	if (shake_detection.shake_wait_count <= 0) {
		shake_detection.shake_wait_count = SHAKE_WAIT_COUNT;
		shake_detection.shake_detected_count = 0;
		shake_state.stage = Shake_Disarmed;
	}

    // Shake stage processing
    switch(shake_state.stage) {
    case Shake_Disarmed:
        motors->armed(false);
        // demand zero throttle (motors will be stopped anyway) and continually reset the attitude controller
		attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        break;
    case Shake_Detecting_1:
        // demand zero throttle (motors will be stopped anyway) and continually reset the attitude controller
		attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        break;
    case Shake_Detecting_2:
        // Hold throttle at zero during the shake and continually reset the attitude controller
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        break;
    case Shake_Armed:
    	// set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        // prevent motors from rotating before the shake is detected unless enabled by the user
        if (g.shake_motor_start == 1) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        }
    	motors->armed(true);
        break;
    case Shake_HgtStabilize:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f, get_smoothing_gain());

        // call height controller
        pos_control->set_alt_target_from_climb_rate_ff(0.0f, G_Dt, false);
        pos_control->update_z_controller();

        break;

    case Shake_PosHold:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // run loiter controller
        wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), 0.0f, get_smoothing_gain());

        // call height controller
        pos_control->set_alt_target_from_climb_rate_ff(0.0f, G_Dt, false);
        pos_control->update_z_controller();

        break;
    }

    if (shake_state.stage <= 3) {
        // Play the waiting for shake tone sequence to alert the user
        AP_Notify::flags.waiting_for_shake = true;
    	shake_detection.shake_wait_count--;
    } else {
        // Play the waiting for shake tone sequence to alert the user
        AP_Notify::flags.waiting_for_shake = false;
    }
    // log at 10Hz or stage changed
    if (shake_state.stage != shake_state.prev_stage
            || (now - shake_state.last_log_ms) > 100) {
        shake_state.prev_stage = shake_state.stage;
        shake_state.last_log_ms = now;
        uint32_t wcount = shake_detection.shake_wait_count;
        uint32_t dcount = shake_detection.shake_detected_count;
        Log_Write_Shake(shake_state.stage,
                        velocityLen,
                        motors->armed(),
                        wcount,
                        dcount);
    }
}
