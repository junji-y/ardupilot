#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// define start time --------------------------------------------
uint32_t init_now;

// althold_init - initialise althold controller
bool ModeSTARWARS::init(bool ignore_checks)                        // change
{
    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // measure the time only once -------------------------------
    init_now = AP_HAL::millis();
    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeSTARWARS::run()
{
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // 普通のalt_hold
    // get pilot desired lean angle
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // ch_7が1800より大きい場合は、target_roll, target_pitch, target_yaw_rate に直接値を入れてstarwars実行
    // ch_7値の取得
    int ch_7 = RC_Channels::get_radio_in(CH_7);
    // gcs().send_text(MAV_SEVERITY_WARNING, "ch_7(%i)", ch_7);
    
    // update times
    uint32_t now;
    now = AP_HAL::millis();

    // ch_7 high
    if (ch_7 > 1800) {

        // 最初ピッチ３０度で２秒前進飛行(starwars開始前)
        if (now - init_now < 2000) { 
          // fix target_roll, target_pitch
          target_roll = 0.0f, target_pitch = -3000.0f;      
          // fix yaw rate
          target_yaw_rate = 0.0f; 
        }

        // ヨーを左５５度くらい回してまっすぐな進行方向に向ける（starwars開始）
        if (now - init_now >= 2000 && now - init_now < 3000) { 
          // fix target_roll, target_pitch
          target_roll = 0.0f, target_pitch = -3000.0f;      
          // fix yaw rate
          target_yaw_rate = -5500.0f; 
        }

        // 右ロール６０度に加え、前進飛行のためピッチ６０度も加え４秒直線飛行(starwars本番)
        if (now - init_now >= 3000 && now - init_now < 7000) { 
          // fix target_roll, target_pitch
          target_roll = 6000.0f, target_pitch = -6000.0f;      
          // fix yaw rate
          target_yaw_rate = 0.0f; 
        }

        // ヨーを右５５度で進行方向をまっすぐにもどす(starwars終了後)
        if (now - init_now >= 7000 && now - init_now < 8000) { 
          // fix target_roll, target_pitch
          target_roll = 0.0f, target_pitch = -3000.0f;      
          // fix yaw ratemode
          target_yaw_rate = 5500.0f; 
        }

        // ピッチ３０度で前進飛行２秒間で終わり！(starwars終了後)
        if (now - init_now >= 8000 && now - init_now < 10000) { 
          // fix target_roll, target_pitch
          target_roll = 0.0f, target_pitch = -3000.0f;      
          // fix yaw rate
          target_yaw_rate = 0.0f;
        }

    }

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->set_yaw_target_to_current_heading();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        break;

    case AltHold_Flying:
        // gcs().send_text(MAV_SEVERITY_WARNING, "target_roll(%f), target_pitch(%f)", target_roll, target_pitch);     //taught by 市原さん
        // gcs().send_text(MAV_SEVERITY_WARNING, "target_climb_rate(%f), taget_yaw_rate(%f)", target_climb_rate, target_yaw_rate);      //100hz位か、コンソール上で激しく出続けるが、これは置いといて

        // display every 2 seconds
        // static uint32_t last_time;
            // if (now - last_time > 2000) {
               // last_time = now;
               // gcs().send_text(MAV_SEVERITY_WARNING, "now:%d", now/1000);
               // gcs().send_text(MAV_SEVERITY_WARNING, "Roll(%3.0f),Pitch(%3.0f)", target_roll/100, target_pitch/100);
            // }     

        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        // adjust climb rate using rangefinder
        target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // call z-axis position controller
    pos_control->update_z_controller();

}
