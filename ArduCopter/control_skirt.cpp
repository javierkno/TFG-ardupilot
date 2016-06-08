/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"
#include <string>

/*
 * control_skirt.pde - init and run calls for auto flight mode
 *
 *
 *
 *
 *
 *
 *
 *
 */

/*
 *  While in the skirt flight mode only navigation commands can be run.
 *  Code in this file implements the navigation commands
 */


// skirt_init - initialise skirt controller
bool Copter::skirt_init(bool ignore_checks)
{

    /*AP_Mission::Mission_Command c;
    uint16_t ind = 0;
    for(bool read = true;read;ind++) {
      read = mission.read_cmd_from_storage(ind,c);
      // reject switching to skirt mode if there are commands other tan navigation commands
      if(!mission.is_nav_cmd(c)) {
        gcs_send_text(MAV_SEVERITY_CRITICAL, "Skirt: All commands must be navigation commands");
        return false;
      }
    }*/

    comm_index = 1;

    if ((position_ok() && mission.num_commands() > 1) || ignore_checks) {
        //skirt_mode = Auto_Loiter;

        // reject switching to skirt mode if landed with motors armed but first command is not a takeoff (reduce change of flips)
        if (motors.armed() && ap.land_complete && !mission.starts_with_takeoff_cmd()) {
            gcs_send_text(MAV_SEVERITY_CRITICAL, "Skirt: Missing Takeoff Cmd");
            return false;
        }

        // stop ROI from carrying over from previous runs of the mission
        // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
        if (auto_yaw_mode == AUTO_YAW_ROI) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }

        // initialise waypoint and spline controller
        wp_nav.wp_and_spline_init();
        circle_nav.set_radius(skirt_radius);

        // clear guided limits
        guided_limit_clear();

        // start/resume the mission (based on MIS_RESTART parameter)
        //mission.start_or_resume();


        //--------------------------------------------------------------------------------------------------------------------
        //--------   Start                 -----------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------

        get_next_waypoint();

        //--------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------
        //--------------------------------------------------------------------------------------------------------------------


        return true;
    } else{
        return false;
    }
}

// skirt_run - runs the skirt controller
//      should be called at 100hz or more
//      relies on run_autopilot being called at 10hz which handles decision making and non-navigation related commands
void Copter::skirt_run()
{
    //calc_wp_distance();
    //std::string s = std::to_string(wp_distance+1);
    //gcs_send_text(MAV_SEVERITY_NOTICE, s.c_str());

    // call the correct skirt controller
    switch (skirt_mode) {
    case Skirt_WP:
        skirt_wp_run();
        break;

    case Skirt_Circle:
        skirt_circle_run();
        break;
    }
}


// skirt_wp_run - runs the skirt waypoint controller
//      called by skirt_run at 100hz or more
void Copter::skirt_wp_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors.armed() || !ap.auto_armed || !motors.get_interlock()) {
        // To-Do: reset waypoint origin to current location because copter is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw_smooth(0, 0, 0, get_smoothing_gain());
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        // clear i term when we're taking off
        set_throttle_takeoff();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    failsafe_terrain_set_status(wp_nav.update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    //if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        //attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    //}else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.input_euler_angle_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_heading() ,true);
    //}


    if (wp_nav.reached_wp_destination()) {

      get_next_waypoint();

    }
}

// skirt_circle_run - circle in SKIRT flight mode
//      called by skirt_run at 100hz or more
void Copter::skirt_circle_run()
{

    // call circle controller
    circle_nav.update();

    // call z-axis position controller
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.input_euler_angle_roll_pitch_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), circle_nav.get_yaw(),true);

    /*
    Vector3f dist_to_dest = (curr_pos - Vector3f(0,0,terr_offset)) - _destination;
    if( dist_to_dest.length() <= _wp_radius_cm ) {
        _flags.reached_destination = true;
    }*/

    /*std::string s = std::to_string(inertial_nav.get_position().x);
    std::string s1 = std::to_string(inertial_nav.get_position().y);
    gcs_send_text(MAV_SEVERITY_NOTICE, ("CURR_POS: "+s+", "+s1).c_str());*/

    //.length() <= _wp_radius_cm

    //std::string s = std::to_string(pv_get_horizontal_distance_cm(inertial_nav.get_position(), lv_dest_mod));
    //gcs_send_text(MAV_SEVERITY_NOTICE, ("Distance to wp: "+s).c_str());

    if(pv_get_horizontal_distance_cm(inertial_nav.get_position(), lv_dest_mod) < 300) {

        gcs_send_text(MAV_SEVERITY_NOTICE, "Skirt: Circle complete");
        get_next_waypoint();
    }
}





Vector3f Copter::pv_dist_to_vector(const float &rad, const Vector3f &waypoint){
    Vector3f home = pv_location_to_vector(ahrs.get_home());
    Vector3f vec;
    float bearing = (-pv_get_bearing_cd(home, waypoint)+9000)/DEGX100;
    float distance =  pv_get_horizontal_distance_cm(home, waypoint);


    vec.x = sin(bearing) * (distance - skirt_radius);
    vec.y = cos(bearing) * (distance - skirt_radius);
    vec.z = waypoint.z;

    return vec;
}

Vector3f Copter::pv_get_vector_par(const Vector3f &origin, const Vector3f &waypoint1, const Vector3f &waypoint2, const float &r) {
    Vector3f vec;
    Vector3f vec2;
    float m = 0;
    float n = 0;
    float first_part = 0;
    float a = waypoint2.y;
    float b = waypoint2.x;

    if (waypoint2.x - waypoint1.x != 0 ) {
        m = (waypoint2.y - waypoint1.y) / (waypoint2.x - waypoint1.x);
        n = origin.y - m * origin.x;
        first_part = safe_sqrt(-sq(a) + 2*a*b*m + 2*a*n - sq(b)*sq(m) - 2*b*m*n + sq(m)*sq(r) - sq(n) + sq(r));
        vec.x =     (first_part + a*m + b - m*n) / (sq(m)+1);
        vec.y =   (m*first_part + a*sq(m) + b*m + n) / (sq(m)+1);
        vec2.x =   (-first_part + a*m + b - m*n) / (sq(m)+1);
        vec2.y = (-m*first_part + a*sq(m) + b*m + n) / (sq(m)+1);
    } else {
        n = origin.x;
        first_part = safe_sqrt(-sq(a) + 2*a*b*m + 2*a*n - sq(b)*sq(m) - 2*b*m*n + sq(m)*sq(r) - sq(n) + sq(r));
        vec.y =     (first_part + a*m + b - m*n) / (sq(m)+1);
        vec.x =   (m*first_part + a*sq(m) + b*m + n) / (sq(m)+1);
        vec2.y =   (-first_part + a*m + b - m*n) / (sq(m)+1);
        vec2.x = (-m*first_part + a*sq(m) + b*m + n) / (sq(m)+1);
    }

    if (pv_get_horizontal_distance_cm(origin, vec) < pv_get_horizontal_distance_cm(origin, vec2)) {
        return vec;
    } else {
        return vec2;
    }
}

Vector3f Copter::pv_get_vector_perp(const Vector3f &origin, const Vector3f &waypoint1, const Vector3f &waypoint2, const float &r) {
    Vector3f vec;
    Vector3f vec2;
    float m = 0;
    float n = 0;
    float first_part = 0;
    float a = waypoint1.y;
    float b = waypoint1.x;

    if (waypoint2.y - waypoint1.y != 0 ) {
        m =-1/( (waypoint2.y - waypoint1.y) / (waypoint2.x - waypoint1.x) );
        n = waypoint1.y - m * waypoint1.x;

        first_part = safe_sqrt(-sq(a) + 2*a*b*m + 2*a*n - sq(b)*sq(m) - 2*b*m*n + sq(m)*sq(r) - sq(n) + sq(r));
        vec.x =     (first_part + a*m + b - m*n) / (sq(m)+1);
        vec.y =   (m*first_part + a*sq(m) + b*m + n) / (sq(m)+1);
        vec2.x =   (-first_part + a*m + b - m*n) / (sq(m)+1);
        vec2.y = (-m*first_part + a*sq(m) + b*m + n) / (sq(m)+1);
    } else {
        n = waypoint1.x;

        first_part = safe_sqrt(-sq(a) + 2*a*b*m + 2*a*n - sq(b)*sq(m) - 2*b*m*n + sq(m)*sq(r) - sq(n) + sq(r));
        vec.y =     (first_part + a*m + b - m*n) / (sq(m)+1);
        vec.x =   (m*first_part + a*sq(m) + b*m + n) / (sq(m)+1);
        vec2.y =   (-first_part + a*m + b - m*n) / (sq(m)+1);
        vec2.x = (-m*first_part + a*sq(m) + b*m + n) / (sq(m)+1);
    }

    if (pv_get_horizontal_distance_cm(origin, vec) < pv_get_horizontal_distance_cm(origin, vec2)) {
        return vec2;
    } else {
        return vec;
    }
}

//if mode = false vector perpendicular, si true vector paralelo
Vector3f Copter::pv_get_vector(const Vector3f &origin, const Vector3f &waypoint1, const Vector3f &waypoint2, const float &r, const bool &mode) {
    Vector3f vec;
    Vector3f vec2;
    float m = 0;
    float n = 0;
    float first_part = 0;
    float a = waypoint2.y;
    float b = waypoint2.x;

    if (mode) {
        a = waypoint2.x;
        b = waypoint2.y;
    }

    if (waypoint2.x - waypoint1.x != 0 ) {
        m = (waypoint2.y - waypoint1.y) / (waypoint2.x - waypoint1.x);
        if (mode) {
        m=-1/m;
        }
        n = origin.y - m * origin.x;
        first_part = safe_sqrt(-sq(a) + 2*a*b*m + 2*a*n - sq(b)*sq(m) - 2*b*m*n + sq(m)*sq(r) - sq(n) + sq(r));
        vec.x =     (first_part + a*m + b - m*n) / (sq(m)+1);
        vec.y =   (m*first_part + a*sq(m) + b*m + n) / (sq(m)+1);
        vec2.x =   (-first_part + a*m + b - m*n) / (sq(m)+1);
        vec2.y = (-m*first_part + a*sq(m) + b*m + n) / (sq(m)+1);
    } else {
        if (mode) {
            n = waypoint1.x;
        } else {
            n = origin.x;
        }
        first_part = safe_sqrt(-sq(a) + 2*a*b*m + 2*a*n - sq(b)*sq(m) - 2*b*m*n + sq(m)*sq(r) - sq(n) + sq(r));
        vec.y =     (first_part + a*m + b - m*n) / (sq(m)+1);
        vec.x =   (m*first_part + a*sq(m) + b*m + n) / (sq(m)+1);
        vec2.y =   (-first_part + a*m + b - m*n) / (sq(m)+1);
        vec2.x = (-m*first_part + a*sq(m) + b*m + n) / (sq(m)+1);
    }


    if (pv_get_horizontal_distance_cm(origin, vec) < pv_get_horizontal_distance_cm(origin, vec2)) {
        if (mode) {
            return vec2;
        }
        return vec;
    } else {
        if (mode) {
            return vec;
        }
        return vec2;
    }
}


// get_auto_heading - returns target heading depending upon auto_yaw_mode
// 100hz update rate
float Copter::get_heading(void) {
    return wp_nav.get_yaw();//+9000;
}


void Copter::get_next_waypoint() {

    mission.read_cmd_from_storage(comm_index,com);
    lv_new_dest = pv_location_to_vector(com.content.location);


    if (comm_index == 1) {

        std::string s = std::to_string(lv_new_dest.x);
        std::string s1 = std::to_string(lv_new_dest.y);
        gcs_send_text(MAV_SEVERITY_NOTICE, (s+", "+s1).c_str());

        lv_dest = lv_new_dest;
        lv_dest_mod = pv_dist_to_vector(skirt_radius, lv_dest);
        wp_nav.set_wp_destination(lv_dest_mod,false);
        skirt_mode = Skirt_WP;

    } else {

        Vector3f v1;
        v1.x = lv_dest_mod.x - lv_dest.x;
        v1.y = lv_dest_mod.y - lv_dest.y;
        Vector3f v2;
        v2.x = lv_new_dest.x - lv_dest.x;
        v2.y = lv_new_dest.y - lv_dest.y;

        //si angulo entre vector1 y vector2 > 90

        std::string s = std::to_string(degrees(v1.angle(v2)));
        gcs_send_text(MAV_SEVERITY_NOTICE, ("Angle = " + s).c_str());

        if (degrees(v1.angle(v2)) > 90) {

            circle_nav.set_center(lv_dest);
            circle_nav.init(circle_nav.get_center());

            lv_dest_mod = pv_get_vector_perp(lv_dest_mod, lv_dest, lv_new_dest, skirt_radius);
            lv_dest = lv_new_dest;
            wp_nav.set_wp_destination(lv_dest_mod,false);

            skirt_mode=Skirt_Circle;

        } else {

            lv_dest_mod = pv_get_vector_par(lv_dest_mod, lv_dest, lv_new_dest, skirt_radius);
            lv_dest = lv_new_dest;
            wp_nav.set_wp_destination(lv_dest_mod,false);

            skirt_mode = Skirt_WP;
        }

    }
    comm_index++;
}
