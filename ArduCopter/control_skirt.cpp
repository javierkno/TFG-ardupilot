/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_auto.pde - init and run calls for auto flight mode
 *
 * This file contains the implementation for Land, Waypoint navigation and Takeoff from Auto mode
 * Command execution code (i.e. command_logic.pde) should:
 *      a) switch to Auto flight mode with set_mode() function.  This will cause auto_init to be called
 *      b) call one of the three auto initialisation functions: auto_wp_start(), auto_takeoff_start(), auto_land_start()
 *      c) call one of the verify functions auto_wp_verify(), auto_takeoff_verify, auto_land_verify repeated to check if the command has completed
 * The main loop (i.e. fast loop) will call update_flight_modes() which will in turn call auto_run() which, based upon the auto_mode variable will call
 *      correct auto_wp_run, auto_takeoff_run or auto_land_run to actually implement the feature
 */

/*
 *  While in the auto flight mode, navigation or do/now commands can be run.
 *  Code in this file implements the navigation commands
 */

// auto_init - initialise auto controller
bool Copter::skirt_init(bool ignore_checks)
{
    //if ((position_ok() && mission.num_commands() > 1) || ignore_checks) {
        //auto_mode = Auto_Loiter;

        // reject switching to auto mode if landed with motors armed but first command is not a takeoff (reduce change of flips)
        if (motors.armed() && ap.land_complete && !mission.starts_with_takeoff_cmd()) {
            gcs_send_text(MAV_SEVERITY_CRITICAL, "Auto: Missing Takeoff Cmd");
            return false;
        }

        // stop ROI from carrying over from previous runs of the mission
        // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
        if (auto_yaw_mode == AUTO_YAW_ROI) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }

        // initialise waypoint and spline controller
        wp_nav.wp_and_spline_init();

        // clear guided limits
        guided_limit_clear();

        // start/resume the mission (based on MIS_RESTART parameter)
        //mission.start_or_resume();
        //return true;
    //}else{
    //    return false;
    //}

    //waypoint_loc.alt = 4000;
    //waypoint_loc.lat = 376184080;
    //waypoint_loc.lng = -1223754350;

    //wp_nav.set_wp_destination(waypoint_loc);


    //Vector3f vec(100*100,100*100,10*100);

    //Vector3f vec = pv_location_to_vector(waypoint_loc);
    //vec = pv_dist_to_vector(skirt_radius, vec);

    //circle_nav.set_center(vec);
    //circle_nav.init(circle_nav.get_center());

    AP_Mission::Mission_Command com;
    mission.read_cmd_from_storage(1,com);
    Vector3f vec = pv_location_to_vector(com.content.location);
    vec = pv_dist_to_vector(skirt_radius, vec);

    wp_nav.set_wp_destination(vec,false);

    /*uint16_t ind = 0;
    AP_Mission::Mission_Command com;
    for(bool read = true;read;ind++) {
      mission.read_cmd_from_storage(ind,com);

      com.content.location;

    }*/

    return true;
}

// auto_run - runs the auto controller
//      should be called at 100hz or more
//      relies on run_autopilot being called at 10hz which handles decision making and non-navigation related commands
void Copter::skirt_run()
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


      if(wp_nav.reached_wp_destination()) {
        //mission.set_current_cmd(mission.get_current_nav_index()+1);
      }




/*
//CIRCLE --------------------------------------

// call circle controller
circle_nav.update();

// call z-axis position controller
pos_control.update_z_controller();

// roll & pitch from waypoint controller, yaw rate from pilot
attitude_control.input_euler_angle_roll_pitch_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), circle_nav.get_yaw(),true);
*/

}









Vector3f Copter::pv_dist_to_vector(const float &rad, const Vector3f &destination)
{
    Vector3f home = pv_location_to_vector(ahrs.get_home());
    Vector3f vec;
    float bearing = (-pv_get_bearing_cd(home, destination)+9000)/DEGX100;
    float distance =  pv_get_horizontal_distance_cm(home, destination);


    vec.x = sin(bearing) * (distance - skirt_radius);
    vec.y = cos(bearing) * (distance - skirt_radius);
    vec.z = destination.z;

    return vec;
}





// get_auto_heading - returns target heading depending upon auto_yaw_mode
// 100hz update rate
float Copter::get_heading(void)
{
    return wp_nav.get_yaw()+90;
}
