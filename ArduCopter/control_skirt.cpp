/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"
//#include <string>

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
    AP_Mission::Mission_Command c;
    uint16_t ind = 0;
    for(bool read = true;read;ind++) {
        read = mission.read_cmd_from_storage(ind,c);
        // reject switching to skirt mode if there are commands other tan navigation commands
        if(!mission.is_nav_cmd(c)) {
            gcs_send_text(MAV_SEVERITY_CRITICAL, "Skirt: All commands must be navigation commands");
            return false;
      }
    }

    if ((position_ok() && mission.num_commands() > 1) || ignore_checks) {
        //skirt_mode = Auto_Loiter;

        // reject switching to skirt mode if landed with motors armed but first command is not a takeoff (reduce change of flips)
        if (motors.armed() && ap.land_complete && !mission.starts_with_takeoff_cmd()) {
            gcs_send_text(MAV_SEVERITY_CRITICAL, "Skirt: this flight mode can't take off");
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

        if (follow_left) {
            circle_nav.set_rate(45);
        } else {
            circle_nav.set_rate(-45);
        }

        //--------------------------------------------------------------------------------------------------------------------
        //--------             Start                 -------------------------------------------------------------------------
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
{    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
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
        attitude_control.input_euler_angle_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_heading(),true);
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

    if(pv_get_horizontal_distance_cm(inertial_nav.get_position(), waypoint_calculado) < 220) {

        get_next_waypoint();
    }
}

Vector3f Copter::pv_translate_vector(const Vector3f &origin, const Vector3f &waypoint1, const Vector3f &waypoint2, const float &r) {

    Vector3f temp = (waypoint2 - waypoint1).normalized();

    float d = (waypoint1-origin).length();
    float i = 0;

    if (sq(d) - sq(r) > 0) {
        i = safe_sqrt(sq(d) - sq(r));
    }

    temp = temp * i;
    temp = waypoint1 + temp;
    temp = origin - temp;
    temp.z = 0;

    return waypoint2 + temp;
}

Vector3f Copter::pv_get_vector_perp(const Vector3f &origin, const Vector3f &waypoint1, const Vector3f &waypoint2, const float &r) {
    Vector3f vec;
    Vector3f vec2;
    float m = 0;
    float n = 0;
    float first_part = 0;
    float a = floor(waypoint1.y*1000)/1000;
    float b = floor(waypoint1.x*1000)/1000;

    if (waypoint2.y - waypoint1.y != 0 ) {
        m = (waypoint2.y - waypoint1.y) / (waypoint2.x - waypoint1.x);
        m= -1/m;
        n = waypoint1.y - m * waypoint1.x;

        first_part = safe_sqrt(-sq(a) + 2*a*b*m + 2*a*n - sq(b)*sq(m) - 2*b*m*n + sq(m)*sq(r) - sq(n) + sq(r));
        vec.x = (first_part + a*m + b - m*n) / (sq(m)+1);
        vec.y = m*vec.x + n;
        vec2.x = (-first_part + a*m + b - m*n) / (sq(m)+1);
        vec2.y = m*vec2.x + n;
    } else {
        n = waypoint1.x;

        first_part = safe_sqrt(-sq(a) + 2*a*b*m + 2*a*n - sq(b)*sq(m) - 2*b*m*n + sq(m)*sq(r) - sq(n) + sq(r));
        vec.y = (first_part + a*m + b - m*n) / (sq(m)+1);
        vec.x = n;
        vec2.y = (-first_part + a*m + b - m*n) / (sq(m)+1);
        vec2.x = n;
    }

    vec.z = waypoint2.z;
    vec2.z = waypoint2.z;

    if (get_direction(origin, waypoint1, waypoint2)) {
        if ((origin-vec).length() < (origin-vec2).length()) {
            if (follow_left) {
                return vec2;
            } else {
                return vec;
            }
        } else {
            if (follow_left) {
                return vec;
            } else {
                return vec2;
            }
        }
    } else {
        if ((origin-vec).length() > (origin-vec2).length()) {
            if (follow_left) {
                return vec2;
            } else {
                return vec;
            }
        } else {
            if (follow_left) {
                return vec;
            } else {
                return vec2;
            }
        }
    }
}

Vector3f Copter::pv_translate_vector_collision(const Vector3f &origin, const Vector3f &waypoint1, const Vector3f &waypoint2, const Vector3f &waypoint3, const float &r) {
    Vector3f vec;
    float d = 0;
    float alpha = 0;

    float m = (waypoint2.y - waypoint1.y) / (waypoint2.x - waypoint1.x);
    float n = origin.y - m * origin.x;

    float m2 = (waypoint3.y - waypoint2.y) / (waypoint3.x - waypoint2.x);
    float n2 = waypoint2.y - m2 * waypoint2.x;

    vec.x = (n2 - n) / (m - m2);
    vec.y = (m*n2 - m2*n) / (m - m2);
    vec.z = waypoint3.z;

    //std::string s = std::to_string(degrees((waypoint1-waypoint2).angle(waypoint3-waypoint2)));
    //gcs_send_text(MAV_SEVERITY_NOTICE, ("Angulo pared = " + s).c_str());

    alpha = 360 - 90 - degrees((waypoint1-waypoint2).angle(waypoint3-waypoint2));
    d = abs(r / cos(radians(alpha)));

    //std::string s3 = std::to_string(alpha);
    //gcs_send_text(MAV_SEVERITY_NOTICE, ("Angulo pared complementario = " + s3).c_str());

    //std::string s1 = std::to_string(r);
    //std::string s2 = std::to_string(d);
    //gcs_send_text(MAV_SEVERITY_NOTICE, ("Distancia pared = " + s1).c_str());
    //gcs_send_text(MAV_SEVERITY_NOTICE, ("Distancia calculada = " + s2).c_str());

    return shorten_vector(d, origin, vec);
}


// get_heading - returns target heading depending upon auto_yaw_mode
// 100hz update rate
float Copter::get_heading(void) {
    if (follow_left) {
        return wp_nav.get_yaw() + 9000;
    } else {
        return wp_nav.get_yaw() - 9000;
    }
}

// get_direction
// devuelve cierto si el siguiente waypoint esta a la derecha del vector director (waypoint2-waypoint1), falso en otro caso
bool Copter::get_direction(const Vector3f &waypoint1, const Vector3f &waypoint2, const Vector3f &waypoint3) {

    float first_angle = pv_get_bearing_cd(waypoint1, waypoint2);
    float second_angle = pv_get_bearing_cd(waypoint2, waypoint3);

    if (first_angle > 0 && first_angle < 18000) {
        if ((first_angle > second_angle || (second_angle > 36000 && second_angle > abs(18000 - first_angle)))) {
            //left
            return false;
        } else {
            //right
            return true;
        }
    } else {
        if (first_angle < second_angle || (second_angle > 0 && second_angle < abs(18000 - first_angle))) {
            //right
            return true;
        } else {
            //left
            return false;
        }
    }
}

// isLeft(): test if a point is Left|On|Right of an infinite 2D line.
//    Input:  three points P0, P1, and P2
//    Return: >0 for P2 left of the line through P0 to P1
//          =0 for P2 on the line
//          <0 for P2 right of the line
//inline int
//isLeft( Point P2, Point P0, Point P1) {
//    return ( (P1.x - P0.x) * (P2.y - P0.y)
//           - (P2.x - P0.x) * (P1.y - P0.y) );
//}
//
//uint8_t isLeft( Point origin, Point waypoint1, Point waypoint2) {
//    return ( (waypoint2.x - waypoint1.x) * (origin.y - waypoint1.y)
//           - (origin.x - waypoint1.x) * (waypoint2.y - waypoint1.y) );
//}
//}

void Copter::get_next_waypoint() {

    if (get_next_command()) {

        if (first_run) {

            waypoint_calculado = shorten_vector(skirt_radius, pv_location_to_vector(ahrs.get_home()), waypoint_actual);
            set_wp_mode();
            first_run = false;

        } else {
            Vector3f v1 = waypoint_calculado - waypoint_anterior;//(waypoint_calculado.x - waypoint_anterior.x, waypoint_calculado.y - waypoint_anterior.y, 0);
            Vector3f v2 = waypoint_actual - waypoint_anterior;//(waypoint_actual.x - waypoint_anterior.x, waypoint_actual.y - waypoint_anterior.y, 0);

            //curve
            if (degrees(v1.angle(v2)) > 95) { //curve

                Vector3f temp = waypoint_calculado;
                waypoint_calculado = pv_get_vector_perp(waypoint_calculado, waypoint_anterior, waypoint_actual, skirt_radius);

                if (check_collisions(temp, waypoint_calculado)) {
                    waypoint_calculado = temp;
                    get_next_command();
                    waypoint_calculado = get_line_waypoint();

                    set_wp_mode();

                } else {
                    circle_nav.set_center(waypoint_anterior);
                    circle_nav.init(circle_nav.get_center());
                    wp_nav.set_wp_destination(waypoint_calculado,false);

                    skirt_mode=Skirt_Circle;
                }

            } else { //line

                Vector3f temp = waypoint_calculado;
                waypoint_calculado = get_line_waypoint();


                if (check_collisions(temp, waypoint_calculado)) {
                    get_next_command();
                    waypoint_calculado = get_line_waypoint();
                }

                set_wp_mode();
            }
        }

    } else {
        //No deberia pasar nunca por aqui
        //Lanzar failsafe
        //gcs_send_text(MAV_SEVERITY_NOTICE, "Error");
    }
}


void Copter::set_wp_mode() {
    wp_nav.set_wp_destination(waypoint_calculado,false);
    waypoint_anterior = waypoint_actual;
    skirt_mode = Skirt_WP;
    prev_index = comm_index;
    comm_index++;
}

// check_collisions

bool Copter::check_collisions(const Vector3f &waypoint1, const Vector3f &waypoint2) {
    Vector3f temp;

    for(uint16_t i=mission.num_commands()-1; i>comm_index; i--) {
        mission.read_cmd_from_storage(i,com);
        temp = pv_location_to_vector(com.content.location);
        if(pv_dist_to_segment(temp, waypoint1, waypoint2) < skirt_radius+200 && i!=prev_index) {
            waypoint_actual = temp;
            prev_index = previous_index();
            comm_index = i;
            return true;
        }
    }
    return false;
}

bool Copter::get_next_command() {

    if(comm_index > mission.num_commands()-1) {
        comm_index = 1;
    }

    if (mission.read_cmd_from_storage(comm_index,com)) {

        //std::string a = std::to_string(comm_index);
        //gcs_send_text(MAV_SEVERITY_NOTICE, ("Actual: "+a).c_str());
        waypoint_actual = pv_location_to_vector(com.content.location);

        if(comm_index+1 < mission.num_commands()) {
            mission.read_cmd_from_storage(comm_index+1,com);
            //a = std::to_string(comm_index+1);
            //gcs_send_text(MAV_SEVERITY_NOTICE, ("Siguiente: "+a).c_str());
            waypoint_siguiente = pv_location_to_vector(com.content.location);
        }
        else {

            mission.read_cmd_from_storage(1,com);
            waypoint_siguiente = pv_location_to_vector(com.content.location);

            //gcs_send_text(MAV_SEVERITY_NOTICE, "Siguiente: 1");

        }
        return true;
    } else {
        return false;
    }

}

Vector3f Copter::get_line_waypoint() {
    Vector3f aux = pv_translate_vector(waypoint_calculado, waypoint_anterior, waypoint_actual, skirt_radius);
    Vector3f aux2 = pv_translate_vector_collision(waypoint_calculado, waypoint_anterior, waypoint_actual, waypoint_siguiente, skirt_radius);

    if (follow_left) {
        if (get_direction(waypoint_anterior, waypoint_actual, waypoint_siguiente) ||
            (aux-waypoint_calculado).length() < (aux2-waypoint_calculado).length()) {
            return  aux;
        } else {
            return  aux2;
        }
    } else {
        if (get_direction(waypoint_anterior, waypoint_actual, waypoint_siguiente) ||
            (aux-waypoint_calculado).length() < (aux2-waypoint_calculado).length()) {
            return  aux2;
        } else {
            return  aux;
        }
    }
}


uint16_t Copter::previous_index() {
    if (comm_index-1 > 0) {
        return comm_index-1;
    } else {
        //TODO comprobar si esto cambia cuando falta el comando 0 (HOME)
        return mission.num_commands()-1;
    }
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------

// dist_Point_to_Segment(): get the distance of a point to a segment
//     Input:  a Point P and a Segment S (in any dimension)
//     Return: the shortest distance from P to S
float Copter::pv_dist_to_segment(const Vector3f &point, const Vector3f &start, const Vector3f &end) {
    Vector3f v = end - start;
    Vector3f w = point - start;

    double c1 = w * v;
    if (c1 <= 0) {
        return (point - start).length();
    }

    double c2 = v * v;
    if (c2 <= c1) {
        return (point - end).length();
    }

    double b = c1 / c2;
    Vector3f Pb = start + v * b;
    return (point - Pb).length();
}

Vector3f Copter::shorten_vector(const float &r, const Vector3f &waypoint1, const Vector3f &waypoint2) {
    Vector3f vec;

    float bearing = (9000-pv_get_bearing_cd(waypoint1, waypoint2))/DEGX100;
    float distance =  pv_get_horizontal_distance_cm(waypoint1, waypoint2);

    vec.x = sin(bearing) * (distance - r);
    vec.y = cos(bearing) * (distance - r);

    vec.x += waypoint1.x;
    vec.y += waypoint1.y;
    vec.z = waypoint2.z;

    return vec;
}
