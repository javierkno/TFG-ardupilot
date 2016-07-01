/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"


void Copter::printV(const Vector3f &v, int n) {
    gcs_send_text(MAV_SEVERITY_NOTICE, ("W" + std::to_string(n) + " = (" + std::to_string(v.x) + ", " + std::to_string(v.y) + ")").c_str());
}

void Copter::print(std::string s) {
    gcs_send_text(MAV_SEVERITY_NOTICE, s.c_str());
}
//std::string a = std::to_string(comm_index);


/*
 * control_skirt.pde - init and run calls for skirt flight mode
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
    for(uint16_t ind = 0;ind < mission.num_commands();ind++) {
        mission.read_cmd_from_storage(ind,c);
        printV(pv_location_to_vector(c.content.location),ind);
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

        first_run = true;
        // starts the flight mode calling to the decision maker
        skirt_get_next_waypoint();

        return true;

    } else {
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
        skirt_wp_run();
        //skirt_circle_run();
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
        attitude_control.input_euler_angle_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), skirt_get_heading(),true);
    //}

    // check if reached destination
    if (wp_nav.reached_wp_destination()) {
        // call the decision maker
        skirt_get_next_waypoint();
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

    // check if reached destination
    if(pv_get_horizontal_distance_cm(inertial_nav.get_position(), waypoint_calculado) < 220) {
        // call the decision maker
        skirt_get_next_waypoint();
    }
}

// skirt_get_heading - returns target heading depending upon auto_yaw_mode
// 100hz update rate
float Copter::skirt_get_heading(void)
{
    if (follow_left) {
        return wp_nav.get_yaw() + 9000;
    } else {
        return wp_nav.get_yaw() - 9000;
    }
}

// skirt_get_next_command - gets the next command allocated in memory
bool Copter::skirt_get_next_command()
{
    // if reached the last command, return to first
    if(comm_index > mission.num_commands()-1) {
        comm_index = 1;
    }

    // read next command
    if (mission.read_cmd_from_storage(comm_index,com)) {
        // transform absolute location to a vector from home
        waypoint_actual = pv_location_to_vector(com.content.location);

        // read the following command for route calculation pourposes
        if(comm_index+1 < mission.num_commands()) {
            mission.read_cmd_from_storage(comm_index+1,com);
            waypoint_siguiente = pv_location_to_vector(com.content.location);
        }
        else {
            mission.read_cmd_from_storage(1,com);
            waypoint_siguiente = pv_location_to_vector(com.content.location);
        }
        return true;
    } else {
        return false;
    }

}

// skirt_get_next_waypoint - decides the route to the next waypoint
void Copter::skirt_get_next_waypoint()
{
    if (skirt_get_next_command()) {

        if (first_run) {

            waypoint_calculado = pv_shorten(skirt_radius, pv_location_to_vector(ahrs.get_home()), waypoint_actual);
            skirt_set_wp_mode();
            first_run = false;

        } else {
            Vector3f v1 = waypoint_calculado - waypoint_anterior;//(waypoint_calculado.x - waypoint_anterior.x, waypoint_calculado.y - waypoint_anterior.y, 0);
            Vector3f v2 = waypoint_actual - waypoint_anterior;//(waypoint_actual.x - waypoint_anterior.x, waypoint_actual.y - waypoint_anterior.y, 0);

            print("next");
            if (degrees(v1.angle(v2)) > 95) {

                //print("origin");
                //printV(waypoint_calculado,0);

                Vector3f temp = waypoint_calculado;
                waypoint_calculado = pv_get_vector_perp(waypoint_calculado, waypoint_anterior, waypoint_actual, skirt_radius);

                // check for collisions in route
                if (skirt_check_collisions(temp, waypoint_calculado)) {
                    print("avoiding");
                    print("line");
                    // avoid waypoints and set "Skirt_WP" mode
                    waypoint_calculado = temp;
                    skirt_get_next_command();
                    waypoint_calculado = skirt_get_line_waypoint();
                    skirt_set_wp_mode();

                } else {
                    print("circle");
                    // set "Skirt_Circle" mode
                    circle_nav.set_center(waypoint_anterior);
                    circle_nav.init(circle_nav.get_center());
                    wp_nav.set_wp_destination(waypoint_calculado,false);

                    skirt_mode=Skirt_Circle;
                }

                /*print("anterior");
                printV(waypoint_anterior,1);
                print("actual");
                printV(waypoint_actual,2);
                print("siguiente");
                printV(waypoint_siguiente,3);
                print("calculado");
                printV(waypoint_calculado,4);*/

            } else {
                print("line");
                // set "Skirt_WP" mode
                //print("origin");
                //printV(waypoint_calculado,0);
                Vector3f temp = skirt_get_line_waypoint();
                // check for collisions in route
                if (skirt_check_collisions(waypoint_calculado, temp)) {
                    // avoid waypoints

                    skirt_get_next_command();
                    waypoint_calculado = skirt_get_line_waypoint();
                } else {
                    waypoint_calculado = temp;
                }

                /*print("anterior");
                printV(waypoint_anterior,1);
                print("actual");
                printV(waypoint_actual,2);
                print("siguiente");
                printV(waypoint_siguiente,3);
                print("calculado");
                printV(waypoint_calculado,4);*/

                skirt_set_wp_mode();
            }
        }

    } else {
        // No deberia pasar nunca por aqui
        // ¿Lanzar failsafe?
        // gcs_send_text(MAV_SEVERITY_NOTICE, "Error");
    }
}

// skirt_set_wp_mode - set the line waypoint mode
void Copter::skirt_set_wp_mode()
{
    wp_nav.set_wp_destination(waypoint_calculado,false);
    waypoint_anterior = waypoint_actual;
    skirt_mode = Skirt_WP;
    prev_index = comm_index;
    comm_index++;
}

// skirt_check_collisions - checks possible collisions in the actual route and avoids them
bool Copter::skirt_check_collisions(const Vector3f &waypoint1, const Vector3f &waypoint2)
{
    Vector3f temp;

    // read every command following actual command
    for(uint16_t i=mission.num_commands()-1; i>comm_index; i--) {
        mission.read_cmd_from_storage(i,com);
        temp = pv_location_to_vector(com.content.location);
        // if the distance between the segment waypoint1-waypoint2 and the waypoint in the route is too close, ignore the waypoint
        if(pv_dist_to_segment(temp, waypoint1, waypoint2) < skirt_radius+200 && i!=prev_index) {
            waypoint_actual = temp;
            prev_index = skirt_previous_index();
            comm_index = i;
            return true;
        }
    }
    return false;
}

// skirt_get_line_waypoint - returns the right line route to the next waypoint
Vector3f Copter::skirt_get_line_waypoint()
{
    // calcule the two posible destination waypoints
    Vector3f aux = pv_translate_vector(waypoint_calculado, waypoint_anterior, waypoint_actual, skirt_radius);
    Vector3f aux2 = pv_translate_vector_collision(waypoint_calculado, waypoint_anterior, waypoint_actual, waypoint_siguiente, skirt_radius);

    // decides which is the right one depending on the direction o the multicopter (left or right)
    if (follow_left) {
        if (get_direction(waypoint_anterior, waypoint_actual, waypoint_siguiente)==1) {
            //sentido horario --> derecha
            print("h-long-right");
            return  aux;
        } else {
            print("h-short-left");
            return aux2;
        }
    } else {
        if (get_direction(waypoint_anterior, waypoint_actual, waypoint_siguiente)==1) {
            print("a-short-right");
            return  aux2;
        } else {
            print("a-long-left");
            return  aux;
        }
    }
}

// skirt_previous_index - returns the index of the previous waypoint (avoiding ignored waypoints due to collisions)
uint16_t Copter::skirt_previous_index()
{
    if (comm_index-1 > 0) {
        return comm_index-1;
    } else {
        //TODO comprobar si esto cambia cuando falta el comando 0 (HOME)
        return mission.num_commands()-1;
    }
}


// pv_translate_vector_collision - calculates the translation of the waypoint1-origin vector avoiding collisions with the waypoint2, waypoint3 line
Vector3f Copter::pv_translate_vector_collision(const Vector3f &origin, const Vector3f &waypoint1, const Vector3f &waypoint2, const Vector3f &waypoint3, const float &r)
{
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

    alpha = 360 - 90 - degrees((waypoint1-waypoint2).angle(waypoint3-waypoint2));
    d = abs(r / cos(radians(alpha)));

    return pv_shorten(d, origin, vec);
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------  Calculation functions  -----------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------

// pv_dist_to_segment - get the distance of a point to a segment (formed by two points, start and end)
float Copter::pv_dist_to_segment(const Vector3f &point, const Vector3f &start, const Vector3f &end)
{
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


// pv_shorten - shortens the vector formed by waypoint1 and waypoint2 the distance specified in r
Vector3f Copter::pv_shorten(const float &r, const Vector3f &waypoint1, const Vector3f &waypoint2)
{
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

// pv_translate_vector - translates the vector formed between waypoint1 and orgigin to waypoint2
Vector3f Copter::pv_translate_vector(const Vector3f &origin, const Vector3f &waypoint1, const Vector3f &waypoint2, const float &r)
{
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

// pv_get_vector_perp - obtains the position vector equivalent to rotate 90º the waypoint1-origin vector translated to the waypoint2
Vector3f Copter::pv_get_vector_perp(const Vector3f &origin, const Vector3f &waypoint1, const Vector3f &waypoint2, const float &r)
{
    /*
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

        if ((origin-vec).length() < (origin-vec2).length()) {
            return vec2;
        } else {
            return vec;
        }*/


    Vector3f temp = (origin - waypoint1);
    Vector3f inc;
    float alpha;
    float angle;

    angle = pv_get_bearing_cd(waypoint1, origin) - pv_get_bearing_cd(waypoint1, waypoint2);


    if(follow_left) {
        if (get_direction(origin, waypoint1, waypoint2)==1) {
            angle = -angle;
            angle -= 9000;
        } else {
            angle = -angle;
            angle -= 9000;
        }
    } else {
        if (get_direction(origin, waypoint1, waypoint2)==-1) {
            //angle = -angle;
            angle -= 9000;
            angle = -angle;
        } else {
            angle -= 9000;
            angle = -angle;
        }
    }

    print("rotating" + std::to_string(angle));

    alpha = radians(angle/100);

    inc.x = temp.x*cos(alpha) - temp.y*sin(alpha);
    inc.y = temp.x*sin(alpha) + temp.y*cos(alpha);

    return waypoint1 + inc;

}


// get_direction - returns true if destination is in the right side of the line formed by waypoint1 and waypoint2, false in other case
int8_t Copter::get_direction(const Vector3f &waypoint1, const Vector3f &waypoint2, const Vector3f &destination)
{
    float first_angle = pv_get_bearing_cd(waypoint1, waypoint2);
    float second_angle = pv_get_bearing_cd(waypoint2, destination);

    float neg_first_angle = first_angle + 18000;
    if(neg_first_angle > 36000) {
        neg_first_angle -= 36000;
    }

    /*print("first angle");
    print(std::to_string(first_angle));
    print("second angle");
    print(std::to_string(second_angle));*/

    if(abs(first_angle - second_angle) < 1000) {
        return 0;
    }

    if (first_angle > 0 && first_angle < 18000) {
        if (first_angle > second_angle || (second_angle < 36000 && second_angle > neg_first_angle)) {
            // left
            return -1;
        } else {
            // right
            return 1;
        }
    } else {
        if (first_angle < second_angle || (second_angle > 0 && second_angle < neg_first_angle)) {
            // right
            return 1;
        } else {
            // left
            return -1;
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
//uint8_t isLeft( Vector3f origin, Vector3f waypoint1, Vector3f waypoint2) {
//    return ( (waypoint2.x - waypoint1.x) * (origin.y - waypoint1.y)
//           - (origin.x - waypoint1.x) * (waypoint2.y - waypoint1.y) );
//}
//}

//std::string a = std::to_string(comm_index);
//gcs_send_text(MAV_SEVERITY_NOTICE, ("Actual: "+a).c_str());
