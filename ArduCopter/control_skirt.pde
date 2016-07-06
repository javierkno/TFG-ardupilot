/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define RIGTH 1
#define LEFT -1


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


bool follow_left = true;
float skirt_radius = 50 * 100;
SkirtMode skirt_mode;

uint16_t comm_index = 1;
uint16_t prev_index = 0;

AP_Mission::Mission_Command com;
Location waypoint_loc;

Vector3f next_waypoint;
Vector3f actual_waypoint;
Vector3f prev_waypoint;
Vector3f calc_waypoint;

bool first_run = true;

// skirt_init - initialise skirt controller
static bool skirt_init(bool ignore_checks)
{
    first_run = true;
    comm_index = 1;
    prev_index = 0;

    mission.read_cmd_from_storage(1,com);
    Vector3f vec = pv_location_to_vector(com.content.location);
    Vector3f actual_pos = inertial_nav.get_position();
    float dist;
    float dist_min = (vec - actual_pos).length();

    // check stored commands
    for(uint16_t ind = 1; ind < mission.num_commands(); ind++) {
        mission.read_cmd_from_storage(ind,com);

        vec = pv_location_to_vector(com.content.location);
        if ((dist = (vec - actual_pos).length()) < dist_min) {
            comm_index = ind;
            dist_min = dist;
        }

        //printV(vec,ind);
        // reject switching to skirt mode if there are commands other tan navigation commands
        if(!mission.is_nav_cmd(com)) {
            //gcs_send_text(MAV_SEVERITY_CRITICAL, "Skirt: All commands must be navigation commands");
            return false;
      }
    }

    if ((GPS_ok() && inertial_nav.position_ok() && mission.num_commands() > 1) || ignore_checks) {
        // stop ROI from carrying over from previous runs of the mission
        if (auto_yaw_mode == AUTO_YAW_ROI) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }

        // initialise waypoint and spline controller
        wp_nav.wp_and_spline_init();
        circle_nav.set_radius(skirt_radius);

        if (follow_left) {
            circle_nav.set_rate(45);
        } else {
            circle_nav.set_rate(-45);
        }

        // first time entering skirt mode
        // calculate the route from vehicle's acual position to first waypoint
        skirt_get_next_command();
        calc_waypoint = pv_shorten(skirt_radius, actual_pos/*ahrs.get_home()*/, actual_waypoint);
        // set destination
        wp_nav.set_wp_destination(calc_waypoint);
        // set mode
        skirt_set_wp_mode();

        return true;
    } else {
        return false;
    }
}

// skirt_run - runs the skirt controller
//      should be called at 100hz or more
//      relies on run_autopilot being called at 10hz which handles decision making and non-navigation related commands
static void skirt_run()
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
static void skirt_wp_run()
{

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed) {
        // To-Do: reset waypoint origin to current location because copter is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        // tell motors to do a slow start
        motors.slow_start(true);
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
        if (target_yaw_rate != 0) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // run waypoint controller
    wp_nav.update_wpnav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(),true);
    }

    // check if reached destination
    if (wp_nav.reached_wp_destination()) {
        // call the decision maker
        skirt_get_next_waypoint();
    }
}

// skirt_circle_run - circle in SKIRT flight mode
//      called by skirt_run at 100hz or more
static void skirt_circle_run()
{
    // call circle controller
    circle_nav.update();

    // call z-axis position controller
    pos_control.update_z_controller();

    // roll & pitch from waypoint controller, yaw rate from pilot
    attitude_control.angle_ef_roll_pitch_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), circle_nav.get_yaw(),true);

    // check if reached destination
    if(pv_get_horizontal_distance_cm(inertial_nav.get_position(), calc_waypoint) < 220) {
        // call the decision maker
        skirt_get_next_waypoint();
    }
}

// skirt_get_heading - returns target heading depending upon auto_yaw_mode
// 100hz update rate
static float skirt_get_heading(void)
{
    if(!first_run) {
        if (follow_left) {
            return wp_nav.get_yaw() + 9000;
        } else {
            return wp_nav.get_yaw() - 9000;
        }
    }
    return wp_nav.get_yaw();
}

// skirt_get_next_command - gets the next command allocated in memory
static bool skirt_get_next_command()
{
    // if reached the last command, return to first
    if(comm_index > mission.num_commands()-1) {
        comm_index = 1;
    }

    // read next command
    if (mission.read_cmd_from_storage(comm_index,com)) {
        // transform absolute location to a vector from home
        actual_waypoint = pv_location_to_vector(com.content.location);

        // read the following command for route calculation pourposes
        if(comm_index+1 < mission.num_commands()) {
            mission.read_cmd_from_storage(comm_index+1,com);
            next_waypoint = pv_location_to_vector(com.content.location);
        }
        else {
            mission.read_cmd_from_storage(1,com);
            next_waypoint = pv_location_to_vector(com.content.location);
        }
        return true;
    } else {
        return false;
    }
}

// skirt_get_next_waypoint - decides the route to the next waypoint
static void skirt_get_next_waypoint()
{
    first_run = false;
    if (skirt_get_next_command()) {
        Vector3f v1 = calc_waypoint - prev_waypoint;
        Vector3f v2 = actual_waypoint - prev_waypoint;

        // calculate angle in route
        if (degrees(v1.angle(v2)) > 95) {

            // calculate circular route
            Vector3f temp = calc_waypoint;
            calc_waypoint = pv_get_vector_perp(calc_waypoint, prev_waypoint, actual_waypoint, skirt_radius);

            // check for collisions in route
            if (skirt_check_collisions(temp, calc_waypoint)) {
                // avoid waypoints and set straight line mode
                calc_waypoint = temp;
                skirt_get_next_command();
                calc_waypoint = skirt_get_line_waypoint();
                skirt_set_wp_mode();
            } else {
                // set circle mode
                circle_nav.set_center(prev_waypoint);
                circle_nav.init(circle_nav.get_center());
                wp_nav.set_wp_destination(calc_waypoint);
                skirt_mode=Skirt_Circle;
            }
        } else {
            //print("line");
            // set straight line mode
            Vector3f temp = skirt_get_line_waypoint();
            // check for collisions in route
            if (skirt_check_collisions(calc_waypoint, temp)) {
                // avoid waypoints
                skirt_get_next_command();
                calc_waypoint = skirt_get_line_waypoint();
            } else {
                calc_waypoint = temp;
            }
            skirt_set_wp_mode();
        }
    } else {
        // No deberia pasar nunca por aqui
        // ¿Lanzar failsafe?
        // gcs_send_text(MAV_SEVERITY_NOTICE, "Error");
    }
}

// skirt_set_wp_mode - set the line waypoint mode
static void skirt_set_wp_mode()
{
    // set waypoint controller target
    wp_nav.set_wp_destination(calc_waypoint);

    // set controller
    skirt_mode = Skirt_WP;
    // increment variables
    prev_waypoint = actual_waypoint;
    prev_index = comm_index;
    comm_index++;
}

// skirt_check_collisions - checks possible collisions in the actual route and avoids them
static bool skirt_check_collisions(const Vector3f &waypoint1, const Vector3f &waypoint2)
{
    Vector3f temp;
    // read every command following actual command
    for(uint16_t i=mission.num_commands()-1; i>comm_index; i--) {
        mission.read_cmd_from_storage(i,com);
        temp = pv_location_to_vector(com.content.location);
        // if the distance between the segment waypoint1-waypoint2 and the waypoint in the route is too close, ignore the waypoint
        if(pv_dist_to_segment(temp, waypoint1, waypoint2) < skirt_radius+200 && i!=prev_index) {
            actual_waypoint = temp;
            prev_index = comm_index;
            comm_index = i;
            return true;
        }
    }
    return false;
}

// skirt_get_line_waypoint - returns the right line route to the next waypoint
static Vector3f skirt_get_line_waypoint()
{
    // calcule the two posible destination waypoints
    Vector3f aux = pv_translate_vector(calc_waypoint, prev_waypoint, actual_waypoint, skirt_radius);
    Vector3f aux2 = pv_translate_vector_collision(calc_waypoint, prev_waypoint, actual_waypoint, next_waypoint, skirt_radius);

    // decides which is the right one depending on the direction o the multicopter (left or right)
    if (follow_left) {
        if (get_direction(prev_waypoint, actual_waypoint, next_waypoint) == RIGTH) {
            //clockwise --> right : long way
            return  aux;
        } else {
            //clockwise --> left : short way
            return aux2;
        }
    } else {
        if (get_direction(prev_waypoint, actual_waypoint, next_waypoint) == RIGTH) {
            //counterclockwise --> right : short way
            return  aux2;
        } else {
            //counterclockwise --> left : long way
            return  aux;
        }
    }
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------  Calculation functions  -----------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------

// pv_translate_vector_collision - calculates the translation of the (waypoint1, origin) vector avoiding collisions with the (waypoint2, waypoint3) line
static Vector3f pv_translate_vector_collision(const Vector3f &origin, const Vector3f &waypoint1, const Vector3f &waypoint2, const Vector3f &waypoint3, const float &r)
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

// pv_dist_to_segment - get the distance from a point to a segment (formed by two points, start and end)
static float pv_dist_to_segment(const Vector3f &point, const Vector3f &start, const Vector3f &end)
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
static Vector3f pv_shorten(const float &r, const Vector3f &waypoint1, const Vector3f &waypoint2)
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

// pv_translate_vector - translates the vector (waypoint1, orgigin) to (waypoint2)
static Vector3f pv_translate_vector(const Vector3f &origin, const Vector3f &waypoint1, const Vector3f &waypoint2, const float &r)
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

// pv_get_vector_perp - obtains the position vector equivalent to rotate 90º the (waypoint1, origin) vector translated to (waypoint2)
static Vector3f pv_get_vector_perp(const Vector3f &origin, const Vector3f &waypoint1, const Vector3f &waypoint2, const float &r)
{
    Vector3f temp = (origin - waypoint1);
    Vector3f inc;
    float alpha;
    float angle;

    angle = pv_get_bearing_cd(waypoint1, origin) - pv_get_bearing_cd(waypoint1, waypoint2);

    if(follow_left) {
        angle = -angle;
        angle -= 9000;
    } else {
        angle -= 9000;
        angle = -angle;
    }

    alpha = radians(angle/100);

    inc.x = temp.x*cos(alpha) - temp.y*sin(alpha);
    inc.y = temp.x*sin(alpha) + temp.y*cos(alpha);

    return waypoint1 + inc;
}


// get_direction - returns true if destination is in the right side of the line formed by waypoint1 and waypoint2, false in other case
static int8_t get_direction(const Vector3f &waypoint1, const Vector3f &waypoint2, const Vector3f &destination)
{
    float first_angle = pv_get_bearing_cd(waypoint1, waypoint2);
    float neg_first_angle = pv_get_bearing_cd(waypoint2, waypoint1);
    float second_angle = pv_get_bearing_cd(waypoint2, destination);

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
