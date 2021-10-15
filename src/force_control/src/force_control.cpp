#include <cmath>
#include "force_control.h"

using namespace force_data_type;
using namespace math_calculate;
using namespace planar_data_type;
using namespace ur_data_type;

ForceControl::ForceControl()
{
    passive_force_ = ForceValue();

    random_param_.x = (rand() % 200 - 100) / 100.0;
	random_param_.y = (rand() % 200 - 100) / 100.0;
	random_param_.z = (rand() % 200 - 100) / 100.0;

    double_accuracy_ = 0.000001;
    time_scale_ = 0;

    move_boundary_.boundary_min.x = -0.4;
    move_boundary_.boundary_min.y = -0.7; 
    move_boundary_.boundary_min.z = -0.2; 

    move_boundary_.boundary_max.x = 0.4 + 0.2;
    move_boundary_.boundary_max.y = -0.3; 
    move_boundary_.boundary_max.z = 0.6; 

    move_boundary_.radium_min = 0.15;
    move_boundary_.error_range = 0.05;

    end_effector_gravity_in_wf_.fx = 0.0;
    end_effector_gravity_in_wf_.fy = 0.0;  
    end_effector_gravity_in_wf_.fz =  -2.6;

    force_limit_max_.fx = 500.0;
    force_limit_max_.fy = 500.0;
    force_limit_max_.fz = 500.0;

    torque_limit_max_.tx = 8.0;
    torque_limit_max_.ty = 8.0;
    torque_limit_max_.tz = 8.0;

    force_pid_.kp = 0.06;
    force_pid_.ki = 0.0;
    force_pid_.kd = 0.0;
	torque_pid_.kp = 1.0;
	torque_pid_.ki = 0.0;
	torque_pid_.kd = 0.0;

    force_pid_output_limit_max_.fx = 0.5;
    force_pid_output_limit_max_.fy = 0.5;
    force_pid_output_limit_max_.fz = 0.5;
	torque_pid_output_limit_max_.tx = 5.0;
	torque_pid_output_limit_max_.ty = 5.0;
	torque_pid_output_limit_max_.tz = 5.0;

    memset(force_in_sensor_frame_, 0, sizeof(force_in_sensor_frame_));
    memset(&tcp_pose_in_base_, 0, sizeof(tcp_pose_in_base_));
    memset(&tcp_pose_in_workpiece_, 0, sizeof(tcp_pose_in_workpiece_));
    memset(&tcp_pose_in_workpiece_, 0, sizeof(tcp_pose_in_workpiece_));

    memset(&passive_force_torque_tf_a_, 0, sizeof(passive_force_torque_tf_a_));
    memset(&passive_force_torque_tf_r_, 0, sizeof(passive_force_torque_tf_r_));
    memset(&pid_output_, 0, sizeof(pid_output_));
    memset(&force_pid_error_pre_, 0, sizeof(force_pid_error_pre_));

    memset(&end_effector_gravity_in_tf_, 0, sizeof(end_effector_gravity_in_tf_));
    memset(&force_bias_, 0, sizeof(force_bias_));
    memset(&force_ref_, 0, sizeof(force_ref_));
    memset(&force_output_, 0, sizeof(force_output_));
    memset(&force_error_pre_, 0, sizeof(force_error_pre_));
    memset(&force_error_pre_1_, 0, sizeof(force_error_pre_1_));
    memset(&force_error_pre_2_, 0, sizeof(force_error_pre_2_));
    memset(&torque_error_pre_, 0, sizeof(torque_error_pre_));
}

void ForceControl::initPidFactor()
{
    force_pid_.kp = 0.045;
    torque_pid_.kp = 0.9;
}

ForceControl::~ForceControl()
{

}

force_data_type::ForceValue ForceControl::setPassiveForce(force_data_type::ForceValue force)
{
    passive_force_ = force;
}

planar_data_type::PlanarForceValue ForceControl::setPlanarPassiveForce(planar_data_type::PlanarForceValue force)
{
    passive_force_.fx = force.fx;
    passive_force_.fy = force.fy;
}

planar_data_type::PlanarForceValue ForceControl::getPlanarPassiveForce()
{
    PlanarForceValue force;
    force.fx = passive_force_.fx;
    force.fy = passive_force_.fy;
    return force;
}

void ForceControl::updatePidFactors(unsigned int force_level)
{
	switch (force_level)
    {
    case 1:
        force_pid_.kp = 1.5* force_pid_.kp * force_level;
        torque_pid_.kp = 1.5* torque_pid_.kp * force_level;
        break;
    case 2:
        force_pid_.kp = 1 * force_pid_.kp * force_level;
        torque_pid_.kp = 1* torque_pid_.kp * force_level;
        break;
    case 3:
        force_pid_.kp = 0.05 * force_pid_.kp * force_level;
        torque_pid_.kp = 0.05* torque_pid_.kp * force_level;
        break;
    case 4:
        force_pid_.kp = 0.2 * force_pid_.kp * force_level;
        torque_pid_.kp =0.2 * torque_pid_.kp * force_level;
        break;
    case 5:
        force_pid_.kp = 0.1 * force_pid_.kp * force_level;
        torque_pid_.kp =0.1 * torque_pid_.kp * force_level;
        break;
    default:
        break;
    }
}

force_data_type::ForceValue ForceControl::getPassiveForce()
{
    return passive_force_;
}

planar_data_type::PlanarForceValue ForceControl::getPlanarPassiveForce(
        planar_data_type::PlanarPoint target, 
        planar_data_type::PlanarPoint tcp_point_in_base,
        int vel_ratio,
        double distance_min
)
{
    PlanarPoint vector_tcp2target;
    vector_tcp2target.x = target.x - tcp_point_in_base.x;
    vector_tcp2target.y = target.y - tcp_point_in_base.y;
    double distance_tcp2target = sqrt(pow(vector_tcp2target.x, 2) + pow(vector_tcp2target.y, 2));

    PlanarForceValue passive_force;
    if (distance_tcp2target > distance_min)
    {
        passive_force.fx = vel_ratio / 100.0 * vector_tcp2target.x / distance_tcp2target;
        passive_force.fy = vel_ratio / 100.0 * vector_tcp2target.y / distance_tcp2target;
    }
    else if (distance_tcp2target > ZERO_RANGE)
    {
        passive_force.fx = vel_ratio / 100.0 * vector_tcp2target.y / distance_tcp2target * (distance_tcp2target - ZERO_RANGE) / (distance_min - ZERO_RANGE);
        passive_force.fy = vel_ratio / 100.0 * vector_tcp2target.x / distance_tcp2target * (distance_tcp2target - ZERO_RANGE) / (distance_min - ZERO_RANGE);
    }
    else
    {
        passive_force.fx = 0;
        passive_force.fy = 0;
    }

    return passive_force;
}

planar_data_type::PlanarForceValue ForceControl::getPlanarPassiveForce(
    planar_data_type::PlanarPoint start, 
    planar_data_type::PlanarPoint end, 
    planar_data_type::PlanarPoint tcp_point_in_base,
    int vel_ratio,
    double distance_min
)
{
    PlanarPoint vector_tcp2end;
    vector_tcp2end.x = end.x - tcp_point_in_base.x;
    vector_tcp2end.y = end.y - tcp_point_in_base.y;
    double distance_tcp2end = sqrt(pow(vector_tcp2end.x, 2) + pow(vector_tcp2end.y, 2));
    double distance_start2tcp = getPlanarP2PDistance(start, tcp_point_in_base);

    PlanarForceValue passive_force;
    if (distance_start2tcp < ZERO_RANGE)
    {
        passive_force.fx = 0;
        passive_force.fy = 0;
    }
    else if (distance_start2tcp < distance_min)
    {
        passive_force.fx = vel_ratio / 100.0 * vector_tcp2end.x / distance_tcp2end * (distance_start2tcp - ZERO_RANGE) / (distance_min - ZERO_RANGE);
        passive_force.fy = vel_ratio / 100.0 * vector_tcp2end.y / distance_tcp2end * (distance_start2tcp - ZERO_RANGE) / (distance_min - ZERO_RANGE);
    }
    else if (distance_tcp2end > distance_min)
    {
        passive_force.fx = vel_ratio / 100.0 * vector_tcp2end.x / distance_tcp2end;
        passive_force.fy = vel_ratio / 100.0 * vector_tcp2end.y / distance_tcp2end;
    }
    else if (distance_tcp2end == ZERO_RANGE)
    {
        passive_force.fx = vel_ratio / 100 * vector_tcp2end.x / distance_tcp2end * (distance_start2tcp - ZERO_RANGE) / (distance_min - ZERO_RANGE);
        passive_force.fy = vel_ratio / 100 * vector_tcp2end.y / distance_tcp2end * (distance_start2tcp - ZERO_RANGE) / (distance_min - ZERO_RANGE);
    }
    else
    {
        passive_force.fx = 0;
        passive_force.fy = 0;
    }

    return passive_force;
}

force_data_type::ForceValue ForceControl::getCartPassiveForce(
    ur_data_type::CartesianPoint start, 
    ur_data_type::CartesianPoint end, 
    ur_data_type::CartesianPoint tcp_point_in_base,
    int vel_ratio,
    double distance_min
)
{
    CartesianPoint vector_tcp2end;
    vector_tcp2end.x = end.x - tcp_point_in_base.x;
    vector_tcp2end.y = end.y - tcp_point_in_base.y;
    vector_tcp2end.z = end.z - tcp_point_in_base.z;
    double distance_to_tcp2end = sqrt(pow(vector_tcp2end.x, 2) + pow(vector_tcp2end.y, 2) + pow(vector_tcp2end.z, 2));
    double distance_to_start2tcp = getP2PDistance(start, tcp_point_in_base);

    ForceValue passive_force;
    if (distance_to_start2tcp < ZERO_RANGE)
    {
        passive_force.fx = 0.0;
        passive_force.fy = 0.0;
        passive_force.fz = 0.0;
    }
    else if (distance_to_start2tcp < distance_min)
    {
        passive_force.fz = vel_ratio / 100.0 * vector_tcp2end.z / distance_to_tcp2end * (distance_to_start2tcp - ZERO_RANGE) / (distance_min - ZERO_RANGE);
        passive_force.fy = vel_ratio / 100.0 * vector_tcp2end.y / distance_to_tcp2end * (distance_to_start2tcp - ZERO_RANGE) / (distance_min - ZERO_RANGE);
        passive_force.fx = vel_ratio / 100.0 * vector_tcp2end.x / distance_to_tcp2end * (distance_to_start2tcp - ZERO_RANGE) / (distance_min - ZERO_RANGE);
    }
    else if (distance_to_tcp2end > distance_min)
    {
        passive_force.fz = vel_ratio / 100.0 * vector_tcp2end.z / distance_to_tcp2end;
        passive_force.fy = vel_ratio / 100.0 * vector_tcp2end.y / distance_to_tcp2end;
        passive_force.fx = vel_ratio / 100.0 * vector_tcp2end.x / distance_to_tcp2end;
    }
    else if (distance_to_tcp2end >= ZERO_RANGE)
    {
        passive_force.fz = vel_ratio / 100.0 * vector_tcp2end.z / distance_to_tcp2end * (distance_to_tcp2end - ZERO_RANGE) / (distance_min - ZERO_RANGE);
        passive_force.fy = vel_ratio / 100.0 * vector_tcp2end.y / distance_to_tcp2end * (distance_to_tcp2end - ZERO_RANGE) / (distance_min - ZERO_RANGE);
        passive_force.fx = vel_ratio / 100.0 * vector_tcp2end.x / distance_to_tcp2end * (distance_to_tcp2end - ZERO_RANGE) / (distance_min - ZERO_RANGE);
    }
    else
    {
        passive_force.fz = 0;
        passive_force.fy = 0;
        passive_force.fx = 0;
    }
    return passive_force;
}

double ForceControl::getPlanarPassiveForceScaleFactor(
    planar_data_type::PlanarPoint start, 
    planar_data_type::PlanarPoint end, 
    planar_data_type::PlanarPoint tcp_point_in_base,
    double distance_min
)
{
    double distance_start2tcp  = getPlanarP2PDistance(start, tcp_point_in_base);
	double distance_tcp2end = getPlanarP2PDistance(end, tcp_point_in_base);

    double scale_factor;
	if (distance_start2tcp < ZERO_RANGE)
	{
        scale_factor = 0.0;
        printf("ForceControl: formal zero\n");
	}
	else if (distance_start2tcp < distance_min)
	{
        scale_factor = pow((distance_start2tcp - ZERO_RANGE) / (distance_min - ZERO_RANGE), 0.5);
        printf("ForceControl: enlarge\n");
	}
	else if (distance_tcp2end > distance_min)
	{
        scale_factor = 1.0;
        printf("ForceControl: normal\n");
	}
	else if (distance_tcp2end >= ZERO_RANGE)
	{
        scale_factor = pow((distance_tcp2end - ZERO_RANGE) / (distance_min - ZERO_RANGE), 0.5);
        printf("ForceControl: shrunk\n");
	}
	else
	{
        scale_factor = 0.0;
        printf("ForceControl: next zero\n");
	}  
    return scale_factor;
}


double ForceControl::getCartPassiveForceScaleFactor(
    ur_data_type::CartesianPoint start,
    ur_data_type::CartesianPoint end,
    ur_data_type::CartesianPoint tcp_point_in_base,
    double distance_min
)
{
    double distance_start2tcp = getP2PDistance(start, tcp_point_in_base);
	double distance_tcp2end = getP2PDistance(tcp_point_in_base, end);

    double scale_factor;
	if (distance_start2tcp < ZERO_RANGE)
	{
        scale_factor = 0.0;
		printf("ForceControl: formal zero\n");
	}
	else if (distance_start2tcp >= ZERO_RANGE && distance_start2tcp < distance_min)
	{
        scale_factor = pow((distance_start2tcp - ZERO_RANGE) / (distance_min - ZERO_RANGE), 0.5);
		printf("ForceControl: enlarge");
	}

	else if ((distance_start2tcp >= distance_min) && (distance_tcp2end > distance_min))
	{
        scale_factor = 1.0;
		printf("ForceControl: normal\n");
	}
	else if (distance_tcp2end >= ZERO_RANGE && distance_tcp2end <= distance_min)
	{
        scale_factor = pow((distance_tcp2end - ZERO_RANGE) / (distance_min - ZERO_RANGE), 0.5);
		printf("ForceControl: shrunk\n");
	}
	else
	{
        scale_factor = 0.0;
		printf("ForceControl: next zero\n");
	}
	return scale_factor;
}

planar_data_type::PlanarForceValue ForceControl::getPlanarPassiveForceInCircle(
        planar_data_type::PlanarPoint center, float radium,
        planar_data_type::PlanarPoint tcp_point_in_base,
        float rigidity, float path_width
)
{
	assert(radium > 2 * path_width);

    double distance_tcp2center = getPlanarP2PDistance(tcp_point_in_base, center);
    PlanarPoint point_on_circle = getTargetPlanarPointInCirclePath(center, radium, tcp_point_in_base, distance_tcp2center);

    PlanarPoint vector_tcp2cirlcepoint;
    vector_tcp2cirlcepoint.x = point_on_circle.x - tcp_point_in_base.x;
    vector_tcp2cirlcepoint.y = point_on_circle.y - tcp_point_in_base.y;

	float first_level = 0.5f;
    planar_data_type::PlanarForceValue passive_force;
    double abs_distance = fabs(distance_tcp2center - radium);

    if (abs_distance < path_width)
    {
        passive_force.fx = first_level * abs_distance * 100 * vector_tcp2cirlcepoint.x;
        passive_force.fy = first_level * abs_distance * 100 * vector_tcp2cirlcepoint.y;
    }
    else if (abs_distance < 1.5*path_width)
    {
        passive_force.fx = first_level * abs_distance * 100 * vector_tcp2cirlcepoint.x 
            + rigidity * (abs_distance - path_width) * 100 * vector_tcp2cirlcepoint.x;
        passive_force.fy = first_level * abs_distance * 100 * vector_tcp2cirlcepoint.y 
            + rigidity * (abs_distance - path_width) * 100 * vector_tcp2cirlcepoint.y;   
    }
    else
    {
        passive_force.fx = first_level * abs_distance * 100 * vector_tcp2cirlcepoint.x 
                            + rigidity * (abs_distance - path_width) * 100 * vector_tcp2cirlcepoint.x 
                            + rigidity * (abs_distance - 1.5*path_width) * 100 * vector_tcp2cirlcepoint.x;
        passive_force.fy = first_level * abs_distance * 100 * vector_tcp2cirlcepoint.y 
                            + rigidity * (abs_distance - path_width) * 100 * vector_tcp2cirlcepoint.y 
                            + rigidity * (abs_distance- 1.5*path_width) * 100 * vector_tcp2cirlcepoint.y;
    }
    return passive_force;
}

void ForceControl::updatePlanarPassiveForceByMoveDirection(
    MoveDirectionInCircle move_direction,
    planar_data_type::PlanarForceValue& passive_force
)
{
	if (move_direction == CLOCKWISE)
	{
		passive_force.fx = -passive_force.fy;
		passive_force.fy = passive_force.fx;
	}
	else
	{
		passive_force.fx = passive_force.fy;
		passive_force.fy = -passive_force.fx;
	}
}


planar_data_type::PlanarPoint ForceControl::getTargetPlanarPointInCirclePath(
    planar_data_type::PlanarPoint center, 
    double radium,
    planar_data_type::PlanarPoint point_in_circle,
    double distance_tcp2center
)
{
    planar_data_type::PlanarPoint target_point;
    target_point.x = center.x + (point_in_circle.x - center.x) * radium / distance_tcp2center;
    target_point.y = center.y + (point_in_circle.y - center.y) * radium / distance_tcp2center;
    return target_point;
}

planar_data_type::PlanarForceValue getPlanarPassiveForceInLine( //MoveAlongLine
    planar_data_type::PlanarPoint start, 
    planar_data_type::PlanarPoint end,
    planar_data_type::PlanarPoint tcp_point_in_base,
    double distance_min,
    int vel_ratio
)
{
    Line line = Line(start, end);
    PlanarPoint vector_start2end;
    vector_start2end.x = end.x - start.x;
    vector_start2end.y = end.y - start.y; 

    PlanarForceValue passive_force;
    passive_force.fx = vel_ratio / 100.0 * vector_start2end.x / abs(vector_start2end.x)*abs(line.a1 / sqrt(pow(line.a1, 2) + 1));
    passive_force.fy = vel_ratio / 100.0 * vector_start2end.y / abs(vector_start2end.y)*abs(1 / sqrt(pow(line.a1, 2) + 1));

    double distance_start2tcp = getPlanarP2PDistance(start, tcp_point_in_base) < distance_min;
    if(distance_start2tcp < distance_min)
    {
        passive_force.fx *= distance_start2tcp / 0.01;
		passive_force.fy *= distance_start2tcp / 0.01;
    }

    return passive_force;
}


force_data_type::ForceValue ForceControl::getCartRandomPassiveForce(
    double max_force, double total_time)
{
    ForceValue passive_force;
    if (time_scale_ == 0)
    {
        passive_force.fx = 0.0;
        passive_force.fy = 0.0;
        passive_force.fz = 0.0;
        time_scale_++;
    }
    else if(total_time > static_cast<double>(time_scale_) * 5.0)
    {
        passive_force.fx = random_param_.x * max_force * 5.0 * static_cast<double>(time_scale_) / total_time;
        passive_force.fy = random_param_.y * max_force * 5.0 * static_cast<double>(time_scale_) / total_time;
        passive_force.fz = random_param_.z * max_force * 5.0 * static_cast<double>(time_scale_) / total_time;
        time_scale_++;
    }
    else if(total_time > static_cast<double>(time_scale_) * 1.25)
    {
        passive_force.fx = random_param_.x * max_force;
        passive_force.fy = random_param_.y * max_force;
        passive_force.fz = random_param_.z * max_force;
        time_scale_++;
    }
    else if(total_time > static_cast<double>(time_scale_) * 1.0)
    {
        passive_force.fx = random_param_.x * max_force * 5.0 * (total_time - static_cast<double>(time_scale_)) / total_time;
        passive_force.fy = random_param_.y * max_force * 5.0 * (total_time - static_cast<double>(time_scale_)) / total_time;
        passive_force.fz = random_param_.z * max_force * 5.0 * (total_time - static_cast<double>(time_scale_)) / total_time; 
        time_scale_++;
    }
    else // total_time <= time_scale_
    {
        time_scale_ = 0;
        passive_force.fx = 0.0;
        passive_force.fy = 0.0;
        passive_force.fz = 0.0;
    }
    return passive_force;
}


force_data_type::FTSensorData ForceControl::getPidOutput()
{
    return pid_output_;
}

void ForceControl::initPidOutput()
{
    pid_output_.force.fx = 0.0;
    pid_output_.force.fy = 0.0;
    pid_output_.force.fz = 0.0; 
    pid_output_.torque.tx = 0.0;
    pid_output_.torque.ty = 0.0;
    pid_output_.torque.tz = 0.0; 

    passive_force_torque_tf_a_.force.fx = 0.0;
    passive_force_torque_tf_a_.force.fy = 0.0;
    passive_force_torque_tf_a_.force.fz = 0.0;
    passive_force_torque_tf_a_.torque.tx = 0.0;
    passive_force_torque_tf_a_.torque.ty = 0.0;
    passive_force_torque_tf_a_.torque.tz = 0.0;

    passive_force_torque_tf_r_.force.fx = 0.0;
    passive_force_torque_tf_r_.force.fy = 0.0;
    passive_force_torque_tf_r_.force.fz = 0.0;
    passive_force_torque_tf_r_.torque.tx = 0.0;
    passive_force_torque_tf_r_.torque.ty = 0.0;
    passive_force_torque_tf_r_.torque.tz = 0.0;
}

void ForceControl::decreasePidOutput(double scale_factor)
{
    pid_output_ = pid_output_ * scale_factor;
    passive_force_torque_tf_a_ = passive_force_torque_tf_a_ * scale_factor;
    passive_force_torque_tf_r_ = passive_force_torque_tf_r_ * scale_factor;
}

void ForceControl::boundaryProtection(
    const CartPose tcp_target, 
    const Joint actual_joint,
    force_data_type::ForceValue& force_error
)
{
    CartPose actual_pose;
    double rotation[3][3] = {};

    ur_kine_.FK(actual_joint, actual_pose, rotation);

    double force_error_double_list[3];
    force_error.transDoubleList(force_error_double_list); 

    double force_error_in_base[3];
    vectorTransBase(force_error_double_list, rotation, 3, force_error_in_base);

    CartesianPoint vector_in_wf;
    vector_in_wf.x = calculateCoordInWf(tcp_target.point.x,  move_boundary_.boundary_max.x, move_boundary_.boundary_min.x);
    vector_in_wf.y = calculateCoordInWf(tcp_target.point.y,  move_boundary_.boundary_max.y, move_boundary_.boundary_min.y);
    vector_in_wf.z = calculateCoordInWf(tcp_target.point.z,  move_boundary_.boundary_max.z, move_boundary_.boundary_min.z);
    
    // printf("-- X --\n");
    calculateForceErrorInBase(
        tcp_target.point.x, 
        move_boundary_.boundary_max.x, 
        move_boundary_.boundary_min.x, 
        move_boundary_.error_range, 
        force_error_in_base[0]
        );

    force_error_in_base[0] += vector_in_wf.x;
    // printf("X: force_error_in_base[0] = %lf\n", force_error_in_base[0]); /* debug */

    // printf("-- Y --\n"); /* debug */
    calculateForceErrorInBase(
        tcp_target.point.y, 
        move_boundary_.boundary_max.y, 
        move_boundary_.boundary_min.y, 
        move_boundary_.error_range, 
        force_error_in_base[1]
        );

    force_error_in_base[1] += vector_in_wf.y;
    // printf("Y: force_error_in_base[1] = %lf\n", force_error_in_base[1]); /* debug */

    // printf("-- Z --\n");
    calculateForceErrorInBase(
        tcp_target.point.z, 
        move_boundary_.boundary_max.z, 
        move_boundary_.boundary_min.z, 
        move_boundary_.error_range, 
        force_error_in_base[2]
        );

    force_error_in_base[2] += vector_in_wf.z;
    //printf("Z: force_error_in_base[2] = %lf\n", force_error_in_base[2]);
    double force_error_in_tcp[3];
    vectorTransTcp(force_error_in_base, rotation, 3, force_error_in_tcp);
    force_error.fx = force_error_in_tcp[0];
    force_error.fy = force_error_in_tcp[1];
    force_error.fz = force_error_in_tcp[2];
    //force_error.printValue("boundaryProtection: force_error");
}

double ForceControl::calculateCoordInWf(double point_coord, double boundary_max, double boundary_min)
{
    if (point_coord > boundary_max)
    {
        return -pow(fabs((point_coord - boundary_max)), 2.0)*12.5 * 500;
    }
    else if(point_coord < boundary_min)
    {
         return pow(fabs((point_coord - boundary_min)), 2.0)*12.5 * 500;
    }
    else 
    {
        return 0.0;
    }
}

void ForceControl::calculateForceErrorInBase(double point_coord, double boundary_max, double boundary_min, double error_range, double &force_error)
{
    if (point_coord > boundary_max)
    {
        if (point_coord < boundary_max + error_range)
        {
            if (force_error > 0)
            {
                printf("-------------------------- 分支--1--------------------\n");
                printf("-------------------------- 分支--1.1------------------\n");
                force_error = fabs(point_coord - (boundary_max + error_range)) 
                                                / error_range * force_error 
                                        + fabs(point_coord - boundary_max) 
                                                / error_range     
                                                * atan(0.5*force_error) 
                                                / 3.1415926 
                                                * 20;
            }
        }
        else 
        {
            if (force_error > 0)
            {
                printf("-------------------------- 分支--1--------------------\n");
                printf("-------------------------- 分支--1.2------------------\n");
                force_error = atan(0.5*force_error) / 3.1415926 * 20;
            }
        }
    }

    if (point_coord < boundary_min)
    {
        printf("-------------------------- 分支--2--------------------\n");
        if (point_coord > boundary_min - error_range)
        {
            printf("-------------------------- 分支--2.1------------------\n");
            if (force_error < 0)
            {
                force_error = fabs(point_coord - boundary_min) 
                                / error_range 
                                * force_error 
                        + fabs(point_coord - boundary_min) 
                                / error_range 
                                * atan(0.5*force_error) 
                                / 3.1415926 
                                * 20;
            }
        }
        else 
        {
            printf("-------------------------- 分支--2.2------------------\n");
            if (force_error < 0)
            {
                force_error = atan(0.5*force_error) / 3.1415926 * 20;
            }
        }
    }
}

bool ForceControl::oddProtection(Joint actual_joint, Joint &speed)
{
	//选取最大角速度并锁定为1
	double speed_max = 0.0;
	for (int i = 0; i < 6; i++)
		if (speed.jVal[i] > speed_max) speed_max = speed.jVal[i];

	if (speed_max > 1)
		for (int i = 0; i < 6; i++)
			speed.jVal[i] *= 1 / speed_max;

	//求雅各比行列式并求得最小奇异值的
	double jecobian_mat[36] = {};
    ur_jacobian_.jacobi(actual_joint.jVal, jecobian_mat);

    std::vector<double> eign_value;
    eign_value.clear();
	double jecobian_mat_temp1[36] = {};
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			for (int k = 0; k < 6; k++)
				jecobian_mat_temp1[i * 6 + j] += jecobian_mat[i * 6 + k] * jecobian_mat[j * 6 + k];
		}
	}
	solveEign(jecobian_mat_temp1, eign_value);
	double eign_value_min_1 = eign_value[0];

    if (eign_value_min_1 > 0.0225)
    {
        // printf("Warnning: ForceControl: oddProtection: eign_value_min_1(%lf) > %lf\n", eign_value_min_1, 0.0225); /* debug */
        return true;
    }

    eign_value.clear();
    double jecobian_mat_temp2[36] = {};
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            for (int k = 0; k < 6; k++)
                jecobian_mat_temp2[i * 6 + j] += jecobian_mat[i * 6 + k] * jecobian_mat[j * 6 + k];
        }
    }
    solveEign(jecobian_mat_temp2, eign_value);
    double eign_value_min_2 = eign_value[0];

    //eign_value_min_1 = eign_value_min_2 ??, to check...
    //printf("ForceControl: oddProtection: eign_value_min_1, eign_value_min_2: %lf, %lf\n", eign_value_min_1, eign_value_min_2);

    //判断行进方向
    if (eign_value_min_2 - eign_value_min_1 < -DOUBLE_ZERO)
    {
        if (eign_value_min_1 < 0.0121)
        {
            for (int i = 0; i < 6; i++)
                speed.jVal[i] = 0;
        }
        else
        {
            for (int i = 0; i < 6; i++)
                speed.jVal[i] *= (eign_value_min_1 / 0.0121 - 0.0104 / 0.0121);
        }
    }
    else
    {
        for (int i = 0; i < 6; i++)
            speed.jVal[i] *= (eign_value_min_1 / 0.011 - 1) / (0.0225 / 0.011 - 1);
    }

    if (speed.jVal[3] > pow((eign_value_min_1 / 0.0225), 0.5))
    {
        for (int i = 0; i < 6; i++)
        {
            speed.jVal[i] *= pow((eign_value_min_1 / 0.0225), 0.5);
        }
    }

    return true;
}

// 为什么是jointProtection？？
void ForceControl::jointProtection( 
    const ur_data_type::CartesianPoint point, 
    const ur_data_type::Joint actual_joint,
    double* vw)
{
	double angle2 = 40 * M_PI / 180;
    double stop_angle2 = 10 * M_PI / 180;

	if (actual_joint.jVal[2] < angle2)
	{
		for (int i = 0; i < 6; i++)
		{
			vw[i] *= pow(abs((stop_angle2 - actual_joint.jVal[2]) / (stop_angle2 - angle2)), 3); //vw[0-6]相等？？
		}
		float factor_k = -0.1*abs((angle2 - actual_joint.jVal[2]) / (stop_angle2 - angle2));
		double stop_descent[3] = {factor_k*point.x, factor_k*point.y, factor_k*point.z};

        //printf("ForceControl: jointProtection: FK\n");
        CartPose actual_pose;
        double rotation[3][3] = {};
        ur_kine_.FK(actual_joint, actual_pose, rotation);
        //printf("ForceControl: jointProtection: FK ok\n");
        double stop_descent_tcp[3] = {};
        vectorTransTcp(stop_descent, rotation, 3, stop_descent_tcp);

		vw[0] += stop_descent_tcp[0];
		vw[1] += stop_descent_tcp[1];
		vw[2] += stop_descent_tcp[2];

        printf("ForceControl: jointProtection: stop_descent_tcp: %lf, %lf %lf\n",
            stop_descent_tcp[0], stop_descent_tcp[1], stop_descent_tcp[2]);
	}
}

bool ForceControl::initAssistForceInfo(
    unsigned int force_level, unsigned int rigidity_level, float assist_factor, 
    AssistForceInfo &assist_info)
{
    if (force_level > 3 || rigidity_level > 3)
    {
        printf("ForceControl: parameter out og range: force_level %d > %d, rigidity_level: %d > %d\n",
            force_level, 3, rigidity_level, 3);
        force_level = 3;
        rigidity_level = 3;
     }
 
    switch (force_level)
	{
	case 0:
    {
        assist_info.assist_force = 0 * assist_factor + 0.005; 
        assist_info.random_force = 0.005;
    }
        break;
	case 1:
    {
        assist_info.assist_force = 0.05 * assist_factor + 0.005; 
        assist_info.random_force = 0.06;
    }
        break;
	case 2:
    {
        assist_info.assist_force = 0.075 * assist_factor + 0.005; 
        assist_info.random_force = 0.07;
    }
        break;
	case 3:
    {
        assist_info.assist_force = 0.125 * assist_factor + 0.005; 
        assist_info.random_force = 0.09;
    }
        break;
    default:;
	}

	switch (rigidity_level)
	{
	case 0:
        {
            assist_info.force_rigidity[0] = 3; 
            assist_info.force_rigidity[1] = 50; 
            assist_info.force_edge[0] = 0.005; 
            assist_info.force_edge[1] = 20; 
            assist_info.force_edge[2] = 0.1; 
            assist_info.force_edge[3] = 1; 
        }
        break;
	case 1:
        {
            assist_info.force_rigidity[0] = 6; 
            assist_info.force_rigidity[1] = 100; 
            assist_info.force_edge[0] = 0.005; 
            assist_info.force_edge[1] = 10; 
            assist_info.force_edge[2] = 0.074; 
            assist_info.force_edge[3] = 0.74;
        }
        break;
	case 2:
        {
            assist_info.force_rigidity[0] = 12; 
            assist_info.force_rigidity[1] = 200; 
            assist_info.force_edge[0] = 0.005; 
            assist_info.force_edge[1] = 2; 
            assist_info.force_edge[2] = 0.047; 
            assist_info.force_edge[3] = 0.47;
        }
        break;
	case 3:
        {
            assist_info.force_rigidity[0] = 25; 
            assist_info.force_rigidity[1] = 400;
            assist_info.force_edge[0] = 0.005; 
            assist_info.force_edge[1] = 1; 
            assist_info.force_edge[2] = 0.02; 
            assist_info.force_edge[3] = 0.2;
        }
        break;
    default:;
	}
    return true;
}

bool ForceControl::calculateComplianceVW(
    force_data_type::ForceValue force_pid_output,
    force_data_type::TorqueValue torque_pid_output,
    force_data_type::FTSensorData &vw)
{
    if (fabs(force_output_.fx) < DOUBLE_ZERO
        && fabs(force_output_.fy) < DOUBLE_ZERO
        && fabs(force_output_.fz) < DOUBLE_ZERO)
    {
        vw.force = force_pid_output + passive_force_torque_tf_a_.force + passive_force_torque_tf_r_.force;
    }
    else
    {
        vw.force = force_output_ + passive_force_torque_tf_a_.force + passive_force_torque_tf_r_.force;
    }

    vw.torque = torque_pid_output + passive_force_torque_tf_r_.torque;

    if (isVWBeyondLimit(vw))
    {
        printf("Warnning: ManualMovement:vw too large, stoping.\n");
        return false;
    }

    return true;
}

void ForceControl::calculatePlaneVW(ur_data_type::Joint actual_joint, 
    force_data_type::ForceValue force_pid_output,
    force_data_type::TorqueValue torque_pid_output,
    double height,
    force_data_type::FTSensorData &vw
    )
{
    //printf("ForceControl: jointProtection: FK\n");
    CartPose actual_pose;
    double rotation[3][3] = {};
    ur_kine_.FK(actual_joint, actual_pose, rotation);
    //printf("ForceControl: jointProtection: FK ok\n");

    double stop_descent[3] = { 0, 0, 3*(height - actual_pose.point.y)};
    double stop_descent_tcp[3] = {};
    vectorTransTcp(stop_descent, rotation, 3, stop_descent_tcp);

    vw.force.fx = force_pid_output.fx + force_in_sensor_frame_[0] + force_in_sensor_frame_[2]; // 用雅克比逆矩阵做速度逆运算的力，此为tcp坐标系x轴分力
    vw.force.fy = force_pid_output.fy + force_in_sensor_frame_[1] + force_in_sensor_frame_[3]; // 用雅克比逆矩阵做速度逆运算的力，此为tcp坐标系y轴分力
    vw.force.fz = 0.02*force_pid_output.fz + stop_descent_tcp[2];
    vw.torque.tx = 0;
    vw.torque.ty = 0;
    vw.torque.tz = torque_pid_output.tz;
}

void ForceControl::calculateConstantVW(force_data_type::ForceValue force_pid_output, force_data_type::FTSensorData &vw)
{
    double sum = sqrt(pow(force_pid_output.fx, 2) + pow(force_pid_output.fy, 2));
    double max_force = 0.2;
    if (sum > max_force)
    {
        vw.force.fx = force_pid_output.fx / sum * max_force;
        vw.force.fy = force_pid_output.fy / sum * max_force;
    }
    else
    {
        vw.force.fx = force_pid_output.fx;
        vw.force.fy = force_pid_output.fy;
    }
    vw.force.fz = 0.0;
    vw.torque.tx = 0.0;
    vw.torque.ty = 0.0;
    vw.torque.tz = 0.0;
}

void ForceControl::updateForceRef(ur_data_type::Joint actual_joint, 
    int rigidity_level,
    force_data_type::ForceValue &force_ref
)
{
    //printf("ForceControl: jointProtection: FK\n");
    CartPose actual_pose;
    double rotation[3][3] = {};
    ur_kine_.FK(actual_joint, actual_pose, rotation);
    //printf("ForceControl: jointProtection: FK ok\n");

    double buoyancy_vector_in_wf[3] = {};
    buoyancy_vector_in_wf[2] = static_cast<double>(rigidity_level);

    double buoyancy_vector_in_tf[3] = {};
    vectorTransTcp(buoyancy_vector_in_wf, rotation, 3, buoyancy_vector_in_tf);

    force_ref.fx = buoyancy_vector_in_tf[0];
    force_ref.fy = buoyancy_vector_in_tf[1];
    force_ref.fz = buoyancy_vector_in_tf[2];
}

void ForceControl::updateForceRef(force_data_type::ForceValue random_disturbance_force,
    force_data_type::ForceValue &force_ref
)
{
    double disturbances[3] = {0.0, 0.0, 0.0};
    double random_disturbances[3] = {};

    random_disturbances[0] = random_disturbance_force.fx;
    random_disturbances[1] = random_disturbance_force.fy;
    random_disturbances[2] = random_disturbance_force.fz;
#if 0 // todo...
    for (int j = 0; j < 3; j++) //Random torque is disabled due to safety conserns
    {
        if ((copysign(1.0, disturbances[j]) != copysign(1.0, random_disturbances[j])) && (fabs(disturbances[j]) > 0.001))
        {
            disturbances[j] = disturbances[j] / 1.2; //Gentle step-down
        }
        else if (disturbances[j] < (random_disturbances[j] * user_parameters[j]))
        {
            disturbances[j] += fabs((random_disturbances[j] * user_parameters[j]) - disturbances[j]) / fabs((random_disturbances[j] * user_parameters[j])); //Gentle step-up
        }
    }
#endif
    force_ref.fx = disturbances[0];
    force_ref.fy = disturbances[1];
    force_ref.fz = disturbances[2];
}

int ForceControl::variableDigidity(
    planar_data_type::PlanarPoint start, 
    planar_data_type::PlanarPoint end, 
    planar_data_type::PlanarPoint tcp_point_in_base, 
    double rigidity,
    double path_width,
    planar_data_type::PlanarForceValue &force
)
{
static int num_lose = 0;
static int flag_lose = 400;

    force.fx = 0.0;
    force.fy = 0.0;
	float first_level = 0.5f;
	std::pair<CuttingDirection, float> vector_return(CuttingDirection::POSITIVE, 0);

	Line line = Line(start, end);

    PlanarPoint foot_point;
    foot_point.x = (pow(line.a1, 2)*tcp_point_in_base.x - line.a0*line.a1*tcp_point_in_base.y - line.a0*line.a2) / (pow(line.a0, 2) + pow(line.a1, 2));
    foot_point.y = (pow(line.a0, 2)*tcp_point_in_base.y - line.a0*line.a1*tcp_point_in_base.x - line.a1*line.a2) / (pow(line.a0, 2) + pow(line.a1, 2));

    PlanarPoint point_in_wp = PlanarPoint(foot_point.x-tcp_point_in_base.x, foot_point.y-tcp_point_in_base.y);
    double distance_tcp2foot = sqrt(pow(point_in_wp.x, 2) + pow(point_in_wp.y, 2));

	if (distance_tcp2foot > path_width)
	{
		if (distance_tcp2foot > 1.5*path_width)
		{
			force.fx = first_level * distance_tcp2foot * 100*point_in_wp.x 
                        + rigidity * (distance_tcp2foot - path_width) * 100 * point_in_wp.x
				        + rigidity * (distance_tcp2foot - 1.5*path_width) * 100 * point_in_wp.x;
			force.fy = first_level * distance_tcp2foot * 100 * point_in_wp.y 
                        + rigidity * (distance_tcp2foot - path_width) * 100 * point_in_wp.y
				        + rigidity * (distance_tcp2foot - 1.5*path_width) * 100 * point_in_wp.y;
			num_lose++;
		}
		else
		{
			force.fx = first_level * point_in_wp.x * distance_tcp2foot * 100 
                        + rigidity * (distance_tcp2foot - path_width) * 100 * point_in_wp.x;
			force.fy = first_level * point_in_wp.y * distance_tcp2foot * 100 
                        + rigidity * (distance_tcp2foot - path_width) * 100 * point_in_wp.y;
		}

		if (num_lose >= flag_lose)
		{
			num_lose = 0;
			vector_return.first = CuttingDirection::NEGATIVE;
			return vector_return.first;
		}
	}
	else
	{
		num_lose = 0;
		force.fx = first_level * distance_tcp2foot * 100 * point_in_wp.x;
		force.fy = first_level * distance_tcp2foot * 100 * point_in_wp.y;
	}

	return vector_return.first;
}


force_data_type::ForceValue ForceControl::updateEndEffectorGravityInTf(double rotation[3][3])
{
    double end_effector_gravity_in_tf[3];

    double end_effector_gravity_in_wf_vector[3];
    end_effector_gravity_in_wf_vector[0] = end_effector_gravity_in_wf_.fx;
    end_effector_gravity_in_wf_vector[1] = end_effector_gravity_in_wf_.fy;
    end_effector_gravity_in_wf_vector[2] = end_effector_gravity_in_wf_.fz; 
    vectorTransTcp(end_effector_gravity_in_wf_vector, rotation, 3, end_effector_gravity_in_tf);

    force_data_type::ForceValue end_effector_gravity;
    end_effector_gravity.fx = end_effector_gravity_in_tf[0];
    end_effector_gravity.fy = end_effector_gravity_in_tf[1];   
    end_effector_gravity.fz = end_effector_gravity_in_tf[2];
    return end_effector_gravity;
}

void ForceControl::calculateEndEffectorForceBias(double rotation[3][3], force_data_type::ForceValue sensor_force)
{
    end_effector_gravity_in_tf_ = updateEndEffectorGravityInTf(rotation);

   force_bias_ = sensor_force - end_effector_gravity_in_tf_;
}

force_data_type::ForceValue ForceControl::calculateForce(double rotation[3][3], force_data_type::ForceValue sensor_force)
{
    memset(&end_effector_gravity_in_tf_, 0, sizeof(end_effector_gravity_in_tf_));
    end_effector_gravity_in_tf_ = updateEndEffectorGravityInTf(rotation);

    return sensor_force - (force_bias_ + end_effector_gravity_in_tf_);
}

bool ForceControl::isForceBeyondLimit(force_data_type::ForceValue force)
{
    double force_abs = sqrt(pow(force.fx, 2) + pow(force.fy, 2) + pow(force.fz, 2));
    if(force_abs > FORCE_ABS_MAX)
    {
        printf("ForceControl: Absolute value of force out of range, greater than %lf: the value is %lf\n",
            FORCE_ABS_MAX, force_abs);
        return true;
    }
    return false;
}

force_data_type::ForceValue ForceControl::calculateForceError(force_data_type::ForceValue force_error)
{
    double force_error_abs = force_error.getForceAbs();

    if (force_error_abs < FORCE_ERROR_ABS_MIN) // Dead-band filter, cut-off at 1N
    {
        force_error.fx = 0.0; //Gentle step-down
        force_error.fy = 0.0;
        force_error.fz = 0.0;
    }
    else
    {
        double error_ratio_temp = (force_error_abs - FORCE_ERROR_ABS_MIN) / force_error_abs;
        double error_ratio = (atan(M_PI / 60 * force_error_abs * error_ratio_temp) / M_PI * 60) / force_error_abs;
        force_error = force_error * error_ratio;
    }
}

void ForceControl::calculateForceError(force_data_type::ForceValue force_error_pre, force_data_type::ForceValue &force, force_data_type::ForceValue &force_error)
{
    double force_abs = force.getForceAbs();
    if (force_abs < FORCE_ABS_MIN) // Dead-band filter, cut-off at 1N
    {
        force_error = force_error_pre * (1.0/DESCENDING_GRADIENT);
    }
    else
    {
        force = force * ((force_abs - FORCE_ABS_MIN) / force_abs);
        force_error = force_ref_ + force;

        double force_error_abs = force_error.getForceAbs();
        double force_derivative = atan(M_PI / 30 * force_error_abs) / M_PI * 30;
        if(force_derivative < 2.0)
        {
            force_derivative = pow(force_derivative/2, 2);
        }
        else 
        {
            force_derivative -= 1.0;
        }

        force_error = force_error * (force_derivative / force_error_abs);
    }
}

void ForceControl::calculateForceError(force_data_type::ForceValue &force, force_data_type::ForceValue &force_error)
{
    calculateForceError(force_error_pre_, force, force_error); //force_error_pre_ = error_Fx_temp
    force_error_pre_ = force_error;
    // force_error.printValue("force_error:"); /* debug */
}

void ForceControl::calculateTorqueError(force_data_type::TorqueValue torque, force_data_type::TorqueValue &torque_error)
{
    if (fabs(torque.tx) < 0.03
        && fabs(torque.ty) < 0.03
        && fabs(torque.tz) < 0.03)
    {
        torque_error = torque_error * (1/DESCENDING_GRADIENT);
    }
    else 
    {
        torque_error = torque;
    }
}

bool ForceControl::isErrorBeyondLimit(force_data_type::ForceValue force_error, force_data_type::TorqueValue torque_error)
{
    if (fabs(force_error.fx) > force_limit_max_.fx
        || fabs(force_error.fy) > force_limit_max_.fy
        || fabs(force_error.fz) > force_limit_max_.fz)
    {
        return true;
    }

    if (fabs(torque_error.tx) > torque_limit_max_.tx 
        || fabs(torque_error.ty) > torque_limit_max_.ty 
        || fabs(torque_error.tz) > torque_limit_max_.tz)
    {
        return true;
    }

    return false;
}

void ForceControl::updateForceError(force_data_type::ForceValue &force_error)
{
    ForceValue force_pid_error = force_error - force_error_pre_1_; // force_error_pre_1_ = error_Fx_temp1
    force_error = force_pid_error * 0.15 + force_error_pre_1_ + (force_pid_error - force_pid_error_pre_) * 0.5;
    force_pid_error_pre_ = force_pid_error;

    force_error_pre_1_ = force_error;
}

void ForceControl::calculatePidOutput(
    force_data_type::ForceValue force_error, 
    force_data_type::TorqueValue torque_error, 
    force_data_type::ForceValue &force_output,
    force_data_type::TorqueValue &torque_output
    )
{
    ForceValue force_integrator = force_error;
    ForceValue force_derivator = force_error - force_error_pre_2_;
    force_output = force_error * force_pid_.kp + force_integrator * force_pid_.ki + force_derivator * force_pid_.kd;

    TorqueValue torque_integrator = torque_error;
    TorqueValue torque_derivator = torque_error - torque_error_pre_;
    torque_output = torque_error * torque_pid_.kp + torque_integrator * torque_pid_.ki + torque_derivator * torque_pid_.kd;

    force_error_pre_2_ = force_error;
    torque_error_pre_ = torque_error;
}


bool ForceControl::isPidOutputBeyondLimit(force_data_type::ForceValue force_pid_output, force_data_type::TorqueValue torque_pid_output)
{
    if (fabs(force_pid_output.fx) > force_pid_output_limit_max_.fx
        || fabs(force_pid_output.fy) > force_pid_output_limit_max_.fy
        || fabs(force_pid_output.fz) > force_pid_output_limit_max_.fz)
    {
        printf("force_pid_output out limit: %lf, %lf, %lf\n", force_pid_output.fx, force_pid_output.fy, force_pid_output.fz);
        return true;
    }

    if (fabs(torque_pid_output.tx) > torque_pid_output_limit_max_.tx 
        || fabs(torque_pid_output.ty) > torque_pid_output_limit_max_.ty 
        || fabs(torque_pid_output.tz) > torque_pid_output_limit_max_.tz)
    {
        printf("torque_pid_output out limit: %lf, %lf, %lf\n", torque_pid_output.tx, torque_pid_output.ty, torque_pid_output.tz);
        return true;
    }

    return false;
}


bool ForceControl::isVWBeyondLimit(force_data_type::FTSensorData &vw)
{
    if (fabs(vw.force.fy) > 1.15)
    {
        if (vw.force.fy < 0) vw.force.fy = -1.15;
        else vw.force.fy = 1.15; 
    }

    if (fabs(vw.force.fz) > 1.15)
    {
        if (vw.force.fz < 0) vw.force.fz = -1.15;
        else vw.force.fz = 1.15; 
    }

    if (vw.force.fx*vw.force.fx + vw.force.fy*vw.force.fy + vw.force.fz*vw.force.fz > 5.0) return true;
    return false;
}

bool ForceControl::calculatePidOutput(
    const ur_data_type::CartPose actual_tcp, 
    const ur_data_type::Joint actual_joint,
    double rotation[3][3], 
    force_data_type::FTSensorData sensor_data,
    ForceValue &force_pid_output,
    TorqueValue &torque_pid_output
    )
{
    ForceValue force = calculateForce(rotation, sensor_data.force);
    
    ForceValue force_error;
    calculateForceError(force, force_error); //force_error_pre_ = error_Fx_temp
    boundaryProtection(actual_tcp, actual_joint, force_error); 

    TorqueValue torque_error;
    calculateTorqueError(sensor_data.torque, torque_error);

    if (isErrorBeyondLimit(force_error, torque_error)) 
    {
        printf("Warnning: ManualMovement: Force or torque levels too large, stoping.\n");
        return false;
    }

    updateForceError(force_error);
    calculateForceError(force_error);
    calculatePidOutput(force_error, torque_error, force_pid_output, torque_pid_output);

    if (isPidOutputBeyondLimit(force_pid_output, torque_pid_output))
    {
        printf("Warnning: ManualMovement: Force or torque pid output levels too large, stoping.\n");
        return false;
    }

    return true;
}

void ForceControl::updateForceInToolFrame(ur_data_type::Joint actual_joint)
{
    double passive_force_base[6] = { passive_force_.fx, passive_force_.fy, passive_force_.fz, 0.0, 0.0, 0.0};

    CartPose actual_pose;
    double rotation[3][3] = {};
    ur_kine_.FK(actual_joint, actual_pose, rotation);

	double passive_force_tcp[6] = {};
    vectorTransTcp(passive_force_base, rotation, 3, passive_force_tcp);

	force_in_sensor_frame_[0] = passive_force_tcp[0];
	force_in_sensor_frame_[1] = passive_force_tcp[1];
	force_in_sensor_frame_[2] = passive_force_tcp[2];
	force_in_sensor_frame_[3] = 0;

	passive_force_torque_tf_r_.force.fx = passive_force_tcp[0];
	passive_force_torque_tf_r_.force.fy = passive_force_tcp[1];
	passive_force_torque_tf_r_.force.fz = passive_force_tcp[2];
}
