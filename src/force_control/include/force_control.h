
#ifndef FORCE_CONTROL_H
#define FORCE_CONTROL_H

#include <vector>
#include <assert.h> 
#include <cstring>

#include "force_datatype.h"
#include "planar_datatype.h"
#include "ur_datatype.h"
#include "math_calculate.h"
#include "ur_kinematics.h"
#include "solve_jacobian.h"

#define ZERO_RANGE (0.005)

#define FORCE_ABS_MAX (80.0)
#define FORCE_ABS_MIN (0.2)
#define FORCE_ERROR_ABS_MIN (0.05)
#define DESCENDING_GRADIENT (1.2)

class ForceControl 
{
public:
    ForceControl(); 
    ~ForceControl(); 
    planar_data_type::PlanarForceValue setPlanarPassiveForce(planar_data_type::PlanarForceValue force);
    planar_data_type::PlanarForceValue getPlanarPassiveForce();
    force_data_type::ForceValue setPassiveForce(force_data_type::ForceValue force);

    /************ Cartesian alg **********************/
    force_data_type::ForceValue getPassiveForce();
    force_data_type::ForceValue getCartPassiveForce( //MoveToPointFrom
        ur_data_type::CartesianPoint start, 
        ur_data_type::CartesianPoint end, 
        ur_data_type::CartesianPoint tcp_point_in_base,
        int vel_ratio,
        double distance_min
    );
    double getCartPassiveForceScaleFactor( //AddSlowForce
        ur_data_type::CartesianPoint start,
        ur_data_type::CartesianPoint end,
        ur_data_type::CartesianPoint tcp_point_in_base,
        double distance_min
    );

    force_data_type::ForceValue getCartRandomPassiveForce(  //RandomForce
        double max_force, double total_time);

    /************ planar alg **********************/
    planar_data_type::PlanarForceValue getPlanarPassiveForce( // MoveToPoint
        planar_data_type::PlanarPoint target, 
        planar_data_type::PlanarPoint tcp_point_in_base,
        int vel_ratio,
        double distance_min
    ); 
    planar_data_type::PlanarForceValue getPlanarPassiveForce( //MoveToPointFrom
        planar_data_type::PlanarPoint start, 
        planar_data_type::PlanarPoint end, 
        planar_data_type::PlanarPoint tcp_point_in_base,
        int vel_ratio,
        double distance_min
    );
    double getPlanarPassiveForceScaleFactor( //AddSlowForce
        planar_data_type::PlanarPoint start, 
        planar_data_type::PlanarPoint end, 
        planar_data_type::PlanarPoint tcp_point_in_base,
        double distance_min
    );

    planar_data_type::PlanarForceValue getPlanarPassiveForceInCircle( //MoveToCircle
        planar_data_type::PlanarPoint center, float radium,
        planar_data_type::PlanarPoint tcp_point_in_base,
        float rigidity, float path_width
    );
    void updatePlanarPassiveForceByMoveDirection( // MoveAlongCircle
        planar_data_type::MoveDirectionInCircle move_direction,
        planar_data_type::PlanarForceValue& passive_force
    );

    planar_data_type::PlanarForceValue getPlanarPassiveForceInLine( //MoveAlongLine
        planar_data_type::PlanarPoint start, 
        planar_data_type::PlanarPoint end,
        int vel_ratio
    );

    /******************** normal *********************/
    force_data_type::FTSensorData getPidOutput(); // GetForceAndTorque
    void initPidOutput(); //SetForceAndTorque
    void decreasePidOutput(double scale_factor); //DecreaseForceAndTorque


    bool oddProtection(ur_data_type::Joint actual_joint, ur_data_type::Joint &speed); // Manual_Movement
    void jointProtection( // Manual_Movement
        const ur_data_type::CartesianPoint point, 
        const ur_data_type::Joint actual_joint,
        double* vw);

    bool initAssistForceInfo(unsigned int force_level, unsigned int rigidity_level, float assist_factor, 
        force_data_type::AssistForceInfo &assist_info);

	bool isVWBeyondLimit(force_data_type::FTSensorData &vw);
    bool calculateComplianceVW( // Manual_Movement
        force_data_type::ForceValue force_pid_output,
        force_data_type::TorqueValue torque_pid_output,
        force_data_type::FTSensorData &vw);

    void calculatePlaneVW(ur_data_type::Joint actual_joint,  // Manual_Movement
        force_data_type::ForceValue force_pid_output,
        force_data_type::TorqueValue torque_pid_output,
        double height,
        force_data_type::FTSensorData &vw
        );
    void calculateConstantVW(force_data_type::ForceValue force_pid_output, force_data_type::FTSensorData &vw); // Manual_Movement

    void updateForceRef(ur_data_type::Joint actual_joint,  // Manual_Movement
        int rigidity_level,
        force_data_type::ForceValue &force_ref
        );
    void updateForceRef(force_data_type::ForceValue random_disturbance_force, // Manual_Movement
        force_data_type::ForceValue &force_ref
        );

    int variableDigidity(
        planar_data_type::PlanarPoint start, 
        planar_data_type::PlanarPoint end, 
        planar_data_type::PlanarPoint tcp_point_in_base, 
        double rigidity,
        double path_width,
        planar_data_type::PlanarForceValue &force
    );

    void calculateEndEffectorForceBias(double rotation[3][3], force_data_type::ForceValue sensor_force);
    bool isForceBeyondLimit(force_data_type::ForceValue force);

    void initPidFactor();
    void updatePidFactors(unsigned int force_level);
    bool calculatePidOutput(
        const ur_data_type::CartPose actual_tcp, 
        const ur_data_type::Joint actual_joint,
        double rotation[3][3], 
        force_data_type::FTSensorData sensor_data,
        force_data_type::ForceValue &force_pid_output,
        force_data_type::TorqueValue &torque_pid_output
        );

    /* for games */
    void updateForceInToolFrame(ur_data_type::Joint actual_joint); //ForceInWpToSensor
private:
    force_data_type::ForceValue passive_force_;
    ur_data_type::CartPose tcp_pose_in_base_;
    ur_data_type::CartPose tcp_pose_in_workpiece_;
    ur_data_type::CartesianPoint random_param_;
    double double_accuracy_;
    int time_scale_;
    double force_in_sensor_frame_[4];

    force_data_type::MoveBoundary move_boundary_;

    force_data_type::FTSensorData passive_force_torque_tf_a_; // passive_forcea_TF: ??
    force_data_type::FTSensorData passive_force_torque_tf_r_; // passive_forcer_TF: ??
    force_data_type::FTSensorData pid_output_;
	force_data_type::ForceValue force_pid_error_pre_;
	force_data_type::PidFactors force_pid_;
	force_data_type::PidFactors torque_pid_;

    force_data_type::ForceValue end_effector_gravity_in_wf_;
	force_data_type::ForceValue end_effector_gravity_in_tf_; //bias_tool_TF
    force_data_type::ForceValue force_bias_;
	force_data_type::ForceValue force_ref_; // references
    force_data_type::ForceValue force_output_;

	force_data_type::ForceValue force_error_pre_; // error_fx_temp
	force_data_type::ForceValue force_error_pre_1_; // error_Fx_temp1
	force_data_type::ForceValue force_error_pre_2_; // prior_error_Force
	force_data_type::TorqueValue torque_error_pre_; // prior_error_T

	force_data_type::ForceValue force_limit_max_;
	force_data_type::TorqueValue torque_limit_max_;

	force_data_type::ForceValue force_pid_output_limit_max_;
	force_data_type::TorqueValue torque_pid_output_limit_max_;

    UrKinematics ur_kine_;
    SloveJacobian ur_jacobian_;

    planar_data_type::PlanarPoint getTargetPlanarPointInCirclePath( //PointOnCircle
        planar_data_type::PlanarPoint center, 
        double radium,
        planar_data_type::PlanarPoint point_in_circle,
        double distance_tcp2center
    );

    double calculateCoordInWf(double point_coord, double boundary_max, double boundary_min);
    force_data_type::ForceValue calculateForce(double rotation[3][3], force_data_type::ForceValue sensor_force);
    void calculateForceErrorInBase(double point_coord, double boundary_max, double boundary_min, double error_range, double &force_error);

    force_data_type::ForceValue updateEndEffectorGravityInTf(double rotation[3][3]);
    void calculateEndEffectorForceBias(double rotation[3][3], force_data_type::FTSensorData sensor_data);

    void calculateForceError(force_data_type::ForceValue force_error_pre, force_data_type::ForceValue &force, force_data_type::ForceValue &force_error);

    force_data_type::ForceValue calculateForceError(force_data_type::ForceValue force_error);
    void calculateForceError(force_data_type::ForceValue &force, force_data_type::ForceValue &force_error);
    void calculateTorqueError(force_data_type::TorqueValue torque, force_data_type::TorqueValue &torque_error);

    bool isErrorBeyondLimit(force_data_type::ForceValue force_error, force_data_type::TorqueValue torque_error);

    void updateForceError(force_data_type::ForceValue &force_error);

    void calculatePidOutput(
		force_data_type::ForceValue force_error, 
		force_data_type::TorqueValue torque_error, 
		force_data_type::ForceValue &force_output,
		force_data_type::TorqueValue &torque_output
		);
	bool isPidOutputBeyondLimit(force_data_type::ForceValue force_pid_output, force_data_type::TorqueValue torque_pid_output);

    void boundaryProtection( //EdgeProtection // Manual_Movement
        const ur_data_type::CartPose tcp_target, 
        const ur_data_type::Joint actual_joint,
        force_data_type::ForceValue& force_error
    );
};

#endif


