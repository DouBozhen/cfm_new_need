

#include "arm_state.h"
using namespace std;
using namespace ur_data_type;


ArmState::ArmState()
{
    global_params_names_.clear();
}

ArmState::~ArmState()
{

}

void ArmState::setProtocolVersion(double ver)
{
    protocol_version_ = ver;
}

double ArmState::getProtocolVersion()
{
    return protocol_version_;
}

ur_data_type::Joint ArmState::getActualJoint() const
{
    return joint_data_.actual_joint;
}

ur_data_type::Joint ArmState::getActualJointVel() const
{
    return joint_data_.actual_joint_vel;
}

ur_data_type::Joint ArmState::getActualJointCurrent() const
{
    Joint current;
    for (int j = 0; j != JOINT_NUM; ++j)
    {
        current.jVal[j] = static_cast<double>(joint_data_.actual_current[j]);
    }
    return current;
}

ur_data_type::Joint ArmState::getActualJointVoltage() const
{
    Joint voltage;
    for (int j = 0; j != JOINT_NUM; ++j)
    {
        voltage.jVal[j] = static_cast<double>(joint_data_.actual_voltage[j]);
    }
    return voltage;
}

ur_data_type::Joint ArmState::getTargetJoint() const
{
    return joint_data_.target_joint;
}

ur_data_type::Joint ArmState::getTmotor()
{
    Joint t_motor;
    for (int j = 0; j != JOINT_NUM; ++j)
    {
        t_motor.jVal[j] = static_cast<double>(joint_data_.t_motor[j]);
    }
    return t_motor;
}

ur_data_type::Joint ArmState::getTmicro()
{
    Joint t_micro;
    for (int j = 0; j != JOINT_NUM; ++j)
    {
        t_micro.jVal[j] = static_cast<double>(joint_data_.t_micro[j]);
    }
    return t_micro;
}

std::array<uint8_t, 6> ArmState::getAllJointMode()
{
    return joint_data_.joint_mode;
}

ur_data_type::CartPose ArmState::getToolPose()
{
    return cart_info_.tool_pose;
}

double ArmState::getToolPointX()
{
    return cart_info_.tool_pose.point.x;
}

double ArmState::getToolPointY()
{
    return cart_info_.tool_pose.point.y;
}

double ArmState::getToolPointZ()
{
    return cart_info_.tool_pose.point.z;
}

double ArmState::getRpyRx()
{
    return cart_info_.tool_pose.rpy.rx;
}

double ArmState::getRpyRy()
{
    return cart_info_.tool_pose.rpy.ry;
}

double ArmState::getRpyRz()
{
    return cart_info_.tool_pose.rpy.rz;
}

double ArmState::getTcpOffsetPointX()
{
   return cart_info_.tcp_offset.point.x;
}

ur_data_type::CartPose ArmState::getTcpOffsetPose()
{
    return cart_info_.tcp_offset;
}

double ArmState::getTcpOffsetPointY()
{
   return cart_info_.tcp_offset.point.y;
}

double ArmState::getTcpOffsetPointZ()
{
   return cart_info_.tcp_offset.point.z;
}

double ArmState::getTcpOffsetRpyRx()
{
    return cart_info_.tcp_offset.rpy.rx;
}

double ArmState::getTcpOffsetRpyRy()
{
    return cart_info_.tcp_offset.rpy.ry;
}

double ArmState::getTcpOffsetRpyRz()
{
    return cart_info_.tcp_offset.rpy.rz;
}

int ArmState::getDigitalInputs()
{
    return mb_data_.digital_inputs;
}

int ArmState::getDigitalOutputs()
{
    return mb_data_.digital_outputs;
}

double ArmState::getAnalogInput(int index)
{
    if(index == 0)
    {
        return mb_data_.analog_input_0;
    }
    else
    {
        return mb_data_.analog_input_1;
    }
}

char ArmState::getAnalogInputRange(int index)
{
    if(index == 0)
    {
        return mb_data_.analog_input_range_0;
    }
    else
    {
        return mb_data_.analog_input_range_1;
    }
}

double ArmState::getAnalogOutput(int index)
{
    if(index == 0)
    {
        return mb_data_.analog_output_0;
    }
    else
    {
        return mb_data_.analog_output_1;
    }
}

char ArmState::getAnalogOutputDomain(int index)
{
    if(index == 0)
    {
        return mb_data_.analog_output_domain_0;
    }
    else
    {
        return mb_data_.analog_output_domain_1;
    }
}

int ArmState::getEuromapInputs()
{
    return mb_data_.euromap_inputs;
}

int ArmState::getEuromapOutputs()
{
    return mb_data_.euromap_outputs;
}

float ArmState::getEuromapVoltage()
{
    return mb_data_.euromap_voltage_24V;
}

float ArmState::getEuromapCurrent()
{
    return mb_data_.euromap_current;
}

char ArmState::isEuromap67Installed()
{
    return mb_data_.is_euromap67_installed;
}

uint8_t ArmState::getInReduceMode()
{
    return mb_data_.in_reduced_mode;
}

uint8_t ArmState::getOpModeInput()
{
    return mb_data_.op_mode_selector_input;
}

uint8_t ArmState::get3PositionEnablingSwitch()
{
    return mb_data_.three_position_enabling_input;
}

float ArmState::getMasterBoardTemperature()
{
    return mb_data_.mb_temperature;
}

float ArmState::getRobotVoltage48V()
{
    return mb_data_.robot_voltage_48V;
}

float ArmState::getRobotCurrent()
{
    return mb_data_.robot_current;
}

float ArmState::getMasterIOCurrent()
{
    return mb_data_.master_io_current;
}

unsigned char ArmState::getSafetyMode()
{
    return mb_data_.safety_mode;
}

ur_data_type::Joint ArmState::getJointLimitMin() const
{
    return config_data_.joint_limit_min;
}

ur_data_type::Joint ArmState::getJointLimitMax() const
{
    return config_data_.joint_limit_max;
}

ur_data_type::Joint ArmState::getJointMaxVel() const
{
    return config_data_.joint_vel_max;
}

ur_data_type::Joint ArmState::getJointMaxAcc() const
{
    return config_data_.joint_acc_max;
}

double ArmState::getJointVDefault()
{
    return config_data_.joint_v_default;
}

double ArmState::getJointADefault()
{
    return config_data_.joint_a_default;
}

double ArmState::getToolVDefault()
{
    return config_data_.tool_v_default;
}

double ArmState::getToolADefault()
{
    return config_data_.tool_a_default;
}

double ArmState::getEqRad()
{
    return config_data_.eq_rad;
}

#if 0
ur_data_type::DHParam ArmState::getKineDHParam()
{
    return config_data_.dh_param;
}
#endif
int ArmState::getMasterBoardVersion()
{
    return config_data_.mb_version;
}

int ArmState::getControllerBoxType()
{
    return config_data_.controller_box_type;
}

int ArmState::getRobotType()
{
    return config_data_.robot_type;
}

int ArmState::getRobotSubType()
{
    return config_data_.robot_subtype;
}

void ArmState::setDisconnected()
{
    robot_mode_.is_connected = false;
	robot_mode_.is_enabled = false;
	robot_mode_.is_powered_on = false;
}

bool ArmState::isRobotConnected()
{
    return robot_mode_.is_connected;
}

bool ArmState::isRobotEnabled()
{
    return robot_mode_.is_enabled;
}

bool ArmState::isRobotPoweredOn()
{
    return robot_mode_.is_powered_on;
}

bool ArmState::isEmergencyStopped()
{
    return robot_mode_.is_emergency_stopped;
}

bool ArmState::isProtectiveStopped()
{
    return robot_mode_.is_Protective_stopped;
}

bool ArmState::isProgramRunning()
{
    return robot_mode_.is_program_running;
}

bool ArmState::isProgramPaused()
{
    return robot_mode_.is_program_paused;
}

unsigned char ArmState::getRobotMode()
{
    return robot_mode_.robot_mode;
}

unsigned char ArmState::getControlMode()
{
    return robot_mode_.control_mode;
}
double ArmState::getTargetSpeedFraction()
{
    return robot_mode_.target_speed_fraction;
}

double ArmState::getSpeedScaling()
{
    return robot_mode_.speed_scaling;
}

double ArmState::getTargetSpeedFractionLimit()
{
    return robot_mode_.target_speed_fraction_limit;
}

unsigned char ArmState::getToolAnalogInputRange(int index)
{
    if(index == 0)
    {
        return tool_data_.analog_input_range_0;
    }
    else
    {
        return tool_data_.analog_input_range_0;
    } 
}
double ArmState::getToolAnalogInput(int index)
{
    if(index == 0)
    {
        return tool_data_.analog_input_0;
    }
    else
    {
        return tool_data_.analog_input_0;
    } 
}
float ArmState::getToolVoltage48V()
{
    return tool_data_.tool_voltage_48V;       
}
unsigned char ArmState::getToolOutputVoltage()
{
    return tool_data_.tool_output_voltage;    
}
float ArmState::getToolCurrent()
{
    return tool_data_.tool_current;    
}
float ArmState::getToolTemperature()
{
    return tool_data_.tool_temperature;   
}
unsigned char ArmState::getToolMode()
{
    return tool_data_.tool_mode;   
}

std::array<unsigned int, 6> ArmState::getKineCheckSum()
{
    return kine_info_.check_sum;
}

DHParam ArmState::getKineDHParam()
{
    return kine_info_.dh_param;
}

ur_data_type::CartPose ArmState::getForce()
{
    return force_mode_.force;
}

double ArmState::getForcePointX()
{
    return force_mode_.force.point.x;
}

double ArmState::getForcePointY()
{
  return force_mode_.force.point.y;
}

double ArmState::getForcePointZ()
{
  return force_mode_.force.point.z;
}

double ArmState::getForceRpyRx()
{
  return force_mode_.force.rpy.rx;
}

double ArmState::getForceRpyRy()
{
  return force_mode_.force.rpy.ry;
}

double ArmState::getForceRpyRz()
{
  return force_mode_.force.rpy.rz;
}

double ArmState::getForceDexterity()
{
  return force_mode_.dexterity;
}

bool ArmState::isFreeDrivePressed()
{
    return additional_info_.freedrive_button_pressed;
}

bool ArmState::isFreeDriveEnabled()
{
    return additional_info_.freedrive_button_enabled;
}

bool ArmState::isFreeDriveIOEnabled()
{
    return additional_info_.freedrive_io_enabled;
}

int8_t ArmState::getSource()
{
    return version_msg_.source;
}

std::string ArmState::getProjectName()
{
    return string(version_msg_.project_name);
}

std::string ArmState::getVersion()
{
    char version[16];
    sprintf(version, "%d.%d\n", version_msg_.major_version, version_msg_.minor_version); 
    return string(version);
}

std::string ArmState::getBugfixVersion()
{
    char version[16];
    sprintf(version, "%d\n", version_msg_.svn_revision); 
    return string(version);
}

int8_t ArmState::getSafetyModeType()
{
    return safety_mode_msg_.safety_mode_type;
}

std::string ArmState::getSafetyTextMsg()
{
    return safety_mode_msg_.text_msg;
}

int ArmState::getRobotCommWarningLevel()
{
    return robot_comm_msg_.warning_level;
}
std::string ArmState::getRobotCommText()
{
    return robot_comm_msg_.text_msg;
}

std::string ArmState::getKeyTitle()
{
    return key_msg_.key_msg_title;
}

std::string ArmState::getKeyText()
{
    return key_msg_.key_text_msg;
}

int ArmState::getLabelId()
{
    return lable_msg_.id; 
}
std::string ArmState::getLabelText()
{
    return lable_msg_.label_text_msg; 
}

bool ArmState::getPopupWarning()
{
    return popup_msg_.warning; 
}

bool ArmState::getPopupError()
{
    return popup_msg_.error; 
}

bool ArmState::getPopupBlocking()
{
    return popup_msg_.blocking; 
}

std::string ArmState::getPopupTitle()
{
    return popup_msg_.popup_msg_title; 
}
std::string ArmState::getPopupText()
{
    return popup_msg_.popup_text_msg; 
}

uint16_t ArmState::getSetupGlobalParamsStartIndex()
{
    return global_setup_msg_.start_index;
}
std::vector<std::string> ArmState::getSetupGlobalParamsNames()
{
    return global_params_names_;
}

uint8_t ArmState::getUpdateGlobalParamsStartIndex()
{
    return global_update_msg_.start_index;
}

uint8_t ArmState::getUpdateGlobalParamsValueType()
{
    return global_update_msg_.value_type;
}


std::string ArmState::getUpdateGlobalParamsStringValue()
{
    return global_update_msg_.string_value;
}
std::string ArmState::getUpdateGlobalParamsStringValueVar()
{
    return global_update_msg_.var_string_value;
}
int16_t ArmState::getUpdateGlobalParamsListLength()
{
    return global_update_msg_.list_length;
}
uint8_t ArmState::getUpdateGlobalParamsListValType()
{
    return global_update_msg_.list_val_type;
}

ur_data_type::CartPose ArmState::getUpdateGlobalParamsPose()
{
    return global_update_msg_.pose;
}

bool ArmState::getUpdateGlobalParamsBool()
{
    return global_update_msg_.bool_val;
}
float ArmState::getUpdateGlobalParamsNum()
{
    return global_update_msg_.num_value;
}
int32_t ArmState::getUpdateGlobalParamsInt()
{
    return global_update_msg_.int_val;
}
float ArmState::getUpdateGlobalParamsFloat()
{
    return global_update_msg_.float_val;
}

bool ArmState::getRequestWarning()
{
    return request_msg_.warning;
}

bool ArmState::getRequestError()
{
    return request_msg_.error;
}

bool ArmState::getRequestBlocking()
{
    return request_msg_.blocking;
}

std::string ArmState::getRequesteTitle()
{
    return request_msg_.decode_vector_title;
}

std::string ArmState::getRequestText()
{
    return request_msg_.request_value_text_msg;
}

std::string ArmState::getText()
{
    return text_msg_.text_msg;
}

uint32_t ArmState::getRuntimeLineNum()
{
    return runtime_except_msg_.line_num;
}

uint32_t ArmState::getRuntimeColumnNum()
{
    return runtime_except_msg_.column_num;
}

std::string ArmState::getRuntimeText()
{
    return runtime_except_msg_.text_msg;
}

std::string ArmState::getVarTitle()
{
    return var_msg_.msg_title;
}

std::string ArmState::getVarText()
{
    return var_msg_.msg_text;
}
