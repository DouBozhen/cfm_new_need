#ifndef ARM_STATE_H
#define ARM_STATE_H

#include <mutex>
#include <condition_variable>
#include <array>
#include <vector>

#include "ur_datatype.h"
#include "ur_comm_robot_msg.h"
#include "ur_comm_robot_state_msg.h"
#include "ur_comm_pkg.h"

using namespace message_types;
using namespace ur_data_type;

class ArmState
{
public:
	ArmState();
	~ArmState();

	void setProtocolVersion(double ver);
	double getProtocolVersion();

	/********* robot mode data ***********/
	bool isRobotConnected();
	bool isRobotEnabled();
	bool isRobotPoweredOn();
	bool isEmergencyStopped();
	bool isProtectiveStopped();
	bool isProgramRunning();
	bool isProgramPaused();
	unsigned char getRobotMode();
	unsigned char getControlMode();
	double getTargetSpeedFraction();
	double getSpeedScaling();
	double getTargetSpeedFractionLimit();

	/********** joint data *************/
    ur_data_type::Joint getActualJoint() const; 
    ur_data_type::Joint getActualJointVel() const;  
    ur_data_type::Joint getActualJointCurrent() const; 
    ur_data_type::Joint getActualJointVoltage() const; 

	ur_data_type::Joint getTargetJoint() const; 
	ur_data_type::Joint getTmotor(); // tmotor: ??
	ur_data_type::Joint getTmicro(); // t_micro: ??
	std::array<uint8_t, 6> getAllJointMode();

	/********* Cartesian info *************/
	ur_data_type::CartPose getToolPose();
	double getToolPointX();
	double getToolPointY();
	double getToolPointZ();
	double getRpyRx();
	double getRpyRy();
	double getRpyRz();

	ur_data_type::CartPose getTcpOffsetPose();
	double getTcpOffsetPointX();
	double getTcpOffsetPointY();
	double getTcpOffsetPointZ();
	double getTcpOffsetRpyRx();
	double getTcpOffsetRpyRy();
	double getTcpOffsetRpyRz();

	/**********  Masterboard data *************/
	int getDigitalInputs();
	int getDigitalOutputs();
	double getAnalogInput(int index);
	char getAnalogInputRange(int index);
	double getAnalogOutput(int index);
	char getAnalogOutputDomain(int index);

	int getEuromapInputs();
	int getEuromapOutputs();
	float getEuromapVoltage();
	float getEuromapCurrent();
	char isEuromap67Installed();

	uint8_t getInReduceMode();
	uint8_t getOpModeInput();
	uint8_t get3PositionEnablingSwitch();

	float getMasterBoardTemperature();
	float getRobotVoltage48V();
	float getRobotCurrent();
	float getMasterIOCurrent();

	unsigned char getSafetyMode();

	/************* ToolData ******************/
	unsigned char getToolAnalogInputRange(int index);
	double getToolAnalogInput(int index);
	float getToolVoltage48V();
	unsigned char getToolOutputVoltage();
	float getToolCurrent();
	float getToolTemperature();
	unsigned char getToolMode();

	/************ Kinematics info **************/
	std::array<unsigned int, 6> getKineCheckSum();
	DHParam getKineDHParam();

	/*********** Configuration data *************/	
	ur_data_type::Joint getJointLimitMin() const; 
	ur_data_type::Joint getJointLimitMax() const;
	ur_data_type::Joint getJointMaxVel() const;
	ur_data_type::Joint getJointMaxAcc() const;
	double getJointVDefault();
	double getJointADefault();
	double getToolVDefault();
	double getToolADefault();
	double getEqRad();
	//ur_data_type::DHParam getKineDHParam();
	int getMasterBoardVersion();
	int getControllerBoxType();
	int getRobotType();
	int getRobotSubType();

	/******* force mode data *************/
	ur_data_type::CartPose getForce();
	double getForcePointX();
	double getForcePointY();
	double getForcePointZ();
	double getForceRpyRx();
	double getForceRpyRy();
	double getForceRpyRz();
	double getForceDexterity();

	/************ getForceDexterity ***********/
	bool isFreeDrivePressed();
	bool isFreeDriveEnabled();
	bool isFreeDriveIOEnabled();

	/************ VersionMessage *************/
	int8_t getSource();
	std::string getProjectName();
	std::string getVersion();
	std::string getBugfixVersion();

	/******** SafetyModeMessage ***********/
	int8_t getSafetyModeType();
	std::string getSafetyTextMsg();

	// robot comm message
	int getRobotCommWarningLevel();
	std::string getRobotCommText();

	/********  key message ***********/
	std::string getKeyTitle();
	std::string getKeyText();
	//  label message
	int getLabelId();
	std::string getLabelText();

	/***** globa params: setup and update **********/
	uint16_t getSetupGlobalParamsStartIndex();
	std::vector<std::string> getSetupGlobalParamsNames();
	
	uint8_t getUpdateGlobalParamsStartIndex();
	uint8_t getUpdateGlobalParamsValueType();

	std::string getUpdateGlobalParamsStringValue();
	std::string getUpdateGlobalParamsStringValueVar();
	int16_t getUpdateGlobalParamsListLength();
	uint8_t getUpdateGlobalParamsListValType();
	ur_data_type::CartPose getUpdateGlobalParamsPose();

	bool getUpdateGlobalParamsBool();
	float getUpdateGlobalParamsNum();
	int32_t getUpdateGlobalParamsInt();
	float getUpdateGlobalParamsFloat();

	/****** popup msg ***********/
	bool getPopupWarning();
	bool getPopupError();
	bool getPopupBlocking();
	std::string getPopupTitle();
	std::string getPopupText();

	// request value msg
	bool getRequestWarning();
	bool getRequestError();
	bool getRequestBlocking();
	std::string getRequesteTitle();
	std::string getRequestText();

	//text message
	std::string getText();

	//runtime exception
	uint32_t getRuntimeLineNum();
	uint32_t getRuntimeColumnNum();
	std::string getRuntimeText();

	//varmessage
	std::string getVarTitle();
	std::string getVarText();

	void setDisconnected(); 
	void decode(uint8_t* buf, unsigned int len);
private:
	/* pkg */
	GeneralPackage general_pkg_;
	RobotMessagePackage robot_msg_pkg_;
	ProgramStateMessagePackage program_state_msg_pkg_;

	/* msg */
	VersionMessage version_msg_; 
	SafetyModeMessage safety_mode_msg_;
	RobotCommMessage robot_comm_msg_;
	KeyMessage key_msg_;
	LabelMessage lable_msg_;
	PopupMessage popup_msg_;
	RuntimeExceptionMessage runtime_except_msg_;
	RequestValueMessage request_msg_;
	TextMessage text_msg_;
	VarMessage var_msg_;
	GlobalParamsSetupMessage global_setup_msg_;
	GlobalParamsUpdateMessage global_update_msg_;

	/* data */	
	MasterBoardData mb_data_;
	RobotModeData robot_mode_;
	ConfigurationData config_data_;
	KinematicsInfo kine_info_;
	JointData joint_data_;
	CartesianInfo cart_info_;   
	ToolData tool_data_;
	ForceModeData force_mode_;
	AdditionalInfo additional_info_;

	/*Other member variables */
	bool controller_updated;
	double protocol_version_;
	std::condition_variable* pMsg_cond_; 
	bool new_data_available_; 
	unsigned char robot_mode_running_;
	std::vector<std::string> global_params_names_; 

	double ntohd(uint64_t nf);
	float ntohf(uint32_t nf);

	void decodeRobotMsg(uint8_t* buf, unsigned int offset, int32_t len);
	void decodeVersionFromRobotMsg(uint8_t* buf, unsigned int offset, int32_t len, VersionMessage &version_msg);
	void decodeSafetyModeFromRobotMsg(uint8_t* buf, unsigned int offset, int32_t len, SafetyModeMessage &safety_mode_msg);
	void decodeKeyFromRobotMsg(uint8_t* buf, unsigned int offset, int32_t len, KeyMessage &key_msg);
	void decodeRobotCommFromRobotMsg(uint8_t* buf, unsigned int offset, int32_t len,RobotCommMessage &robot_comm_msg);
	void decodeLabelFromRobotMsg(uint8_t* buf, unsigned int offset, int32_t len, LabelMessage &lable_msg);
	void decodePopupFromRobotMsg(uint8_t* buf, unsigned int offset, int32_t len, PopupMessage &popup_msg);
	void decodeRequestFromRobotMsg(uint8_t* buf, unsigned int offset, int32_t len, RequestValueMessage &request_msg);
	void decodeTextFromRobotMsg(uint8_t* buf, unsigned int offset, int32_t len, TextMessage &text_msg);
	void decodeRuntimeExceptFromRobotMsg(uint8_t* buf, unsigned int offset, int32_t len, RuntimeExceptionMessage &runtime_except_msg);

	void decodeRobotStateMsg(uint8_t* buf, unsigned int offset, uint32_t len);
	void decodeJointdataFromRobotStateMsg(uint8_t* buf, unsigned int offset,JointData &joint_data);
	void decodeRobotModeFromRobotStateMsg(uint8_t* buf, unsigned int offset, RobotModeData &robot_mode);
	void decodeCartInfoFromRobotStateMsg(uint8_t* buf, unsigned int offset, CartesianInfo &cart_info);
	void decodeMasterboardFromRobotStateMsg(uint8_t* buf, unsigned int offset, MasterBoardData &mb_data);
	void decodeToolDataFromRobotStateMsg(uint8_t* buf, unsigned int offset, ToolData &tool_data);
	void decodeKineInfoFromRobotStateMsg(uint8_t* buf, unsigned int offset, KinematicsInfo &kine_info);
	void decodeConfigDataFromRobotStateMsg(uint8_t* buf, unsigned int offset, ConfigurationData &config_data);
	void decodeForceModeFromRobotStateMsg(uint8_t* buf, unsigned int offset, ForceModeData &force_mode);
	void decodeAdditionalInfoFromRobotState(uint8_t* buf, unsigned int offset, AdditionalInfo &additional_info);

	void decodeProgStateMsg(uint8_t* buf, unsigned int offset, int32_t len);
	void decodeVarFromProgStateMsg(uint8_t* buf, unsigned int offset, int32_t len,VarMessage &var_msg);
	void decodeGlobalParamsSetupFromProgStateMsg(uint8_t* buf, unsigned int offset, int32_t len, GlobalParamsSetupMessage &global_setup);
	void decodeGlobalParamsUpdataFromProgStateMsg(uint8_t* buf, unsigned int offset, int32_t len, GlobalParamsUpdateMessage &global_update);

	std::vector<double> decodeVector(uint8_t* buf, int start_index, int nr_of_vals);
};

#endif

