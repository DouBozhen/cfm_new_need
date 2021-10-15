#ifndef CONTROL_ARM_H
#define CONTROL_ARM_H

#include <string>

#include "arm_state.h"
#include "arm_comm.h"
#include "ur_datatype.h"

#define DIGITAL_INPUT_INDEX_MIN 0
#define DIGITAL_INPUT_INDEX_MAX 17

class ControlArm 
{
public:
    ControlArm(std::string host);
    ~ControlArm();

    bool start();
    void halt();
	bool isConnected();
	
	/********* robot mode data ***********/
	bool isRobotConnected();
	bool isRobotEnabled();
	bool isRobotPoweredOn();
	bool isEmergencyStopped();
	bool isProtectiveStopped();
	bool isProgramRunning();
	bool isProgramPaused();
	uint8_t getRobotMode();              
	uint8_t getControlMode();
	double getTargetVelFraction(); // ??? to_check...
	double getVelScaling();
	double getTargetVelFractionLimit(); // ??? to_check...

	/************* joint data ****************/
	ur_data_type::Joint getActualJoint();
	ur_data_type::Joint getTargetJoint();
	ur_data_type::Joint getActualJointSpeed();
	ur_data_type::Joint getActualJointCurrent();
	ur_data_type::Joint getActualJointVoltage();
	ur_data_type::Joint getTMotor();
	ur_data_type::Joint getTMicro();
	std::array<unsigned char, 6> getAllJointMode();      

	/********** Cartesian info ***************/
	ur_data_type::CartPose getToolPose();
	double getToolPointX();
	double getToolPointY();
	double getToolPointZ();
	double getToolRpyRx();
	double getToolRpyRy();
	double getToolRpyRz();

	ur_data_type::CartPose getTcpOffsetPose();
	double getTcpOffsetPointX();
	double getTcpOffsetPointY();
	double getTcpOffsetPointZ();
	double getTcpOffsetRpyRx();
	double getTcpOffsetRpyRy();
	double getTcpOffsetRpyRz();

	/* Masterboard data */
	bool getDigitalInputs(int index);
	int getDigitalOutputs();
	char getAnalogInputRange(int index);
	double getAnalogInput(int index);
	char getAnalogOutputDomain(int index);
	double getAnalogOutput(int index);

	uint8_t getSafetyMode();                  
	uint8_t getInReducedMode();
	uint8_t getOperationModeInput();

    /********** ToolData ***************/
	char getToolAnalogInputRange(int index);
	double getToolAnalogInput(int index);
	uint8_t getToolMode();

	/************* Kinematics info **************/
	std::array<uint32_t, 6> getKinematicsChecksum();
	// ur_data_type::DHParam getKineDHParam();

	/*********** Configuration data *************/
	ur_data_type::Joint getJointLimitMin();
	ur_data_type::Joint getJointLimitMax();
	ur_data_type::Joint getJointSpeedMax();
	ur_data_type::Joint getJointAccMax();

	ur_data_type::DHParam getKineDHParam();
	int getMasterBoardVersion();
	int getControllerBoxType(); // ??? to check...
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

	/******** additional info *********/
	bool isFreeDrivePressed();
	bool isFreeDriveEnabled();
	bool isFreeDriveIOEnabled(); // ??? to check...
 
	/************ VersionMessage *************/
	std::string getProjectName();
	std::string getVersion();
	int8_t getSource(); 
	std::string getBugfixVersion();

	/******** safety mode msg and robot comm msg***********/
	int8_t getSafetyModeType();
	std::string getSafetyTextMsg();
	// robot comm message
	int getRobotCommWarningLevel();
	std::string getRobotCommText();

	/********  key msg and lable msg***********/
	std::string getKeyTitle();
	std::string getKeyText();
	//  label message
	int getLabelId();
	std::string getLabelText();

	/****** Other msg ***********/
	// popup msg
	bool getPopupWarning();
	bool getPopupError();
	bool getPopupBlocking(); // ??? to check...
	std::string getPopupTitle();
	std::string getPopupText();

	//runtime exception
	uint32_t getRuntimeLineNum();
	uint32_t getRuntimeColumnNum();
	std::string getRuntimeText();

private:
    ArmComm* arm_comm_;

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

	//varmessage
	std::string getVarTitle();
	std::string getVarText();

	// request value msg
	bool getRequestWarning();
	bool getRequestError();
	bool getRequestBlocking();
	std::string getRequesteTitle();
	std::string getRequestText();

	//text message
	std::string getText();

	/*********** Configuration data *************/
	double getJointVDefault();
	double getJointADefault();
	double getToolVDefault();
	double getToolADefault();
	double getEqRad();

	/* Masterboard data */
	float getMasterBoardTemperature();
	float getRobotVoltage48V();              
	float getRobotCurrent();
	float getMasterIOCurrent();

	int8_t getEuromap67Installed();      
	int getEuromapInputs();
	int getEuromapOutputs();
	float getEuromapVoltage();
	float getEuromapCurrent();
	uint8_t getThreePositionEnablingInput();

	/********** ToolData ***************/
	float getToolVoltage48V();
	uint8_t getToolOutputVoltage();
	float getToolCurrent();
	float getToolTemperature();
};


#endif