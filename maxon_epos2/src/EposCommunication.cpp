//============================================================================
// Name        : EposCommunication.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 26.04.2018
// Copyright   : BSD 3-Clause
// Description : Class providing the communication functions for Maxon EPOS2.
//		 		 Install EPOS2 Linux Library from Maxon first!
//============================================================================

#include "maxon_epos2/EposCommunication.hpp"

namespace maxon_epos2 {

EposCommunication::EposCommunication()
{
	g_pKeyHandle = 0; //set adress to zero
	g_pSubKeyHandle = 0;
	g_usNodeId = 1;
	g_baudrate = 0;
}

EposCommunication::~EposCommunication()
{
}

void EposCommunication::LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode, unsigned short p_usNodeId = 0)
{
	std::cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult <<", ID = " << p_usNodeId << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< std::endl;
}

void EposCommunication::LogInfo(std::string message)
{
	std::cout << message << std::endl;
}

void EposCommunication::SeparatorLine()
{
	const int lineLength = 65;
	for(int i=0; i<lineLength; i++)
	{
		std::cout << "-";
	}
	std::cout << std::endl;
}

void EposCommunication::PrintHeader()
{
	SeparatorLine();

	LogInfo("Initializing EPOS2 Communication Library");

	SeparatorLine();
}

void EposCommunication::PrintSettings()
{
	std::stringstream msg;

	msg << "default settings:" << std::endl;
	msg << "main node id             = " << g_usNodeId << std::endl;
	msg << "device name         = '" << g_deviceName << "'" << std::endl;
	msg << "protocal stack name = '" << g_protocolStackName << "'" << std::endl;
	msg << "interface name      = '" << g_interfaceName << "'" << std::endl;
	msg << "port name           = '" << g_portName << "'"<< std::endl;
	msg << "baudrate            = " << g_baudrate << std::endl;
	msg << "node id list        = " << g_nodeIdList;

	LogInfo(msg.str());

	SeparatorLine();
}

void EposCommunication::SetDefaultParameters(unsigned short *nodeIdList, int motors)
{

	/* Options:
	 * node id: default 1 (not ROS node!)
	 * device name: EPOS2, EPOS4, default: EPOS4
	 * protocol stack name: MAXON_RS232, CANopen, MAXON SERIAL V2, default: MAXON SERIAL V2
	 * interface name: RS232, USB, CAN_ixx_usb 0, CAN_kvaser_usb 0,... default: USB
	 * port name: COM1, USB0, CAN0,... default: USB0
	 * baudrate: 115200, 1000000,... default: 1000000
	 */

	//USB
	g_usNodeId = nodeIdList[0];
	g_deviceName = "EPOS2"; 
	g_protocolStackName = "MAXON SERIAL V2"; 
	g_interfaceName = "USB"; 
	g_baudrate = 1000000; 
	g_subProtocolStackName = "CANopen";
	// g_usSubNodeId = 2;
	g_motors = motors;
	g_nodeIdList = (unsigned short*) std::calloc(g_motors, sizeof(unsigned short));
	for(int i = 0; i < g_motors; i++)
		g_nodeIdList[i] = nodeIdList[i];

	//get the port name:
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pPortNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;
	VCS_GetPortNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), (char*)g_interfaceName.c_str(), lStartOfSelection, pPortNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode);
	g_portName = pPortNameSel;
	ROS_INFO_STREAM("Port Name: " << g_portName);

}

int EposCommunication::SetPositionProfile(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode,
										  unsigned int profile_velocity = 500,
										  unsigned int profile_acceleration = 1000,
										  unsigned int profile_deceleration = 1000)
{
	//to use set variables below first!
	int lResult = MMC_SUCCESS;
	int vel_rpm = radsToRpm(profile_velocity);
	if(vel_rpm == 0)
		return lResult;
	if(VCS_SetPositionProfile(p_DeviceHandle, p_usNodeId, vel_rpm, radsToRpm(profile_acceleration), radsToRpm(profile_deceleration), p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_SetPositionProfile", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	return lResult;
}

int EposCommunication::SetHomingParameter(unsigned int* p_pErrorCode)
{
	//to use set variables below first!
	int lResult = MMC_SUCCESS;
	unsigned int homing_acceleration = 20;
	unsigned int speed_switch = 50;
	unsigned int speed_index = 50;
	int home_offset = 0;
	unsigned short current_threshold = 0;
	int home_position = 0;
	if(VCS_SetHomingParameter(g_pKeyHandle, g_usNodeId, homing_acceleration, speed_switch, speed_index, home_offset, current_threshold, home_position, p_pErrorCode) == MMC_FAILED)
	{
		lResult = MMC_FAILED;
		LogError("VCS_SetHomingParameter", lResult, *p_pErrorCode);
	}

	return lResult;
}

int EposCommunication::SetSensor(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;

	if(VCS_SetSensorType(g_pKeyHandle, g_usNodeId, 1, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_SetSensorType", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(VCS_SetIncEncoderParameter(g_pKeyHandle, g_usNodeId, 256, 0, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_SetIncEncoderParameter", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	return lResult;
}

int EposCommunication::OpenDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_protocolStackName.c_str());
	strcpy(pInterfaceName, g_interfaceName.c_str());
	strcpy(pPortName, g_portName.c_str());

	LogInfo("Open device...");
	LogInfo(pInterfaceName);

	g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

	if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=MMC_FAILED)
		{
			if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=MMC_FAILED)
			{
				if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=MMC_FAILED)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		g_pKeyHandle = 0;
		ROS_ERROR("Opening device failed.");
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	return lResult;
}

int EposCommunication::OpenSubDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_subProtocolStackName.c_str());

	LogInfo("Open device...");

	g_pSubKeyHandle = VCS_OpenSubDevice(g_pKeyHandle, pDeviceName, pProtocolStackName, p_pErrorCode);

	if(g_pSubKeyHandle!=0 && *p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pSubKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=MMC_FAILED)
		{
			if(VCS_SetProtocolStackSettings(g_pSubKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=MMC_FAILED)
			{
				if(VCS_GetProtocolStackSettings(g_pSubKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=MMC_FAILED)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		g_pKeyHandle = 0;
		ROS_ERROR("Opening sub device failed.");
	}

	delete []pDeviceName;
	delete []pProtocolStackName;

	return lResult;
}

int EposCommunication::CloseDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	*p_pErrorCode = 0;

	if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == MMC_FAILED)
	{
		lResult = MMC_FAILED;
	}

	for(int i = 1; i < g_motors; i++)
	{
		if(VCS_SetDisableState(g_pSubKeyHandle, g_nodeIdList[i], p_pErrorCode) == MMC_FAILED)
		{
			lResult = MMC_FAILED;
		}
	}

	LogInfo("Close device");

	if(VCS_CloseAllSubDevices(g_pSubKeyHandle, p_pErrorCode)!=MMC_FAILED && *p_pErrorCode == 0)
	{
		lResult = MMC_SUCCESS;
	}

	if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=MMC_FAILED && *p_pErrorCode == 0)
	{
		lResult = MMC_SUCCESS;
	}

	return lResult;
}

int EposCommunication::PrepareEpos(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	BOOL oIsFault = 0; //0 is not in fault state

	if(VCS_GetFaultState(p_DeviceHandle, p_usNodeId, &oIsFault, p_pErrorCode ) == MMC_FAILED)
	{
		LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	ROS_INFO_STREAM("Debug 1: FaultState:" << oIsFault);
	if(lResult == MMC_SUCCESS)
	{
		if(oIsFault)
		{
			std::stringstream msg;
			msg << "clear fault, node = '" << g_usNodeId << "'";
			LogInfo(msg.str());

			if(VCS_ClearFault(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
			{
				LogError("VCS_ClearFault", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}
		}

		if(lResult == MMC_SUCCESS)
		{

			BOOL oIsEnabled = 0;

			if(VCS_GetEnableState(p_DeviceHandle, p_usNodeId, &oIsEnabled, p_pErrorCode) == MMC_FAILED)
			{
				LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
				lResult = MMC_FAILED;
			}


			if(!oIsEnabled)
			{
				if(VCS_SetEnableState(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
				{
					LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
					lResult = MMC_FAILED;
				}
				else{
					VCS_GetEnableState(p_DeviceHandle, p_usNodeId, &oIsEnabled, p_pErrorCode);
					ROS_INFO_STREAM("SetEnableState should be 1:" <<  oIsEnabled);
				}
			}
		}
	}
	return lResult;
}

// int EposCommunication::PositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode)
// {
// 	int lResult = MMC_SUCCESS;

// 	lResult = ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, p_pErrorCode);

// 	if(lResult != MMC_SUCCESS)
// 	{
// 		LogError("ActivateProfilePositionMode", lResult, *p_pErrorCode);
// 	}

// 	return lResult;
// }

int EposCommunication::HomingMode(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;

	lResult = ActivateHomingMode(g_pKeyHandle, g_usNodeId, p_pErrorCode);

	if(lResult != MMC_SUCCESS)
	{
		LogError("ActivateHomingMode", lResult, *p_pErrorCode);
	}

	return lResult;
}

int EposCommunication::ActivateProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	std::stringstream msg;

	msg << "set profile position mode, node = " << p_usNodeId;
	LogInfo(msg.str());

	if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_ActivateProfilePositionMode", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	else {
		ROS_INFO("VCS_ActivateProfilePositionMode successfull.");
	}
	return lResult;
}

int EposCommunication::ActivatePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	std::stringstream msg;

	msg << "set position mode, node = " << p_usNodeId;
	LogInfo(msg.str());

	if(VCS_ActivatePositionMode(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_ActivatePositionMode", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	else {
		ROS_INFO("VCS_ActivatePositionMode successfull.");
	}
	return lResult;
}

int EposCommunication::ActivateVelocityMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	std::stringstream msg;

	msg << "set velocity mode, node = " << p_usNodeId;
	LogInfo(msg.str());

	if(VCS_ActivateVelocityMode(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_ActivateVelocityMode", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	else {
		ROS_INFO("VCS_ActivateVelocityMode successfull.");
	}
	return lResult;
}

int EposCommunication::ActivateHomingMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	std::stringstream msg;

	msg << "set homing mode, node = " << p_usNodeId;
	LogInfo(msg.str());

	if(VCS_ActivateHomingMode(p_DeviceHandle, p_usNodeId, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_ActivateHomingMode", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	return lResult;
}

int EposCommunication::FindHome(unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	signed char homing_method = 1; //method 1: negative limit switch

	if(VCS_FindHome(g_pKeyHandle, g_usNodeId, homing_method, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_ActivateHomingMode", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	return lResult;
}

int EposCommunication::HomingSuccess(bool* homing_success, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	*homing_success = MMC_FAILED;
	unsigned int timeout = 6000000; //timeout in ms, should be shorter after testing

	if(VCS_WaitForHomingAttained(g_pKeyHandle, g_usNodeId, timeout, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_WaitForHomingAttained", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	int pHomingAttained;
	int pHomingError;

	if(VCS_GetHomingState(g_pKeyHandle, g_usNodeId, &pHomingAttained, &pHomingError, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_GetHomingState", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}

	if(pHomingAttained == MMC_SUCCESS & pHomingError == 0)
	{
		*homing_success = MMC_SUCCESS;
	}

	return lResult;
}

int EposCommunication::SetPosition(HANDLE p_DeviceHandle, unsigned short p_usNodeId, long position_setpoint, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;
	bool absolute = true;

	if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, position_setpoint, absolute, 1, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_MoveToPosition", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
		std::cout<<"Fuck Fail_1"<<std::endl;
	}
	else{
		// ROS_INFO("Movement executed.");
	}

	return lResult;
}

int EposCommunication::PrintAvailablePorts(char* p_pInterfaceNameSel)
{
	int lResult = MMC_FAILED;
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pPortNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;

	do
	{
		if(!VCS_GetPortNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), p_pInterfaceNameSel, lStartOfSelection, pPortNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
		{
			lResult = MMC_FAILED;
			LogError("GetPortNameSelection", lResult, ulErrorCode);
			break;
		}
		else
		{
			lResult = MMC_SUCCESS;
			printf("            port = %s\n", pPortNameSel);
		}

		lStartOfSelection = 0;
	}
	while(lEndOfSelection == 0);

	return lResult;
}

int EposCommunication::PrintAvailableInterfaces()
{
	int lResult = MMC_FAILED;
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pInterfaceNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;

	do
	{
		if(!VCS_GetInterfaceNameSelection((char*)g_deviceName.c_str(), (char*)g_protocolStackName.c_str(), lStartOfSelection, pInterfaceNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
		{
			lResult = MMC_FAILED;
			LogError("GetInterfaceNameSelection", lResult, ulErrorCode);
			break;
		}
		else
		{
			lResult = MMC_SUCCESS;

			printf("interface = %s\n", pInterfaceNameSel);

			PrintAvailablePorts(pInterfaceNameSel);
		}

		lStartOfSelection = 0;
	}
	while(lEndOfSelection == 0);

	SeparatorLine();

	delete[] pInterfaceNameSel;

	return lResult;
}

int EposCommunication::PrintDeviceVersion()
{
	int lResult = MMC_FAILED;
	unsigned short usHardwareVersion = 0;
	unsigned short usSoftwareVersion = 0;
	unsigned short usApplicationNumber = 0;
	unsigned short usApplicationVersion = 0;
	unsigned int ulErrorCode = 0;

	if(VCS_GetVersion(g_pKeyHandle, g_usNodeId, &usHardwareVersion, &usSoftwareVersion, &usApplicationNumber, &usApplicationVersion, &ulErrorCode))
	{
		printf("%s Hardware Version    = 0x%04x\n      Software Version    = 0x%04x\n      Application Number  = 0x%04x\n      Application Version = 0x%04x\n",
				g_deviceName.c_str(), usHardwareVersion, usSoftwareVersion, usApplicationNumber, usApplicationVersion);
		lResult = MMC_SUCCESS;
	}

	return lResult;
}

int EposCommunication::PrintAvailableProtocols()
{
	int lResult = MMC_FAILED;
	int lStartOfSelection = 1;
	int lMaxStrSize = 255;
	char* pProtocolNameSel = new char[lMaxStrSize];
	int lEndOfSelection = 0;
	unsigned int ulErrorCode = 0;

	do
	{
		if(!VCS_GetProtocolStackNameSelection((char*)g_deviceName.c_str(), lStartOfSelection, pProtocolNameSel, lMaxStrSize, &lEndOfSelection, &ulErrorCode))
		{
			lResult = MMC_FAILED;
			LogError("GetProtocolStackNameSelection", lResult, ulErrorCode);
			break;
		}
		else
		{
			lResult = MMC_SUCCESS;

			printf("protocol stack name = %s\n", pProtocolNameSel);
		}

		lStartOfSelection = 0;
	}
	while(lEndOfSelection == 0);

	SeparatorLine();

	delete[] pProtocolNameSel;

	return lResult;
}

int EposCommunication::GetPosition(int* pPositionIsCounts, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;

	if(VCS_GetPositionIs(g_pKeyHandle, g_usNodeId, pPositionIsCounts, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_GetPositionIs", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	return lResult;
}

int EposCommunication::GetVelocity(int* pVelocityIsCounts, unsigned int* p_pErrorCode)
{
	int lResult = MMC_SUCCESS;

	if(VCS_GetVelocityIs(g_pKeyHandle, g_usNodeId, pVelocityIsCounts, p_pErrorCode) == MMC_FAILED)
	{
		LogError("VCS_GetVelocityIs", lResult, *p_pErrorCode);
		lResult = MMC_FAILED;
	}
	return lResult;
}

//public functions:

int EposCommunication::initialization(unsigned short *nodeIdList, int motors){
	int lResult = MMC_SUCCESS;
	unsigned int ulErrorCode = 0;

	//Print Header:
	PrintHeader();

	//Set Default Parameters:
	SetDefaultParameters(nodeIdList, motors);

	//Print Settings:
	PrintSettings();

	//Open device:
	if((lResult = OpenDevice(&ulErrorCode))==MMC_FAILED)
	{
		LogError("OpenDevice", lResult, ulErrorCode);
		deviceOpenedCheckStatus = MMC_FAILED;
	}
	else {
		deviceOpenedCheckStatus = MMC_SUCCESS; //used to forbid other functions as getPosition and getVelocity if device is not opened
	}
	
	//Set Sensor parameters:
	// if((lResult = SetSensor(&ulErrorCode))==MMC_FAILED)
	// {
	// 	LogError("SetSensor", lResult, ulErrorCode);
	// }

	//Set Homing Parameter:
	// if((lResult = SetHomingParameter(&ulErrorCode))==MMC_FAILED)
	// {
	// 	LogError("SetHomingParameter", lResult, ulErrorCode);
	// }
	
	//Prepare EPOS controller:
	if((lResult = PrepareEpos(g_pKeyHandle, g_usNodeId, &ulErrorCode))==MMC_FAILED)
	{
		LogError("PrepareEpos", lResult, ulErrorCode);
	}

	

	// if((lResult = ActivateProfilePositionMode(g_pKeyHandle, g_usNodeId, &ulErrorCode))==MMC_FAILED)
	// {
	// 	LogError("PositionMode", lResult, ulErrorCode);
	// }

	// if((lResult = ActivateProfilePositionMode(g_pSubKeyHandle, g_usSubNodeId, &ulErrorCode))==MMC_FAILED)
	// {
	// 	LogError("PositionSubMode", lResult, ulErrorCode);
	// }

	//Set Position profile:
	// if((lResult = SetPositionProfile(g_pKeyHandle, g_usNodeId, &ulErrorCode))==MMC_FAILED)
	// {
	// 	LogError("SetPositionProfile", lResult, ulErrorCode);
	// }

	
	if(g_motors > 1)
	{
		if((lResult = OpenSubDevice(&ulErrorCode))==MMC_FAILED)
		{
			LogError("OpenSubDevice", lResult, ulErrorCode);
			deviceOpenedCheckStatus = MMC_FAILED;
		}
		else {
			deviceOpenedCheckStatus = MMC_SUCCESS; //used to forbid other functions as getPosition and getVelocity if device is not opened
		}
		for(int i = 1; i < g_motors; i++)
		{
			if((lResult = PrepareEpos(g_pSubKeyHandle, g_nodeIdList[i], &ulErrorCode))==MMC_FAILED)
			{
				LogError("PrepareSubEpos ID = ", lResult, ulErrorCode , g_nodeIdList[i]);
			}

			// if((lResult = SetPositionProfile(g_pSubKeyHandle, g_nodeIdList[i], &ulErrorCode))==MMC_FAILED)
			// {
			// 	LogError("SetSubPositionProfile ID = ", lResult, ulErrorCode, g_nodeIdList[i]);
			// }
		}
	}

	unsigned int pMaxFollowingError;
	unsigned int pMaxProfileVelocity;
	unsigned int pMaxAcceleration;
	unsigned int MaxAcceleration = 5000;
	if((lResult = VCS_GetMaxFollowingError(g_pKeyHandle, g_usNodeId, &pMaxFollowingError, &ulErrorCode))==MMC_FAILED)
	{
		LogError("VCS_GetMaxFollowingError", lResult, ulErrorCode);
	}
	if((lResult = VCS_GetMaxProfileVelocity(g_pKeyHandle, g_usNodeId, &pMaxProfileVelocity, &ulErrorCode))==MMC_FAILED)
	{
		LogError("VCS_GetMaxProfileVelocity", lResult, ulErrorCode);
	}
	if((lResult = VCS_SetMaxProfileVelocity(g_pKeyHandle, g_usNodeId, pMaxProfileVelocity, &ulErrorCode))==MMC_FAILED)
	{
		LogError("VCS_SetMaxProfileVelocity", lResult, ulErrorCode);
	}
	if((lResult = VCS_SetMaxAcceleration(g_pKeyHandle, g_usNodeId, MaxAcceleration, &ulErrorCode))==MMC_FAILED)
	{
		LogError("VCS_SetMaxAcceleration", lResult, ulErrorCode);
	}
	if((lResult = VCS_GetMaxAcceleration(g_pKeyHandle, g_usNodeId, &pMaxAcceleration, &ulErrorCode))==MMC_FAILED)
	{
		LogError("VCS_GetMaxAcceleration", lResult, ulErrorCode);
	}
	for(int i = 1; i < g_motors; i++)
	{
		if((lResult = VCS_GetMaxFollowingError(g_pSubKeyHandle, g_nodeIdList[i], &pMaxFollowingError, &ulErrorCode))==MMC_FAILED)
		{
			LogError("VCS_GetMaxFollowingError", lResult, ulErrorCode, g_nodeIdList[i]);
		}
		if((lResult = VCS_GetMaxProfileVelocity(g_pSubKeyHandle, g_nodeIdList[i], &pMaxProfileVelocity, &ulErrorCode))==MMC_FAILED)
		{
			LogError("VCS_GetMaxProfileVelocity", lResult, ulErrorCode, g_nodeIdList[i]);
		}
		if((lResult = VCS_SetMaxAcceleration(g_pSubKeyHandle, g_nodeIdList[i], MaxAcceleration, &ulErrorCode))==MMC_FAILED)
		{
			LogError("VCS_SetMaxAcceleration", lResult, ulErrorCode, g_nodeIdList[i]);
		}
		if((lResult = VCS_GetMaxAcceleration(g_pSubKeyHandle, g_nodeIdList[i], &pMaxAcceleration, &ulErrorCode))==MMC_FAILED)
		{
			LogError("VCS_GetMaxAcceleration", lResult, ulErrorCode, g_nodeIdList[i]);
		}
		std::cout<<"ID: "<<g_nodeIdList[i]<<", pMaxFollowingError: "<<pMaxFollowingError<<", pMaxProfileVelocity: "<<pMaxProfileVelocity<<", pMaxAcceleration: "<<pMaxAcceleration<<std::endl;
	}
	
	// for(int i = 1; i <= g_motors; i++)
	// {
	// 	if((lResult = setHomingParameter(i, &ulErrorCode))==MMC_FAILED)
	// 	{
	// 		LogError("SetHomingParameter", lResult, ulErrorCode);
	// 	}
	// }

	LogInfo("Initialization successful");

	return lResult;
}

int EposCommunication::setHomingParameter(unsigned short p_usNodeId, unsigned int* p_pErrorCode)
{
	//to use set variables below first!
	int lResult = MMC_SUCCESS;
	unsigned int homing_acceleration = 20;
	unsigned int speed_switch = 50;
	unsigned int speed_index = 50;
	int home_offset = mmToCounts(0.5);
	unsigned short current_threshold = 0;
	int home_position = mmToCounts(0.5);
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;
	if(VCS_SetHomingParameter(p_DeviceHandle, p_usNodeId, homing_acceleration, speed_switch, speed_index, home_offset, current_threshold, home_position, p_pErrorCode) == MMC_FAILED)
	{
		lResult = MMC_FAILED;
		LogError("VCS_SetHomingParameter", lResult, *p_pErrorCode);
	}

	return lResult;
}

int EposCommunication::homing()
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

	//Start homing mode:
	if((lResult = HomingMode(&ulErrorCode))==MMC_FAILED)
	{
		LogError("HomingMode", lResult, ulErrorCode);
	}

	//Find home:
	if((lResult = FindHome(&ulErrorCode))==MMC_FAILED)
	{
		LogError("FindHome", lResult, ulErrorCode);
	}

	//Check if successfull:
	bool homing_success_status = MMC_FAILED;
	if((lResult = HomingSuccess(&homing_success_status, &ulErrorCode))==MMC_FAILED)
	{
		LogError("HomingSuccess", lResult, ulErrorCode);
	}
	else{
		homingCompletedStatus = MMC_SUCCESS;
	}

	return homing_success_status;
}

int EposCommunication::startPositionMode()
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

	//Start position mode:
	// if((lResult = PositionMode(&ulErrorCode))==MMC_FAILED)
	// {
	// 	LogError("PositionMode", lResult, ulErrorCode);
	// }
	// else{
	// 	ROS_INFO("PositionMode successfully started.");
	// }

	if((lResult = ActivatePositionMode(g_pKeyHandle, g_usNodeId, &ulErrorCode))==MMC_FAILED)
	{
		LogError("PositionMode", lResult, ulErrorCode);
	}
	for(int i = 1; i < g_motors; i++)
	{
		if((lResult = ActivatePositionMode(g_pSubKeyHandle, g_nodeIdList[i], &ulErrorCode))==MMC_FAILED)
		{
			LogError("PositionSubMode", lResult, ulErrorCode, g_nodeIdList[i]);
		}
	}
	return lResult;
}

int EposCommunication::startVolicityMode()
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

	if((lResult = ActivateVelocityMode(g_pKeyHandle, g_usNodeId, &ulErrorCode))==MMC_FAILED)
	{
		LogError("ActivateVelocityMode", lResult, ulErrorCode);
	}
	for(int i = 1; i < g_motors; i++)
	{
		if((lResult = ActivateVelocityMode(g_pSubKeyHandle, g_nodeIdList[i], &ulErrorCode))==MMC_FAILED)
		{
			LogError("ActivateVelocityMode Sub", lResult, ulErrorCode, g_nodeIdList[i]);
		}
	}
	return lResult;
}

int EposCommunication::setPositionProfile(unsigned short p_usNodeId, double profile_velocity,
										  double profile_acceleration = 1000,
										  double profile_deceleration = 1000)
{
	// int lResult = MMC_FAILED;
	// unsigned int ulErrorCode = 0;
	// if((lResult = SetPositionProfile(g_pKeyHandle, g_usNodeId, &ulErrorCode))==MMC_FAILED)
	// {
	// 	LogError("SetPositionProfile", lResult, ulErrorCode);
	// }

	// if((lResult = SetPositionProfile(g_pSubKeyHandle, g_usSubNodeId, &ulErrorCode))==MMC_FAILED)
	// {
	// 	LogError("SetSubPositionProfile", lResult, ulErrorCode);
	// }
	unsigned int ulErrorCode = 0;
	int lResult = MMC_SUCCESS;
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;

	int vel_rpm = radsToRpm(profile_velocity);
	if(vel_rpm == 0)
		return lResult;

	if(VCS_SetPositionProfile(p_DeviceHandle, p_usNodeId, vel_rpm, radsToRpm(profile_acceleration), radsToRpm(profile_deceleration), &ulErrorCode) == MMC_FAILED)
	{
		lResult = MMC_FAILED;
		LogError("VCS_SetPositionProfile", lResult, ulErrorCode, p_usNodeId);
		std::cout<< "radsToRpm(profile_velocity) = "<< radsToRpm(profile_velocity)<<std::endl;
	}

	return lResult;
}

bool EposCommunication::deviceOpenedCheck()
{
	return deviceOpenedCheckStatus;
}

int EposCommunication::setPosition(unsigned short p_usNodeId, double position_setpoint){
	//Set position, call this function in service callback:
	int lResult = MMC_SUCCESS;
	unsigned int ulErrorCode = 0;
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;

	//Safety check setpoint and homing:
	if(position_setpoint <= M_PI && position_setpoint >= -1 * M_PI)
	{
		bool absolute = true;

		if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, mmToCounts(position_setpoint), absolute, 1, &ulErrorCode) == MMC_FAILED)
		{
			lResult = MMC_FAILED;
			LogError("VCS_MoveToPosition", lResult, ulErrorCode, p_usNodeId);
		}
		else{
			// ROS_INFO("Movement executed.");
		}
	}
	return lResult;
}

int EposCommunication::setPositionMust(unsigned short p_usNodeId, double position_setpoint)
{
	int lResult = MMC_SUCCESS;
	unsigned int ulErrorCode = 0;
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;

	if(VCS_SetPositionMust(p_DeviceHandle, p_usNodeId, mmToCounts(position_setpoint), &ulErrorCode) == MMC_FAILED)
	{
		LogError("VCS_SetPositionMust", lResult, ulErrorCode, p_usNodeId);
		std::cout<<"position_setpoint: "<<position_setpoint<<", to counts: "<<mmToCounts(position_setpoint)<<std::endl;
		lResult = MMC_FAILED;
	}
	else{
		// ROS_INFO("Movement executed.");
	}

	return lResult;
}

int EposCommunication::setVelocityMust(unsigned short p_usNodeId, double velocity_setpoint)
{
	int lResult = MMC_SUCCESS;
	unsigned int ulErrorCode = 0;
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;

	if(VCS_SetVelocityMust(p_DeviceHandle, p_usNodeId, radsToRpm(velocity_setpoint), &ulErrorCode) == MMC_FAILED)
	{
		LogError("VCS_SetVelocityMust", lResult, ulErrorCode, p_usNodeId);
		std::cout<<"velocity_setpoint: "<<velocity_setpoint<<", to counts: "<<radsToRpm(velocity_setpoint)<<std::endl;
		lResult = MMC_FAILED;
	}
	else{
		// ROS_INFO("Movement executed.");
	}

	return lResult;
}

int EposCommunication::getPosition(unsigned short p_usNodeId, double* pPositionIs)
{
	unsigned int ulErrorCode = 0;
	int pPositionIsCounts = 0;
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;

	// if((lResult = GetPosition(&pPositionIsCounts, &ulErrorCode))==MMC_FAILED)
	// {
	// 	LogError("getPosition", lResult, ulErrorCode);
	// 	return lResult;
	// }

	int lResult = MMC_SUCCESS;

	if(VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, &pPositionIsCounts, &ulErrorCode) == MMC_FAILED)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetPositionIs", lResult, ulErrorCode, p_usNodeId);
	}

	*pPositionIs = countsTomm(&pPositionIsCounts);

	//only for Debugging
	//ROS_INFO_STREAM("!!! pPositionIs: " << *pPositionIs << " pPositionIsCounts: " << pPositionIsCounts);
	return lResult;
}

int EposCommunication::getVelocity(unsigned short p_usNodeId, double* pVelocityIs)
{
	unsigned int ulErrorCode = 0;
	int pVelocityIsCounts;
	HANDLE p_DeviceHandle = (p_usNodeId == g_usNodeId) ? g_pKeyHandle : g_pSubKeyHandle;
	// if((lResult = GetVelocity(&pVelocityIsCounts, &ulErrorCode))==MMC_FAILED)
	// {
	// 	LogError("getVelocity", lResult, ulErrorCode);
	// 	return lResult;
	// }
	int lResult = MMC_SUCCESS;

	if(VCS_GetVelocityIs(p_DeviceHandle, p_usNodeId, &pVelocityIsCounts, &ulErrorCode) == MMC_FAILED)
	{
		lResult = MMC_FAILED;
		LogError("VCS_GetVelocityIs", lResult, ulErrorCode, p_usNodeId);
	}
	*pVelocityIs = rpmToRads(&pVelocityIsCounts);
	return lResult;
}

int EposCommunication::closeDevice(){
	//Close device:
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

	if((lResult = CloseDevice(&ulErrorCode))==MMC_FAILED)
	{
		LogError("CloseDevice", lResult, ulErrorCode);
		return lResult;
	}
	return lResult;
}

double EposCommunication::countsTomm(int* counts){
	double mm = -1 * 2 * M_PI * (*counts) / 4096. / 100.;
	return mm;
}

int EposCommunication::mmToCounts(double mm){
	int counts = -1 * mm  * 4096 * 100 / (2 * M_PI);
	// ROS_INFO_STREAM("counts: " << counts);
	return counts;
}

int EposCommunication::radsToRpm(double rads)
{
	int rpm;
	rpm = -1 * rads * 100 * 60 / (2 * M_PI);
	return rpm;
}

double EposCommunication::rpmToRads(int* rpm)
{
	double rads;
	rads = -1 * (*rpm) / 100. / 60. * (2 * M_PI);
	return rads;
}

	/* workflow:
	 * initialize, setPosition, closeDevice
	 */

} /* namespace */
