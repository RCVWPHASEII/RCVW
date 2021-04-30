//============================================================================
// Name        : RCVWPlugin.cpp
// Author      : Battelle Memorial Institute - Greg Zink (zinkg@battelle.org)
// Version     :
// Copyright   : Battelle 2016
// Description : HRI Status Plugin - sends out a SPAT message every 10ms
//============================================================================

#define GRAVITY 9.81

#include <iostream>
#include <atomic>
#include <thread>
#include <mutex>
#include <math.h>
#include <boost/algorithm/string.hpp>

#include "PluginClient.h"
#include <tmx/j2735_messages/MapDataMessage.hpp>
#include <tmx/j2735_messages/SpatMessage.hpp>
#include <tmx/j2735_messages/RoadSideAlertMessage.hpp>
#include <tmx/messages/message_document.hpp>

#include <LocationMessage.h>
#include <ApplicationMessage.h>
#include <ApplicationMessageEnumTypes.h>
#include <Intersection.h>
#include <MapSupport.h>
#include <ParsedMap.h>
#include <VehicleBasicMessage.h>
#include <TmxMessageManager.h>

#include "HRILocation.h"
#include <Conversions.h>
#include <PluginDataMonitor.h>
#include <FrequencyThrottle.h>
#include <Clock.h>
#include <GeoVector.h>

using namespace std;
using namespace tmx;
using namespace tmx::utils;
using namespace tmx::messages;
using namespace tmx::messages::appmessage;

namespace RCVWPlugin
{

/**
 * <summary>
 * 	This plugin sends out the HRI Status in a SPAT Message
 * </summary>
 */

class RCVWPlugin: public TmxMessageManager
{
public:
	RCVWPlugin(std::string);
	virtual ~RCVWPlugin();
	int Main();

protected:
	void UpdateConfigSettings();

	// virtual method overrides
	void OnConfigChanged(const char *key, const char *value);
	void OnStateChange(IvpPluginState state);

	void HandleMapDataMessage(MapDataMessage &msg,
			routeable_message &routeableMsg);
	void HandleSpatMessage(SpatMessage &msg, routeable_message &routeableMsg);
	void HandleRSAMessage(RsaMessage &msg, routeable_message &routeableMsg);
	void HandleLocationMessage(LocationMessage &msg, routeable_message &routeableMsg);
	void HandleVehicleBasicMessage(VehicleBasicMessage &msg, routeable_message &routeableMsg);

	void HandleDataChangeMessage(DataChangeMessage &msg, routeable_message &routeableMsg);
private:
	std::vector<hri_location_type> _hriLocations;

	//Config Values
	std::mutex _dataLock;
	std::atomic<double> _safetyOffset;
	std::atomic<double> _reactionTime;
	std::atomic<uint64_t> _messageExpiration;
	std::atomic<unsigned int> _outputInterface;
	std::atomic<bool> _configSet;
	std::atomic<bool> _mapReceived;
	std::atomic<bool> _spatReceived;
	std::atomic<double> _distanceToHRI;
	std::atomic<double> _irExtent;
	std::atomic<double> _HRIWarningThresholdSpeed;
	std::atomic<bool> _useCalculatedDeceleration;

	DATA_MONITOR(_safetyOffset);
	DATA_MONITOR(_reactionTime);

	//Values for stopping distance calculation
	std::mutex _locationLock;
	std::atomic<double> _speed;
	std::atomic<double> _prevSpeed;
	std::atomic<double> _prevPrevSpeed;
	std::atomic<uint64_t> _speedTime;
	std::atomic<uint64_t> _prevSpeedTime;
	std::atomic<double> _speedVBM;
	std::atomic<double> _prevSpeedVBM;
	std::atomic<double> _prevPrevSpeedVBM;
	std::atomic<uint64_t> _speedTimeVBM;
	std::atomic<uint64_t> _prevSpeedTimeVBM;
	std::atomic<double> _heading;
	std::atomic<double> _acceleration;
	std::atomic<double> _horizontalDOP;
	std::atomic<double> _mu; // Coefficient of friction, should probably use kinetic friction to be conservative
	std::atomic<double> _weatherFactor;
	std::atomic<double>  _lastCalculatedExpectedStopDistance;
	std::atomic<double>  _lastCalculatedAcceleration;

	DATA_MONITOR(_speed);
	DATA_MONITOR(_mu);

	//Positioning Values
	std::atomic<uint64_t> _lastLocation;
	std::atomic<bool> _locationReceived;
	std::atomic<bool> _rtkReceived;
	std::atomic<bool> _locationProcessed;
	std::atomic<double> _lat;
	std::atomic<double> _long;
	std::atomic<double> _altitude;
	std::atomic<uint8_t> _rtkType;

	//Map Data
	std::atomic<uint64_t> _lastMap;
	MapDataMessage _mapData;

	//SPAT Data
	std::atomic<uint64_t> _lastSpat;
	SpatMessage _spatData;
	std::atomic<bool> _preemption;
	std::atomic<bool> _inLane;

	//VBM Data
	std::atomic<uint64_t> _lastVBM;

	//Warning Queue, lowest to highest priority
	std::atomic<bool> _availableActive;
	std::atomic<bool> _approachInformActive;
	std::atomic<bool> _approachWarningActive;
	std::atomic<bool> _hriWarningActive;
	std::atomic<bool> _errorActive;

	//other
	std::atomic<double> _lastLoggedspeed;
	string _lastLocationTime;
	std::atomic<uint8_t> _stateErrorMessage;
	std::atomic<uint8_t> _changeDirectionCount;

	//V2
	std::atomic<double> _v2AntennaPlacementXMeters;  //measured from front left corner
	std::atomic<double> _v2AntennaPlacementYMeters;  //measured from front left corner
	std::atomic<double> _v2AntennaHeightMeters;
	std::atomic<double> _v2GPSErrorMeters;
	std::atomic<double> _v2ReactionTimeSec;
	std::atomic<double> _v2CommunicationLatencySec;
	std::atomic<double> _v2ApplicationLatencySec;
	std::atomic<double> _v2MinDecelerationCarMPSS;
	std::atomic<double> _v2MinDecelerationLightTruckMPSS;
	std::atomic<double> _v2MinDecelerationHeavyTruckMPSS;
	std::atomic<uint64_t> _v2vehicleType;
	std::atomic<double> _v2vehicleLength;
	std::atomic<bool> _v2useVBMDeceleration;
	std::atomic<bool> _v2LogSPAT;
	std::atomic<uint64_t> _v2CriticalMessageExpiration;
	std::atomic<bool> _v2UseConfigGrade;
	std::atomic<double> _v2Grade;
	std::atomic<bool> _v2CheckRTK;
	std::atomic<bool> _v2CheckLocationFrequency;
	std::atomic<uint64_t> _v2LocationFrequencySampleSize;
	std::atomic<double> _v2MinumumLocationFrequency;
	std::atomic<double> _v2LocationFrequencyTargetIntervalMS;
	std::atomic<double> _v2LocationFrequencyCurrentIntervalMS;
	std::atomic<uint64_t> _v2LocationFrequencyCount;
	std::atomic<double> _v2MaxHeadingChange;
	std::atomic<uint64_t> _v2MaxIgnoredPositions;

	typedef enum V2VehicleTypeEnum
	{
		Car = 1,
		LightTruck = 2,
		HeavyTruck = 3
	} V2VehicleType;

	typedef enum V2RTKTypeEnum
	{
		NA = 0,
		None = 1,
		Float = 2,
		Fixed = 3
	} V2RTKType;

	typedef enum V2StateErrorMessageEnum
	{
		NoError = 0,
		MAP = 1,
		SPAT = 2,
		Location = 3,
		Frequency = 4,
		RTK = 5
	} V2StateErrorMessage;

	//Helper Functions
	void CheckForErrorCondition(double lat, double lon, bool frequencyError);
	bool IsLocationInRangeOfEquippedHRI(double latitude, double longitude);
	bool ParseHRILocationJson(cJSON *root);
	uint64_t GetMsTimeSinceEpoch();
	bool IsDecelerating();
	bool InHRI(double lat, double lon, double speed, double heading);
	//bool HRIPreemptionActive();
	double GetDistanceToCrossing(double lat, double lon, double heading, double& grade);
	double GetStoppingDistance(double speed, double friction, double incline);
	double GetStoppingDistanceV2(double speed, double deceleration, double grade);
	void AlertVehicle();
	void AlertVehicle_2();

	void SendApplicationMessage(tmx::messages::appmessage::EventCodeTypes, tmx::messages::appmessage::Severity, std::string = "", std::string = "", uint64_t = 0);

	void SendAvailable();
	void SendAvailableCleared();
	void SendApproachInform();
	void SendApproachInformCleared();
	void SendApproachWarning();
	void SendApproachWarningCleared();
	void SendHRIWarning();
	void SendHRIWarningCleared();
	void SendError(string message);
	void SendErrorCleared();

};

/**
 * Creates an instance of the HRI Status Plugin
 *
 * @param name Name to identify the plugin instance
 */
RCVWPlugin::RCVWPlugin(std::string name) : TmxMessageManager(name)
{
	//Initialize Atomics
	_distanceToHRI = 480;
	_safetyOffset = 0.0;
	_messageExpiration = 2000;
	_speed = 0;
	_horizontalDOP = 0;
	_mu = 0.0;
	_weatherFactor = 1.0;
	_prevSpeed = 0.0;
	_prevPrevSpeed = 0.0;
	_lat = 0.0;
	_long = 0.0;
	_reactionTime = 1.0;
	_HRIWarningThresholdSpeed = 1.0;
	_useCalculatedDeceleration = false;
	_configSet = false;
	_mapReceived = false;
	_spatReceived = false;
	_locationReceived = false;
	_rtkReceived = false;
	_locationProcessed = true;
	_preemption = false;
	_inLane = false;
	_availableActive = false;
	_approachInformActive = false;
	_approachWarningActive = false;
	_hriWarningActive = false;
	_errorActive = false;
	_lastMap = 0;
	_lastSpat = 0;
	_lastLocation = 0;
	_outputInterface = 0;
	_lastLoggedspeed = -1;
	_speedTime = 0;
	_prevSpeedTime = 0;
	_heading = 0;
	_altitude = 0;
	_acceleration = 0;
	_lastCalculatedExpectedStopDistance = 999999;
	_lastCalculatedAcceleration = 0;
	_lastLocationTime = "";
	_rtkType = V2RTKType::NA;
	_stateErrorMessage = V2StateErrorMessage::NoError;
	_changeDirectionCount = 0;
	_speedVBM = 0;
	_prevSpeedVBM = 0;
	_prevPrevSpeedVBM = 0;
	_speedTimeVBM = 0;
	_prevSpeedTimeVBM = 0;

	_v2AntennaPlacementXMeters = 0.5;
	_v2AntennaPlacementYMeters = 2.5;
	_v2AntennaHeightMeters = 1.5;
	_v2GPSErrorMeters = 3.12;
	_v2ReactionTimeSec = 2.5;
	_v2CommunicationLatencySec = 0.3;
	_v2ApplicationLatencySec = 0.085;
	_v2MinDecelerationCarMPSS = 3.4;
	_v2MinDecelerationLightTruckMPSS = 2.148;
	_v2MinDecelerationHeavyTruckMPSS = 2.322;
	_v2vehicleType = V2VehicleType::Car;
	_v2vehicleLength = 4.8;
	_v2useVBMDeceleration = true;
	_v2LogSPAT = false;
	_v2CriticalMessageExpiration = 500;
	_v2UseConfigGrade = false;
	_v2Grade = 0;
	_v2CheckRTK = true;
	_v2CheckLocationFrequency = true;
	_v2LocationFrequencySampleSize = 10;
	_v2MinumumLocationFrequency = 8.9;
	_v2LocationFrequencyTargetIntervalMS = 1000.0 / _v2MinumumLocationFrequency;
	_v2LocationFrequencyCurrentIntervalMS = 0.0;
	_v2LocationFrequencyCount = 0;
	_v2MaxHeadingChange = 90.0;
	_v2MaxIgnoredPositions = 2;

	//We want to listen for Map/Spat Messages
	AddMessageFilter<MapDataMessage>(this, &RCVWPlugin::HandleMapDataMessage);
	AddMessageFilter<SpatMessage>(this, &RCVWPlugin::HandleSpatMessage);
	AddMessageFilter<LocationMessage>(this, &RCVWPlugin::HandleLocationMessage);
	AddMessageFilter<DataChangeMessage>(this, &RCVWPlugin::HandleDataChangeMessage);
	AddMessageFilter<RsaMessage>(this, &RCVWPlugin::HandleRSAMessage);
	AddMessageFilter<VehicleBasicMessage>(this, &RCVWPlugin::HandleVehicleBasicMessage);

	SubscribeToMessages();
}

/**
 * Default Deconstructor
 */
RCVWPlugin::~RCVWPlugin()
{

}

/**
 * Update the configuration parameters from the database
 */
void RCVWPlugin::UpdateConfigSettings()
{
	GetConfigValue("Friction", _mu);
	GetConfigValue("Safety Offset", _safetyOffset);
	GetConfigValue("Reaction Time", _reactionTime);
	GetConfigValue("Message Expiration", _messageExpiration);
	GetConfigValue("Output Interface", _outputInterface);
	GetConfigValue("Distance To HRI", _distanceToHRI);
	GetConfigValue("Extended Intersection", _irExtent);
	GetConfigValue("HRI Warning Threshold Speed", _HRIWarningThresholdSpeed);
	GetConfigValue("Use Calculated Deceleration", _useCalculatedDeceleration);

	GetConfigValue("V2 Antenna Placement X", _v2AntennaPlacementXMeters);
	GetConfigValue("V2 Antenna Placement Y", _v2AntennaPlacementYMeters);
	GetConfigValue("V2 Antenna Height", _v2AntennaHeightMeters);
	GetConfigValue("V2 GPS Error", _v2GPSErrorMeters);
	GetConfigValue("V2 Reaction Time", _v2ReactionTimeSec);
	GetConfigValue("V2 Communication Latency", _v2CommunicationLatencySec);
	GetConfigValue("V2 Application Latency", _v2ApplicationLatencySec);
	GetConfigValue("V2 Deceleration Car", _v2MinDecelerationCarMPSS);
	GetConfigValue("V2 Deceleration Light Truck", _v2MinDecelerationLightTruckMPSS);
	GetConfigValue("V2 Deceleration Heavy Truck", _v2MinDecelerationHeavyTruckMPSS);
	GetConfigValue("V2 Vehicle Type", _v2vehicleType);
	GetConfigValue("V2 Vehicle Length", _v2vehicleLength);
	GetConfigValue("V2 Use VBM Deceleration", _v2useVBMDeceleration);
	GetConfigValue("V2 Log SPAT", _v2LogSPAT);
	GetConfigValue("V2 Critical Message Expiration", _v2CriticalMessageExpiration);
	GetConfigValue("V2 Use Config Grade", _v2UseConfigGrade);
	GetConfigValue("V2 Grade", _v2Grade);
	GetConfigValue("V2 Check RTK", _v2CheckRTK);
	GetConfigValue("V2 Check Location Frequency", _v2CheckLocationFrequency);
	GetConfigValue("V2 Location Frequency Sample Size", _v2LocationFrequencySampleSize);
	GetConfigValue("V2 Minimum Location Frequency", _v2MinumumLocationFrequency);
	GetConfigValue("V2 Max Heading Change", _v2MaxHeadingChange);
	GetConfigValue("V2 Max Ignored Positions", _v2MaxIgnoredPositions);

	_v2LocationFrequencyTargetIntervalMS = 1000.0 / _v2MinumumLocationFrequency;
	_v2LocationFrequencyCurrentIntervalMS = 0;
	_v2LocationFrequencyCount = 0;

	string rawHRILocations;
	GetConfigValue<string>("HRI Locations", rawHRILocations);

	if (rawHRILocations.length() > 0) {
		cJSON *root = cJSON_Parse(rawHRILocations.c_str());

		if (root != NULL)
		{
			if (ParseHRILocationJson(root)) {
				//DebugPrintMapFiles();
			} else {
				std::cout << "Error parsing HRI Locations config setting." << std::endl;
			}
			cJSON_Delete(root);
		}
	}
	__mu_mon.check();
	__safetyOffset_mon.check();
	__reactionTime_mon.check();

	_configSet = true;
}

/**
 * Actions to perform when a the configuration parameters
 * change in the database
 *
 * @param key
 * @param value
 */
void RCVWPlugin::OnConfigChanged(const char *key, const char *value)
{
	TmxMessageManager::OnConfigChanged(key, value);
	UpdateConfigSettings();
}

/**
 * Actions to perform when the state of the plugin changes
 *
 * @param state
 */
void RCVWPlugin::OnStateChange(IvpPluginState state)
{
	TmxMessageManager::OnStateChange(state);

	if (IvpPluginState::IvpPluginState_registered == state)
	{
		UpdateConfigSettings();
		SetStatus("HRI", "Not Present");
		//SetStatus("Warning", "Not Active");
		SetStatus("Map Received", false);
		SetStatus("Location Received", false);
		SetStatus("RTK Type", "");
		SetStatus("Spat Received", false);
		SetStatus("Near Active HRI", "");
	}
}


void RCVWPlugin::HandleMapDataMessage(MapDataMessage &msg,
			routeable_message &routeableMsg)
{
	//int newIntersectionId = msg.get<int>("MapData.intersections.IntersectionGeometry.id.id", -1);
	int newIntersectionId = msg.get_j2735_data()->intersections->list.array[0]->id.id;
	PLOG(logDEBUG1) << "MAP Received, IntersectionID: " << newIntersectionId;

	if(!_mapReceived)
	{
		Intersection intersection;

		intersection.LoadMap(msg);

		WGS84Point location(_lat, _long);

		//MapSupport mapSupp;
		//MapMatchResult r = mapSupp.FindVehicleLaneForPoint(location, intersection.Map);

		if (!_mapReceived.exchange(true))
			SetStatus("Map Received", true);

		_lastMap = GetMsTimeSinceEpoch();

		std::lock_guard<mutex> lock(_dataLock);
		_mapData = msg;
	}
	else
	{
		//int oldIntersectionId = _mapData.get<int>("MapData.intersections.IntersectionGeometry.id.id", -1);
		int oldIntersectionId = _mapData.get_j2735_data()->intersections->list.array[0]->id.id;
		if(newIntersectionId == oldIntersectionId)
		{
			_lastMap = GetMsTimeSinceEpoch();
		}

	}


}

void RCVWPlugin::HandleSpatMessage(SpatMessage &msg, routeable_message &routeableMsg)
{

//	if(_mapReceived)
//	{
//		MapDataMessage mapCopy;
//		{
//			std::lock_guard<mutex> lock(_dataLock);
//			mapCopy  = _mapData;
//		}
//
//		Intersection intersection;
//		if(intersection.LoadMap(mapCopy))
//		{
//			if(intersection.DoesSpatMatchMap(msg))
//			{
//				if(!_spatReceived)
//				{
//					SetStatus("Spat Received", true);
//				}
//				_spatReceived = true;
//				_lastSpat = GetMsTimeSinceEpoch();
//				std::lock_guard<mutex> lock(_dataLock);
//				_spatData = msg;
//
//			}
//		}
//
//	}


	//int spatInterId = msg.get<int>("SPAT.intersections.IntersectionState.id.id", -1);
	int spatInterId = msg.get_j2735_data()->intersections.list.array[0]->id.id;
	PLOG(logDEBUG1) << "SPAT Received, IntersectionID: " << spatInterId;

	if(_mapReceived)
	{
		//int intersectionId = _mapData.get<int>("MapData.intersections.IntersectionGeometry.id.id", -1);
		int intersectionId = _mapData.get_j2735_data()->intersections->list.array[0]->id.id;
		if(intersectionId  == spatInterId)
		{
			if(!_spatReceived.exchange(true))
				SetStatus("Spat Received", true);

			_lastSpat = GetMsTimeSinceEpoch();

			std::lock_guard<mutex> lock(_dataLock);
			_spatData = msg;
			if (_v2LogSPAT)
				PLOG(logDEBUG) << "SPAT Received: " << msg;
		}
	}

}

void RCVWPlugin::HandleRSAMessage(RsaMessage &msg, routeable_message &routeableMsg)
{
	int itisCode = msg.get<int>("RoadSideAlert.typeEvent", 0);

	PLOG(logDEBUG1) << "RSA Received, ITIS code: " << itisCode;
	//rainy
	_weatherFactor = 0.6;
	//snowy
	//heavy snow
	if(itisCode >= 4866 && itisCode <=4872) //snow
		_weatherFactor = 0.45;
	else if(itisCode >= 4875 && itisCode <=4876) //ice
		_weatherFactor = 0.45;
	else if(itisCode >= 4881 && itisCode <=4888) //rain
		_weatherFactor = 0.6;
	else 
		_weatherFactor = 1;
}

void RCVWPlugin::HandleDataChangeMessage(DataChangeMessage &msg, routeable_message &routeableMsg)
{
	string field = msg.get_untyped(msg.Name, "?");
	if (field == "?")
		return;

	string name = field.substr(1); 	// Truncate the underscore

	//SendApplicationMessage(EventCodeTypes::NOEVENTID, Severity::Info,
	//		name + " value changed to " + msg.get_untyped(msg.NewValue, "?"), name, routeableMsg.get_timestamp());
}

/**
 * Handles location messages as they are received and extracts
 * the needed information.
 *
 * @param msg The decoded version of the LocationMessage.
 * @param routeableMsg the encoded version of the message routed by TMX
 */
void RCVWPlugin::HandleLocationMessage(LocationMessage &msg, routeable_message &routeableMsg)
{
	uint64_t locationInterval = 0;
	double frequency = 0.0;
	location::SignalQualityTypes signalQuality;
	std::lock_guard<mutex> lock(_locationLock);
	string rtkType = "none";
	double heading;
	double headingChange = 0;
	double tmp_double;
	uint64_t tmp_uint64_t;
	//check if this is a duplicate or old position data
	if (msg.get_Time() <= _lastLocationTime)
		return;
	_lastLocationTime = msg.get_Time();
	if(!_locationReceived)
	{
		SetStatus("Location Received", true);
	}
	uint64_t currentTime = GetMsTimeSinceEpoch();
	uint64_t locationTime = std::stoull(msg.get_Time());
	_locationReceived = true;
	locationInterval = locationTime - _lastLocation;
	_lastLocation = locationTime;

	//if its been out for more than twice the critical message timer then restart the sampling
	if (locationInterval > 2 * _v2CriticalMessageExpiration)
	{
		_v2LocationFrequencyCount = 0;
		_v2LocationFrequencyCurrentIntervalMS = 0.0;
	}

	//calculate running average interval between location messages
	//always have one less sampling interval than the sample size of messages
	if (_v2LocationFrequencyCount < _v2LocationFrequencySampleSize)
	{
		//skip calculations for first point
		if (_v2LocationFrequencyCount > 0)
		{
			_v2LocationFrequencyCurrentIntervalMS = ((_v2LocationFrequencyCount - 1) * _v2LocationFrequencyCurrentIntervalMS) + locationInterval;
			_v2LocationFrequencyCurrentIntervalMS = _v2LocationFrequencyCurrentIntervalMS / _v2LocationFrequencyCount;
		}
		_v2LocationFrequencyCount++;
	}
	else
	{
		_v2LocationFrequencyCurrentIntervalMS = ((_v2LocationFrequencyCount - 2) * _v2LocationFrequencyCurrentIntervalMS) + locationInterval;
		_v2LocationFrequencyCurrentIntervalMS = _v2LocationFrequencyCurrentIntervalMS / (_v2LocationFrequencyCount - 1);
	}

	//calculate heading change
	heading = _heading;
	if (msg.get_Heading() > heading)
		headingChange = msg.get_Heading() - heading;
	else
		headingChange = heading - msg.get_Heading();
	//if heading changed more than (configured) degrees then throw point out, only do it (configured) in a row
	if (_changeDirectionCount < _v2MaxIgnoredPositions && headingChange > _v2MaxHeadingChange)
	{
		//keep previous location data except for time
		if (currentTime - _lastVBM > _v2CriticalMessageExpiration)
		{
			//use location data
			_prevSpeedTime.exchange(_speedTime);
			_speedTime = currentTime;
		}
		else
		{
			//use last saved VBM data
			tmp_uint64_t = _prevSpeedTimeVBM;
			_prevSpeedTime = tmp_uint64_t;
			tmp_uint64_t = _speedTimeVBM;
			_speedTime = tmp_uint64_t;
		}
		_changeDirectionCount++;
	}
	else
	{
		//use location message speed if haven't received VBM in expiration interval
		if (currentTime - _lastVBM > _v2CriticalMessageExpiration)
		{
			//use location data
			_prevPrevSpeed.exchange(_prevSpeed);
			_prevSpeed.exchange(_speed);
			_speed = msg.get_Speed_mps();
			_prevSpeedTime.exchange(_speedTime);
			_speedTime = currentTime;
		}
		else
		{
			//use last saved VBM data
			tmp_double = _prevPrevSpeedVBM;
			_prevPrevSpeed = tmp_double;
			tmp_double = _prevSpeedVBM;
			_prevSpeed = tmp_double;
			tmp_double = _speedVBM;
			_speed = tmp_double;
			tmp_uint64_t = _prevSpeedTimeVBM;
			_prevSpeedTime = tmp_uint64_t;
			tmp_uint64_t = _speedTimeVBM;
			_speedTime = tmp_uint64_t;
		}
		_horizontalDOP = msg.get_HorizontalDOP();
		_lat = msg.get_Latitude();
		_long = msg.get_Longitude();
		//change heading only if speed is not zero
		if (_speed != 0)
			_heading = msg.get_Heading();
		_altitude = msg.get_Altitude();
		_changeDirectionCount = 0;
	}

	//check location message for RTK fix
	signalQuality = msg.get_SignalQuality();
	if (signalQuality == location::SignalQualityTypes::RealTimeKinematic)
	{
		_rtkReceived = true;
		if (_rtkType != V2RTKType::Fixed)
			SetStatus("RTK Type", "Fixed");
		_rtkType = V2RTKType::Fixed;
		rtkType = "fixed";
	}
	else if (signalQuality == location::SignalQualityTypes::FloatRTK)
	{
		_rtkReceived = true;
		if (_rtkType != V2RTKType::Float)
			SetStatus("RTK Type", "Float");
		_rtkType = V2RTKType::Float;
		rtkType = "float";
	}
	else
	{
		_rtkReceived = false;
		if (_rtkType != V2RTKType::None)
			SetStatus("RTK Type", "None");
		_rtkType = V2RTKType::None;
		rtkType = "none";
	}

	_locationProcessed = false;
	if (_v2LocationFrequencyCurrentIntervalMS != 0)
		frequency = 1000.0 / _v2LocationFrequencyCurrentIntervalMS;
	else
		frequency = 0.0;
	//PLOG(logDEBUG) << "LOC Id, MsgTime, CurTime: " <<  msg.get_Id() << ", " << msg.get_Time() << ", " << currentTime;
	heading = msg.get_Heading();
	PLOG(logDEBUG) << std::setprecision(6) << "LOC TIME, LOC SPEED, LOC HEADING, SPEED, HEADING, RTK, FREQUENCY: " <<  msg.get_Time() << ", " <<  msg.get_Speed_mps() << ", " << heading << ", " << _speed << ", " << _heading << ", " << rtkType << ", " << frequency;
	if (_changeDirectionCount > 0)
		PLOG(logDEBUG) << "LOC change ignored count: " <<  (int)_changeDirectionCount;

	// Throttle checks to twice a second
	static FrequencyThrottle<string> throttle(chrono::milliseconds(500));
	if (throttle.Monitor(__speed_mon.get_name()))
		__speed_mon.check();
}

/**
 * Handles location messages as they are received and extracts
 * the needed information.
 *
 * @param msg The decoded version of the LocationMessage.
 * @param routeableMsg the encoded version of the message routed by TMX
 */
void RCVWPlugin::HandleVehicleBasicMessage(VehicleBasicMessage &msg, routeable_message &routeableMsg)
{
	std::lock_guard<mutex> lock(_locationLock);
	uint64_t currentTime = GetMsTimeSinceEpoch();
	_lastVBM = currentTime;
	_prevPrevSpeedVBM.exchange(_prevSpeedVBM);
	_prevSpeedVBM.exchange(_speedVBM);
	_speedVBM = msg.get_Speed_mps();
	_prevSpeedTimeVBM.exchange(_speedTimeVBM);
	_speedTimeVBM = currentTime;
	_acceleration = msg.get_Acceleration();
	PLOG(logDEBUG) << std::setprecision(10) << "VBM SPEED, VBM ACCELERATION: " <<  _speedVBM << ", " << _acceleration;

	// Throttle checks to twice a second
	static FrequencyThrottle<string> throttle(chrono::milliseconds(500));
	if (throttle.Monitor(__speed_mon.get_name()))
		__speed_mon.check();
}

/**
 * Function determines if the vehicle is in a HRI or not
 * based on the data acquired from the map message and the
 * current vehicle location.
 *
 * @return true if the vehicle is currently in the HRI, false otherwise.
 */
bool RCVWPlugin::InHRI(double lat, double lon, double speed, double heading)
{
	MapDataMessage mapCopy;
	WGS84Point front;
	WGS84Point back;
	double backwardsHeading;

	{
		std::lock_guard<mutex> lock(_dataLock);
		mapCopy  = _mapData;
	}

	Intersection intersection;
	if(!intersection.LoadMap(mapCopy))
	{
		PLOG(logERROR) << "Problem reading in the current map message.";
		return false;
	}

	WGS84Point location(lat, lon);

	MapSupport mapSupp;
	mapSupp.SetExtendedIntersectionPercentage(_irExtent);

	MapMatchResult r = mapSupp.FindVehicleLaneForPoint(location, intersection.Map);

	if(r.LaneNumber==0){
		return true;
	}

	//check front and back of vehicle
	//calculate front and back points
	front = GeoVector::DestinationPoint(location, heading, _v2AntennaPlacementYMeters);
	backwardsHeading = heading + 180.0;
	if (backwardsHeading >= 360.0)
		backwardsHeading -= 360.0;
	back = GeoVector::DestinationPoint(location, backwardsHeading, _v2vehicleLength - _v2AntennaPlacementYMeters);
	//check points
	r = mapSupp.FindVehicleLaneForPoint(front, intersection.Map);
	if(r.LaneNumber==0){
		return true;
	}
	r = mapSupp.FindVehicleLaneForPoint(back, intersection.Map);
	if(r.LaneNumber==0){
		return true;
	}

	return false;
}

/**
 * Determine if any lane has preemption active
 * ***THIS FUNCTION IS NOT COMPLETE***
 */
//bool RCVWPlugin::HRIPreemptionActive()
//{
//	MapDataMessage mapCopy;
//	SpatMessage spatCopy;
//	{
//		std::lock_guard<mutex> lock(_dataLock);
//		mapCopy = _mapData;
//		spatCopy = _spatData;
//	}
//
//	Intersection intersection;
//	if(!intersection.LoadMap(mapCopy))
//	{
//		PLOG(logERROR) << "Problem reading in the current map message.";
//		return false;
//	}
//
//	// Check to see if SPAT and MAP intersection Ids match.
//	if(!intersection.DoesSpatMatchMap(spatCopy))
//	{
//		return -1;
//	}
//
//	for (std::list<MapLane>::iterator i = intersection.Map.Lanes.begin();
//				i != intersection.Map.Lanes.end(); i++)
//	{
//
//		MapSupport mapSupp;
//		int signalGroup = mapSupp.GetSignalGroupForVehicleLane(i->LaneNumber, intersection.Map);
//
//		std::string spatSeg = "";
//		//Check all three types of lanes that stand for vehicle lanes.
//		if (i->Type == Vehicle || i->Type == Computed || i->Type == Egress)
//		{
//			if(!intersection.IsSignalForGroupRedLight(spatCopy, signalGroup))
//			{
//			}
//		}
//
//	}
//}

/**
 * Function finds the distance to the crossing based on the current
 * location and the data in the Map message
 *
 * @return distance to the crossing in meters, -1 indicates that the vehicle is not in a lane.
 */
double RCVWPlugin::GetDistanceToCrossing(double lat, double lon, double heading, double& grade)
{
	MapDataMessage mapCopy;
	SpatMessage spatCopy;
	{
		std::lock_guard<mutex> lock(_dataLock);
		mapCopy = _mapData;
		spatCopy = _spatData;
	}

	Intersection intersection;

	intersection.LoadMap(mapCopy);

	WGS84Point location(lat, lon);

	MapSupport mapSupp;

	MapMatchResult r = mapSupp.FindVehicleLaneForPoint(location, heading, intersection.Map);
	//check if not in map
	if (r.LaneNumber == -1)
	{
		//not in lane or map
		if (_preemption)
			SetStatus("HRI", "Not Present");
		_preemption = false;
		_inLane = false;
		return -1;
	}
	if (r.LaneNumber > 0)
	{
		_inLane = true;
	}
	else
	{
		_inLane = false;
	}

	// Check to see if SPAT and MAP intersection Ids match.
	if(!intersection.DoesSpatMatchMap(spatCopy))
	{
		return -1;
	}

	int signalGroup = mapSupp.GetSignalGroupForVehicleLane(r.LaneNumber, intersection.Map);

	PLOG(logDEBUG) << "Lane, SignalGroup = " << r.LaneNumber << ", " << signalGroup;
	std::string spatSeg = "";

	if(!intersection.IsSignalForGroupRedLight(spatCopy, signalGroup))
	{
		if (_preemption)
			SetStatus("HRI", "Not Present");
		_preemption = false;
	}
	else
	{
		if (!_preemption)
			SetStatus("HRI", "Present");
		_preemption = true;
	}

	int laneSegment = r.LaneSegment;

	if((r.IsInLane == false && r.IsNearLane == false) || r.LaneNumber < 0 || laneSegment < 1 || r.IsEgress)
	{
		_inLane = false;
		return -1;
	}

	if (_v2UseConfigGrade)
	{
		grade = _v2Grade;
	}
	else
	{
		grade = r.Grade;
	}
	PLOG(logDEBUG) << "IsInLane: " << r.IsInLane << ", IsNearLane: " << r.IsNearLane << ", LaneNumber: " << r.LaneNumber << ", LaneSegment: " << r.LaneSegment << ", Grade: " << grade;

	//!!!!!!!!!!!!!!!!!!!!!!
	//FOR TESTING ONLY
	//!!!!!!!!!!!!!!!!!!!!!!
	//_preemption = true;


	// Calculate the distance to the crossing on a node by node basis
	// to account for curves when approaching the intersection.

	double distance = 0.0;
	int nodeIndex = 0;
	WGS84Point point1;
	WGS84Point point2;
	std::list<LaneNode>::iterator it;
	std::list<LaneNode> t_nodes = intersection.GetLaneNodes(mapCopy, r.LaneNumber, 0.0, 0.0);
	for (it=t_nodes.begin(); it != t_nodes.end() && nodeIndex < laneSegment; ++it)
	{
		point1 = point2;
		point2 = it->Point;
		PLOG(logDEBUG1) << "node: " << point2.Latitude << ", " << point2.Longitude;
		if (nodeIndex > 0)
		{
			//PLOG(logDEBUG1) << "point1: " << point1.Latitude << ", " << point1.Longitude;
			//PLOG(logDEBUG1) << "point2: " << point2.Latitude << ", " << point2.Longitude;
			distance += Conversions::DistanceMeters(point1, point2);
			//PLOG(logDEBUG1) << "distance: " << distance;
		}
		nodeIndex++;
	}
	distance += Conversions::DistanceMeters(point2, location);
	PLOG(logDEBUG1) << "final distance: " << distance;

//the code below causes seg fault, not sure why, about the same as code above that replaces it
//	auto node = intersection.GetLaneNodes(mapCopy, r.LaneNumber, 0.0, 0.0).begin();
//
//	for(int index = 0; index < nextNode; ++index)
//	{
//		WGS84Point point1 = node->Point;
//		PLOG(logDEBUG1) << "point1: " << point1.Latitude << ", " << point1.Longitude;
//		node++;
//		WGS84Point point2 = node->Point;
//		PLOG(logDEBUG1) << "point2: " << point2.Latitude << ", " << point2.Longitude;
//		distance += Conversions::DistanceMeters(point1, point2);
//		PLOG(logDEBUG1) << "distance: " << distance;
//	}
//
//	distance += Conversions::DistanceMeters(node->Point, location);
//	PLOG(logDEBUG1) << "final distance: " << distance;

	return distance;
}

/**
 * Function to calculate the stopping distance needed based on
 * the current speed of the vehicle and the coefficient of
 * friction between the tires and road for the current vehicle.
 * There are a couple of assumptions that simplify the calculation
 * of stopping distance. By including a safety offset to error on the side
 * of caution we can ignore the fact that friction is not constant in vehicle
 * dynamics.
 *
 * @param speed The current speed of the vehicle in m/s
 * @param friction The coefficient of friction between the tires and the road surface for the current vehicle
 * @param incline the slope of the road in degrees, positive is incline negative is decline
 *
 * @return The distance needed to stop in meters
 */
double RCVWPlugin::GetStoppingDistance(double speed, double friction, double incline)
{
	double distance = 0.0;
	distance = (_reactionTime * speed) + ((speed * speed) / (2 * 9.8 * ((friction * cos(incline)) + sin(incline))));
	return distance;
}

/**
 * Function to calculate the stopping distance needed based on
 * the current speed of the vehicle.
 *
 * @param speed The current speed of the vehicle in m/s
 * @param deceleration The min deceleration for this vehicle in m/s^2
 * @param grade rise/run
 *
 * @return The distance needed to stop in meters
 */
double RCVWPlugin::GetStoppingDistanceV2(double speed, double deceleration, double grade)
{
	double distance = 0.0;
	//if the vehicle is not moving (speed is zero which it should be with speed clamping) then return zero
	if (speed == 0.0)
		return distance;
	double v = units::Convert<units::Speed, units::Speed::mps, units::Speed::kph>(speed); //change to kph for formula
	double t = _v2ReactionTimeSec + _v2CommunicationLatencySec + _v2ApplicationLatencySec;
	distance = _v2AntennaPlacementYMeters + _v2GPSErrorMeters + (0.278 * v * t) +
			((v * v) / (254 * ((deceleration / 9.81) + grade)));
	return distance;
}



/**
 * Function determines if the vehicle is decelerating based on
 * the data received from the location messages.
 *
 * @return true if decelerating, false otherwise
 */
bool RCVWPlugin::IsDecelerating()
{
	if(_speed < _prevSpeed && _speed < _prevPrevSpeed) // current speed is compared against two previous speeds to avoid false positives.
	{
		return true;
	}

	return false;
}

void RCVWPlugin::SendApplicationMessage(EventCodeTypes eventCode, Severity sev, std::string txt, std::string interaction, uint64_t time)
{
	static routeable_message timer;
	timer.refresh_timestamp();

	ApplicationMessage msg;
	msg.set_Id(NewGuid());
	msg.set_AppId(ApplicationTypes::RCVW);
	msg.set_EventCode(eventCode);
	msg.set_Severity(sev);
	msg.set_CustomText(txt);
	if (!interaction.empty())
		msg.set_InteractionId(interaction);
	if (time > 0)
		msg.set_Timestamp(to_string(time));
	else
		msg.set_Timestamp(to_string(timer.get_timestamp()));

	BroadcastMessage(msg);
}

/*
 * Send messages to UI
 */


void RCVWPlugin::SendAvailable()
{
	PLOG(logDEBUG) << "Sending Application Message: Available";
	SetStatus("Available", "Active");
	SendApplicationMessage(EventCodeTypes::RCVW2Available, Severity::Inform);
}
void RCVWPlugin::SendAvailableCleared()
{
	PLOG(logDEBUG) << "Sending Application Message: Clear Available";
	SetStatus("Available", "");
	SendApplicationMessage(EventCodeTypes::RCVW2Available, Severity::Info);
}
void RCVWPlugin::SendApproachInform()
{
	PLOG(logDEBUG) << "Sending Application Message: ApproachInform";
	SetStatus("ApproachInform", "Active");
	SendApplicationMessage(EventCodeTypes::RCVW2ApproachInform, Severity::Inform);
}
void RCVWPlugin::SendApproachInformCleared()
{
	PLOG(logDEBUG) << "Sending Application Message: Clear ApproachInform";
	SetStatus("ApproachInform", "");
	SendApplicationMessage(EventCodeTypes::RCVW2ApproachInform, Severity::Info);
}
void RCVWPlugin::SendApproachWarning()
{
	PLOG(logDEBUG) << "Sending Application Message: ApproachWarning";
	SetStatus("ApproachWarning", "Active");
	SendApplicationMessage(EventCodeTypes::RCVW2ApproachWarning, Severity::Inform);
}
void RCVWPlugin::SendApproachWarningCleared()
{
	PLOG(logDEBUG) << "Sending Application Message: Clear ApproachWarning";
	SetStatus("ApproachWarning", "");
	SendApplicationMessage(EventCodeTypes::RCVW2ApproachWarning, Severity::Info);
}
void RCVWPlugin::SendHRIWarning()
{
	PLOG(logDEBUG) << "Sending Application Message: HRIWarning";
	SetStatus("HRIWarning", "Active");
	SendApplicationMessage(EventCodeTypes::RCVW2HRIWarning, Severity::Inform);
}
void RCVWPlugin::SendHRIWarningCleared()
{
	PLOG(logDEBUG) << "Sending Application Message: Clear HRIWarning";
	SetStatus("HRIWarning", "");
	SendApplicationMessage(EventCodeTypes::RCVW2HRIWarning, Severity::Info);
}
void RCVWPlugin::SendError(string message)
{
	PLOG(logDEBUG) << "Sending Application Message: Error: " << message;
	SetStatus("Error", "Active: " + message);
	SendApplicationMessage(EventCodeTypes::RCVW2Error, Severity::Inform, message);
}
void RCVWPlugin::SendErrorCleared()
{
	PLOG(logDEBUG) << "Sending Application Message: Clear Error";
	SetStatus("Error", "");
	SendApplicationMessage(EventCodeTypes::RCVW2Error, Severity::Info);
}



void RCVWPlugin::AlertVehicle_2()
{
	bool logCalculations = false;
	bool checkDeceleration = false;
	double speed;
	double prevSpeed;
	uint64_t speedTime;
	uint64_t prevSpeedTime;
	double lat;
	double lon;
	double hdop;
	double grade = 0;
	bool inHRI = false;
	uint64_t lastVBM;
	uint64_t v2CriticalMessageExpiration;
	bool locationProcessed;

	float heading;
	{
		std::lock_guard<mutex> lock(_locationLock);
		speed = _speed;
		prevSpeed = _prevSpeed;
		hdop = _horizontalDOP;
		speedTime = _speedTime;
		prevSpeedTime = _prevSpeedTime;
		lat = _lat;
		lon = _long;
		heading = _heading;
		lastVBM = _lastVBM;
		v2CriticalMessageExpiration = _v2CriticalMessageExpiration;
		locationProcessed = _locationProcessed;
		_locationProcessed = true;
	}

	//if we have already processed this location then skip processing
	if (locationProcessed)
		return;

	uint64_t currentTime = GetMsTimeSinceEpoch();

	//calculate crossing distance, safe stopping distance, and set preemption
	//crossing distance = -1 if not in a lane

	double crossingDistance = GetDistanceToCrossing(lat, lon, heading, grade);

	//log data and calculations only if vehicle is not stopped (with location plugin latching we should get a zero speed)
	//log the data after the GetDistanceToCrossing call because _preemption is set there
	if (_lastLoggedspeed > 0 || speed > 0)
	{
		PLOG(logDEBUG) << std::setprecision(10) << "Latitude: " << lat << ", Longitude: " << lon << ", Speed: " << speed << ", PrevSpeed: " << prevSpeed << ", HDOP: " << hdop << ", Preemption: " << _preemption;
		double lastLoggedspeed = speed;
		_lastLoggedspeed = lastLoggedspeed;
		logCalculations = true;
	}

	double mu = _mu * _weatherFactor;
	double safetyStopDistanceV1 = GetStoppingDistance(speed, mu, 0.0) * _safetyOffset;
	double safetyStopDistance;
	if (_v2vehicleType == V2VehicleType::Car)
		 safetyStopDistance = GetStoppingDistanceV2(speed, _v2MinDecelerationCarMPSS, grade);
	else if (_v2vehicleType == V2VehicleType::LightTruck)
		 safetyStopDistance = GetStoppingDistanceV2(speed, _v2MinDecelerationLightTruckMPSS, grade);
	else if (_v2vehicleType == V2VehicleType::HeavyTruck)
		 safetyStopDistance = GetStoppingDistanceV2(speed, _v2MinDecelerationHeavyTruckMPSS, grade);
	else
		 safetyStopDistance = GetStoppingDistanceV2(speed, _v2MinDecelerationCarMPSS, grade);

	double expectedStopDistance = 0;
	double acceleration;

	//calculate acceleration
	if (!locationProcessed)
	{
		//have new data
		if(speed < prevSpeed)
		{
			if (_useCalculatedDeceleration)
				checkDeceleration = true;
			//calculate expected stop distance due to deceleration
			acceleration = (speed - prevSpeed) / (((double)(speedTime - prevSpeedTime)) / 1000);
			expectedStopDistance = (-1 * (speed * speed)) / (2 * acceleration);
			_lastCalculatedAcceleration = acceleration;
			_lastCalculatedExpectedStopDistance = expectedStopDistance;
			PLOG(logDEBUG) << std::setprecision(10) << "Calculated Acceleration: " << acceleration << ", expectedStopDistance: " << expectedStopDistance;
		}
		else
		{
			_lastCalculatedAcceleration = 0;
			_lastCalculatedExpectedStopDistance = 999999;
		}
	}
	else
	{
		//reuse old data
		if (_lastCalculatedAcceleration < 0)
		{
			if (_useCalculatedDeceleration)
				checkDeceleration = true;
			acceleration = _lastCalculatedAcceleration;
			expectedStopDistance = _lastCalculatedExpectedStopDistance;
			PLOG(logDEBUG) << std::setprecision(10) << "Calculated Acceleration: " << acceleration << ", expectedStopDistance: " << expectedStopDistance;
		}
	}

	//check for valid deceleration
	if (currentTime - lastVBM <= v2CriticalMessageExpiration && _acceleration < 0)
	{
		if (_v2useVBMDeceleration)
			checkDeceleration = true;
		//calculate expected stop distance due to deceleration
		expectedStopDistance = (-1 * (speed * speed)) / (2 * _acceleration);
		PLOG(logDEBUG) << std::setprecision(10) << "VBM Acceleration: " << _acceleration << ", expectedStopDistance: " << expectedStopDistance;
	}

	if (logCalculations)
	{
		//PLOG(logDEBUG) << "VSE: " << _mu << ", WeatherFactor: " << _weatherFactor <<  ", ReactionTime: " << _reactionTime;
		PLOG(logDEBUG) << std::setprecision(10) << "CrossingDistance: " << crossingDistance << ", SafetyStopDistanceV1: " << safetyStopDistanceV1 << ", SafetyStopDistance: " << safetyStopDistance << ", ExpectedStopDistance: " << expectedStopDistance;
	}


	inHRI = InHRI(lat, lon, speed, heading);

	if (!_availableActive)
	{
		if (_inLane || inHRI)
		{
			_availableActive = true;
			SendAvailable();
		}
	}
	else
	{
		if (!_inLane && !inHRI)
		{
			_availableActive = false;
			SendAvailableCleared();
		}
	}

	if (!_approachInformActive)
	{
		if (_preemption && !inHRI)
		{
			_approachInformActive = true;
			SendApproachInform();
		}
	}
	else
	{
		if (!_preemption || inHRI)
		{
			_approachInformActive = false;
			SendApproachInformCleared();
		}
	}

	if (!_approachWarningActive)
	{
		if (_preemption  && !inHRI &&
				crossingDistance < safetyStopDistance &&
				(!checkDeceleration || expectedStopDistance > crossingDistance))
		{
			_approachWarningActive = true;
			SendApproachWarning();
		}
	}
	else
	{
		if (!_preemption || inHRI ||
				crossingDistance >= safetyStopDistance ||
				(checkDeceleration && expectedStopDistance <= crossingDistance))
		{
			_approachWarningActive = false;
			SendApproachWarningCleared();
		}
	}


	if (!_hriWarningActive)
	{
		if (inHRI && speed <= _HRIWarningThresholdSpeed)
		{
			_hriWarningActive = true;
			SendHRIWarning();
		}
	}
	else
	{
		if (!inHRI || speed > _HRIWarningThresholdSpeed)
		{
			_hriWarningActive = false;
			SendHRIWarningCleared();
		}
	}


}


/**
 * Generates a time to use for timestamps.
 *
 * @return integer timestamp in ms
 */
uint64_t RCVWPlugin::GetMsTimeSinceEpoch()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (uint64_t) ((double) (tv.tv_sec) * 1000
			+ (double) (tv.tv_usec) / 1000);
}

bool RCVWPlugin::ParseHRILocationJson(cJSON *root) {
	if (root == NULL)
		return false;

	cJSON *latitudecJson;
	cJSON *longitudecJson;
	cJSON *namecJson;

	cJSON *item = cJSON_GetObjectItem(root, "HRIs");

	if (item == NULL || item->type != cJSON_Array)
		return false;

	_hriLocations.clear();

	for (int i = 0; i < cJSON_GetArraySize(item); i++) {
		cJSON *subitem = cJSON_GetArrayItem(item, i);

		latitudecJson = cJSON_GetObjectItem(subitem, "Latitude");
		longitudecJson = cJSON_GetObjectItem(subitem, "Longitude");
		namecJson = cJSON_GetObjectItem(subitem, "HRIName");

		if (latitudecJson == NULL || latitudecJson->type != cJSON_Number
			|| longitudecJson == NULL || longitudecJson->type != cJSON_Number
				|| namecJson == NULL || namecJson->type != cJSON_String)
			return false;

		hri_location_type tempLocation;
		tempLocation.latitude = latitudecJson->valuedouble;
		tempLocation.longitude = longitudecJson->valuedouble;
		tempLocation.name = namecJson->valuestring;

		_hriLocations.push_back(tempLocation);
	}

	return true;
}

bool RCVWPlugin::IsLocationInRangeOfEquippedHRI(double latitude, double longitude)
{
	for(hri_location_type& tempLocation : _hriLocations)
	{
		double distanceToHRI = Conversions::DistanceMeters(latitude, longitude,
				tempLocation.latitude, tempLocation.longitude);

		if(distanceToHRI <= _distanceToHRI)
		{
			SetStatus("Near Active HRI", tempLocation.name);
			return true;
		}
	}
	SetStatus("Near Active HRI", "");
	return false;
}

/**
 * Plugin Logic to run after initialization
 *
 * @return normal exit code
 */
int RCVWPlugin::Main()
{
	uint64_t currentTime;
	double bestNextLocationInterval;
	double testLocationInterval;
	bool frequencyError = false;
	while(!_configSet)
	{
		usleep(1000);
	}


	PLOG(logINFO) << "Starting Plugin";

	while (_plugin->state != IvpPluginState_error)
	{
		frequencyError = false;
		//if we have at least 3 samples we have an average interval
		if (_v2CheckLocationFrequency && _v2LocationFrequencyCount > 2)
		{
			//test if interval out of range
			if (_v2LocationFrequencyCurrentIntervalMS > _v2LocationFrequencyTargetIntervalMS)
				frequencyError = true;
		}
		if(!_mapReceived || !_spatReceived || !_locationReceived || (_v2CheckRTK && !_rtkReceived) || frequencyError)
		{
			CheckForErrorCondition(_lat, _long, frequencyError);
			usleep(100000);
			continue;
		}
		else
		{
			//No longer in Error Condition if we get here, cancel any errors and move on
			if(_errorActive)
			{
				SendErrorCleared();
				_errorActive = false;
			}
		}

		uint64_t curTime = GetMsTimeSinceEpoch();
		bool messageCheck = false;

		if(curTime - _lastSpat > _v2CriticalMessageExpiration)
		{
			if(_spatReceived)
			{
				SetStatus("Spat Received", false);
				_spatReceived = false;
			}
			messageCheck = true;
		}

		if(curTime - _lastMap > _messageExpiration)
		{
			if(_mapReceived)
			{
				SetStatus("Map Received", false);
				_mapReceived = false;
			}
			messageCheck = true;
		}

		if(curTime - _lastLocation > _v2CriticalMessageExpiration)
		{
			if(_locationReceived)
			{
				SetStatus("Location Received", false);
				SetStatus("RTK Type", "");
				_locationReceived = false;
			}
			messageCheck = true;
		}

		//If any of the messages have expired skip the other calculations.
		if(messageCheck)
		{
			//usleep(500000);
			continue;
		}

		AlertVehicle_2();
		usleep(10000);
	}


	return 0;
}


void RCVWPlugin::CheckForErrorCondition(double lat, double lon, bool frequencyError)
{
	bool isInRangeOfHRI = IsLocationInRangeOfEquippedHRI(lat, lon);

	bool isErrorCondition = false;
	string errorMessage = "";

	//Check for error condition
	if(isInRangeOfHRI && !_mapReceived)
	{
		//no MAP and in range of HRI
		isErrorCondition = true;
		errorMessage = "MAP Data Not Received";
		if (_stateErrorMessage != V2StateErrorMessage::MAP)
		{
			_stateErrorMessage = V2StateErrorMessage::MAP;
			SetStatus("Error", errorMessage);
		}
	}
	else if(isInRangeOfHRI && !_spatReceived)
	{
		//no SPAT and in range of HRI
		isErrorCondition = true;
		errorMessage = "SPAT Data Not Received";
		if (_stateErrorMessage != V2StateErrorMessage::SPAT)
		{
			_stateErrorMessage = V2StateErrorMessage::SPAT;
			SetStatus("Error", errorMessage);
		}
	}
	else if(!_locationReceived)
	{
		//no location (GPS)
		isErrorCondition = true;
		errorMessage = "Location Data Not Received";
		if (_stateErrorMessage != V2StateErrorMessage::Location)
		{
			_stateErrorMessage = V2StateErrorMessage::Location;
			SetStatus("Error", errorMessage);
		}
	}
	else if(frequencyError)
	{
		//GPS frequency less than target
		isErrorCondition = true;
		errorMessage = "Location Data Frequency Too Low";
		if (_stateErrorMessage != V2StateErrorMessage::Frequency)
		{
			_stateErrorMessage = V2StateErrorMessage::Frequency;
			SetStatus("Error", errorMessage);
		}
	}
	else if(_v2CheckRTK && isInRangeOfHRI && !_rtkReceived)
	{
		//have location but no RTK fix and in range of HRI
		isErrorCondition = true;
		errorMessage = "RTK Data Not Received";
		if (_stateErrorMessage != V2StateErrorMessage::RTK)
		{
			_stateErrorMessage = V2StateErrorMessage::RTK;
			SetStatus("Error", errorMessage);
		}
	}
	else if(!isInRangeOfHRI)
	{
		//clear error condition if out of range of HRI and it's not a location (GPS) error
		isErrorCondition = false;
	}

	if(isErrorCondition && !_errorActive)
	{
		//send error and clear all other conditions
		SendError(errorMessage);
		_errorActive = true;
		if (_availableActive)
		{
			_availableActive = false;
			SendAvailableCleared();
		}
		if (_approachInformActive)
		{
			_approachInformActive = false;
			SendApproachInformCleared();
		}
		if (_approachWarningActive)
		{
			_approachWarningActive = false;
			SendApproachWarningCleared();
		}
		if (_hriWarningActive)
		{
			_hriWarningActive = false;
			SendHRIWarningCleared();
		}
	}

	if(!isErrorCondition && _errorActive)
	{
		_stateErrorMessage = V2StateErrorMessage::NoError;
		SendErrorCleared();
		_errorActive = false;
	}
}


} /* namespace RCVWPlugin */

int main(int argc, char *argv[])
{
	return run_plugin<RCVWPlugin::RCVWPlugin>("RCVWPlugin", argc, argv);
}
