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

#include "HRILocation.h"
#include <Conversions.h>
#include <PluginDataMonitor.h>
#include <FrequencyThrottle.h>
#include <Clock.h>

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

class RCVWPlugin: public PluginClient
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
	std::atomic<bool> _useDeceleration;

	DATA_MONITOR(_safetyOffset);
	DATA_MONITOR(_reactionTime);

	//Values for stopping distance calculation
	std::mutex _locationLock;
	std::atomic<double> _speed;
	std::atomic<double> _prevSpeed;
	std::atomic<double> _prevPrevSpeed;
	std::atomic<double> _horizontalDOP;
	std::atomic<double> _mu; // Coefficient of friction, should probably use kinetic friction to be conservative
	std::atomic<double> _weatherFactor;
	std::atomic<uint64_t> _speedTime;
	std::atomic<uint64_t> _prevSpeedTime;

	DATA_MONITOR(_speed);
	DATA_MONITOR(_mu);

	//Positioning Values
	std::atomic<uint64_t> _lastLocation;
	std::atomic<bool> _locationReceived;
	std::atomic<double> _lat;
	std::atomic<double> _long;

	//Map Data
	std::atomic<uint64_t> _lastMap;
	MapDataMessage _mapData;

	//SPAT Data
	std::atomic<uint64_t> _lastSpat;
	SpatMessage _spatData;
	std::atomic<bool> _preemption;

	//Warning Queue
	std::atomic<uint64_t> _lastWarning;
	std::atomic<bool> _enteredArea;
	std::atomic<bool> _warningActive;
	std::atomic<bool> _inHRIWarning;
	std::atomic<bool> _inHRISevereWarning;

	DATA_MONITOR(_enteredArea);

	std::atomic<bool> _errorActive;

	std::atomic<double> _lastLoggedspeed;

	//Helper Functions
	bool CheckForOnTrackWarning(double lat, double lon, double speed);
	void CheckForErrorCondition(double lat, double lon);
	bool IsLocationInRangeOfEquippedHRI(double latitude, double longitude);
	bool ParseHRILocationJson(cJSON *root);
	uint64_t GetMsTimeSinceEpoch();
	bool IsDecelerating();
	bool InHRI(double lat, double lon);
	//bool HRIPreemptionActive();
	double GetDistanceToCrossing(double lat, double lon);
	double GetStoppingDistance(double speed, double friction, double incline);
	void AlertVehicle();
	void AlertVehicle_2();

	void SendApplicationMessage(tmx::messages::appmessage::EventCodeTypes, tmx::messages::appmessage::Severity, std::string = "", std::string = "", uint64_t = 0);
	void SendApplicationActive();
	void SendApplicationInactive();
	void SendAlert();
	void SendSevereAlert();
	void SendAlertCleared();
	void SendInHRIWarning();
	void SendInHRISevereWarning();
	void SendInHRIWarningAlertCleared();
	void SendError(string message);
	void SendErrorCleared();

};

/**
 * Creates an instance of the HRI Status Plugin
 *
 * @param name Name to identify the plugin instance
 */
RCVWPlugin::RCVWPlugin(std::string name) : PluginClient(name)
{
	//Initialize Atomics
	_safetyOffset = 0.0;
	_messageExpiration = 6000;
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
	_useDeceleration = false;
	_configSet = false;
	_mapReceived = false;
	_spatReceived = false;
	_locationReceived = false;
	_enteredArea = false;
	_warningActive = false;
	_preemption = false;
	_errorActive = false;
	_inHRIWarning=false;
	_inHRISevereWarning= false;
	_lastMap = 0;
	_lastSpat = 0;
	_lastLocation = 0;
	_outputInterface = 0;
	_lastLoggedspeed = -1;
	_speedTime = 0;
	_prevSpeedTime = 0;

	//We want to listen for Map/Spat Messages
	AddMessageFilter<MapDataMessage>(this, &RCVWPlugin::HandleMapDataMessage);
	AddMessageFilter<SpatMessage>(this, &RCVWPlugin::HandleSpatMessage);
	AddMessageFilter<LocationMessage>(this, &RCVWPlugin::HandleLocationMessage);
	AddMessageFilter<DataChangeMessage>(this, &RCVWPlugin::HandleDataChangeMessage);
	AddMessageFilter<RsaMessage>(this, &RCVWPlugin::HandleRSAMessage);

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
	GetConfigValue("Use Deceleration", _useDeceleration);

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
	PluginClient::OnConfigChanged(key, value);
	UpdateConfigSettings();
}

/**
 * Actions to perform when the state of the plugin changes
 *
 * @param state
 */
void RCVWPlugin::OnStateChange(IvpPluginState state)
{
	PluginClient::OnStateChange(state);

	if (IvpPluginState::IvpPluginState_registered == state)
	{
		UpdateConfigSettings();
		SetStatus("HRI", "Not Present");
		SetStatus("Warning", "Not Active");
		SetStatus("Map Received", false);
		SetStatus("Location Received", false);
		SetStatus("Spat Received", false);
		SetStatus("Near Active HRI", "");
	}
}


void RCVWPlugin::HandleMapDataMessage(MapDataMessage &msg,
			routeable_message &routeableMsg)
{
	int newIntersectionId = msg.get<int>("MapData.intersections.IntersectionGeometry.id.id", -1);

	PLOG(logDEBUG1) << "MAP Received, IntersectionID: " << newIntersectionId;

	if(!_mapReceived)
	{
		Intersection intersection;

		intersection.LoadMap(msg);

		WGS84Point location(_lat, _long);

		MapSupport mapSupp;

		MapMatchResult r = mapSupp.FindVehicleLaneForPoint(location, intersection.Map);

		std::lock_guard<mutex> lock(_dataLock);
		_mapData = msg;
		_lastMap = GetMsTimeSinceEpoch();
		_mapReceived = true;
		SetStatus("Map Received", true);
	}
	else
	{
		int oldIntersectionId = _mapData.get<int>("MapData.intersections.IntersectionGeometry.id.id", -1);
		if(newIntersectionId == oldIntersectionId)
		{
			_lastMap = GetMsTimeSinceEpoch();
		}

	}


}

void RCVWPlugin::HandleSpatMessage(SpatMessage &msg, routeable_message &routeableMsg)
{
	int spatInterId = msg.get<int>("SPAT.intersections.IntersectionState.id.id", -1);

	PLOG(logDEBUG1) << "SPAT Received, IntersectionID: " << spatInterId;

	if(_mapReceived)
	{
		int intersectionId = _mapData.get<int>("MapData.intersections.IntersectionGeometry.id.id", -1);

		if(intersectionId  == spatInterId)
		{
			if(!_spatReceived)
			{
				SetStatus("Spat Received", true);
			}
			_spatReceived = true;
			_lastSpat = GetMsTimeSinceEpoch();
			std::lock_guard<mutex> lock(_dataLock);
			_spatData = msg;
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

	SendApplicationMessage(EventCodeTypes::NOEVENTID, Severity::Info,
			name + " value changed to " + msg.get_untyped(msg.NewValue, "?"), name, routeableMsg.get_timestamp());
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
	if(!_locationReceived)
	{
		SetStatus("Location Received", true);
	}
	std::lock_guard<mutex> lock(_locationLock);
	_locationReceived = true;
	_lastLocation = GetMsTimeSinceEpoch();
	_prevPrevSpeed.exchange(_prevSpeed);
	_prevSpeed.exchange(_speed);
	_speed = msg.get_Speed_kph() / 3.6; // convert the speed from kph to m/s
	_horizontalDOP = msg.get_HorizontalDOP();
	_lat = msg.get_Latitude();
	_long = msg.get_Longitude();
	_prevSpeedTime.exchange(_speedTime);
	_speedTime = GetMsTimeSinceEpoch();

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
bool RCVWPlugin::InHRI(double lat, double lon)
{
	MapDataMessage mapCopy;
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
//			if(!intersection.IsSignalForGroupRedLight(signalGroup, spatCopy, spatSeg))
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
double RCVWPlugin::GetDistanceToCrossing(double lat, double lon)
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

	MapMatchResult r = mapSupp.FindVehicleLaneForPoint(location, intersection.Map);

	// Check to see if SPAT and MAP intersection Ids match.
	if(!intersection.DoesSpatMatchMap(spatCopy))
	{
		return -1;
	}

	int signalGroup = mapSupp.GetSignalGroupForVehicleLane(r.LaneNumber, intersection.Map);

	std::string spatSeg = "";
	if(!intersection.IsSignalForGroupRedLight(signalGroup, spatCopy, spatSeg))
	{
		_preemption = false;
	}
	else
	{
		_preemption = true;
	}

	int laneSegment = r.LaneSegment;

	if(r.IsInLane == false || r.LaneNumber < 0 || laneSegment < 1)
	{
		return -1;
	}

	PLOG(logDEBUG1) << "IsInLane: " << r.IsInLane << ", LaneNumber: " << r.LaneNumber << ", LaneSegment: " << r.LaneSegment;

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

/**
 * Function executes the appropriate logic to
 * display that the system is active based on the
 * configured output interface
 */
void RCVWPlugin::SendApplicationActive()
{
	SetStatus("HRI", "Present");
	PLOG(logDEBUG) << "Sending Application Message: Application Active";
	SendApplicationMessage(EventCodeTypes::ApplicationActive, Severity::Info, "Application Active");
}

/**
 * Function executes the appropriate logic to
 * set the system to inactive based on the
 * configured output interface
 */
void RCVWPlugin::SendApplicationInactive()
{
	SetStatus("HRI", "Not Present");
	SetStatus("Warning", "Not Active");
	PLOG(logDEBUG) << "Sending Application Message: Application Inactive";
	SendApplicationMessage(EventCodeTypes::ApplicationInactive, Severity::Info, "Application Inactive");
}

/**
 * Function executes the appropriate logic to alert the
 * driver based on the configured output interface.
 */
void RCVWPlugin::SendAlert()
{
	PLOG(logDEBUG) << "Sending Application Message: Rail Crossing Violation Warning";
	SetStatus("Warning", "Active");
	SendApplicationMessage(EventCodeTypes::RCVWAlert, Severity::Inform, "RCVW");
}

/**
 * Function executes the appropriate logic to alert the
 * driver based on the configured output interface.
 */
void RCVWPlugin::SendSevereAlert()
{
	PLOG(logDEBUG) << "Sending Application Message: Rail Crossing Violation Warning - Severe";
	SetStatus("Warning", "Active-Severe");
	SendApplicationMessage(EventCodeTypes::RCVWAlert, Severity::Warning, "RCVW");
}

/**
 * Function executes the appropriate logic to clear the alert
 * due to the driver reacting to the alert based on the configured
 * output interface.
 */
void RCVWPlugin::SendAlertCleared()
{
	PLOG(logDEBUG) << "Sending Application Message: Clear Rail Crossing Violation Warning";
	SetStatus("Warning", "RCVW Not Active");
	SendApplicationMessage(EventCodeTypes::RCVWAlert, Severity::Info);
}

void RCVWPlugin::SendInHRIWarning()
{
	PLOG(logDEBUG) << "Sending Application Message: Stopped In HRI Warning";
	SetStatus("Warning", "In HRI Warning Active");
	SendApplicationMessage(EventCodeTypes::RCVWWarning, Severity::Inform, "Stopped in HRI");
}

void RCVWPlugin::SendInHRISevereWarning()
{
	PLOG(logDEBUG) << "Sending Application Message: Stopped In HRI Warning - Severe";
	SetStatus("Warning", "In HRI Warning Active-Severe");
	SendApplicationMessage(EventCodeTypes::RCVWWarning, Severity::Warning, "Stopped in HRI");
}

void RCVWPlugin::SendInHRIWarningAlertCleared()
{
	PLOG(logDEBUG) << "Sending Application Message: Clear Stopped In HRI Warning";
	SetStatus("Warning", "In HRI Warning Active");
	SendApplicationMessage(EventCodeTypes::RCVWWarning, Severity::Info);
}


/**
 * Function executes that appropriate logic to report an error
 * in the current location data to the driver
 */
void RCVWPlugin::SendError(string message)
{
	PLOG(logDEBUG) << "Sending Application Message: Error: " << message;
	SetStatus("Warning", "Error: " + message);
	SendApplicationMessage(EventCodeTypes::ERROR, Severity::Inform, message);
}

void RCVWPlugin::SendErrorCleared()
{
	PLOG(logDEBUG) << "Sending Application Message: Clear Error";
	SetStatus("Warning", "");
	SendApplicationMessage(EventCodeTypes::ERROR, Severity::Info);
}

bool RCVWPlugin::CheckForOnTrackWarning(double lat, double lon, double speed)
{
	if(speed <= _HRIWarningThresholdSpeed && InHRI(lat, lon))
	{
		if(!_inHRISevereWarning)
		{
			uint64_t curTime = GetMsTimeSinceEpoch();
			if(curTime - _lastWarning > 2000)
			{
				PLOG(logDEBUG) << "Send In HRI Severe Warning";

				_inHRISevereWarning = true;
				SendInHRISevereWarning();

				_lastWarning = GetMsTimeSinceEpoch();
			}
		}
		return true;
	}
	else
	{
		if(_inHRISevereWarning)
		{
			SetStatus("InHRI", "No");
			_inHRISevereWarning = false;
			PLOG(logDEBUG) << "Clear In HRI Severe Warning";
			SendInHRIWarningAlertCleared();
		}
		return false;
	}

//	if(_speed == 0.0 && InHRI())
//	{
//		if(!_inHRIWarning)
//		{
//			SetStatus("InHRI", "Yes");
//			_inHRIWarning = true;
//
//			_inHRISevereWarning = false;
//			_lastWarning = GetMsTimeSinceEpoch();
//
//			PLOG(logDEBUG) << "Send In HRI Warning";
//			SendInHRIWarning();
//		}
//		else if(!_inHRISevereWarning)
//		{
//			uint64_t curTime = GetMsTimeSinceEpoch();
//
//			if(curTime - _lastWarning > 2000)
//			{
//				PLOG(logDEBUG) << "Send In HRI Severe Warning";
//
//				_inHRISevereWarning = true;
//				SendInHRISevereWarning();
//
//				_lastWarning = GetMsTimeSinceEpoch();
//			}
//		}
//		return true;
//	}
//	else
//	{
//		if(_inHRIWarning)
//		{
//			SetStatus("InHRI", "No");
//			_inHRIWarning = false;
//			_inHRISevereWarning = false;
//			PLOG(logDEBUG) << "Clear In HRI Warning";
//			SendInHRIWarningAlertCleared();
//		}
//		return false;
//	}
}




void RCVWPlugin::AlertVehicle_2()
{
	bool logCalculations = false;

	double speed;
	double prevSpeed;
	uint64_t speedTime;
	uint64_t prevSpeedTime;
	double lat;
	double lon;
	double hdop;
	{
		std::lock_guard<mutex> lock(_locationLock);
		speed = _speed;
		prevSpeed = _prevSpeed;
		hdop = _horizontalDOP;
		speedTime = _speedTime;
		prevSpeedTime = _prevSpeedTime;
		lat = _lat;
		lon = _long;
	}
	//log data and calculations only if vehicle is not stopped (greater than threshold speed)
	if (_lastLoggedspeed > (_HRIWarningThresholdSpeed / 2.0) || speed > (_HRIWarningThresholdSpeed / 2.0))
	{
		PLOG(logDEBUG) << "Latitude: " << lat << ", Longitude: " << lon << ", Speed: " << speed << ", PrevSpeed: " << prevSpeed << ", HDOP: " << hdop << ", Preemption: " << _preemption;
		double lastLoggedspeed = speed;
		_lastLoggedspeed = lastLoggedspeed;
		logCalculations = true;
	}

	//calculate crossing distance, safe stopping distance, and set preemption
	//crossing distance = -1 if not in a lane

	double mu = _mu * _weatherFactor;
	double safetyStopDistance = GetStoppingDistance(speed, mu, 0.0) * _safetyOffset;
	double crossingDistance = GetDistanceToCrossing(lat, lon);
	double expectedStopDistance = 0;
	double acceleration;

	//calculate acceleration
	if(speed < prevSpeed)
	{
		//calculate expected stop distance due to deceleration
		acceleration = (speed - prevSpeed) / (((double)(speedTime - prevSpeedTime)) / 1000);
		expectedStopDistance = (-1 * (speed * speed)) / (2 * acceleration);
		PLOG(logDEBUG) << "SpeedTime: " << speedTime << ", PrevSpeedTime: " << prevSpeedTime <<  ", Acceleration: " << acceleration;
	}

	if (logCalculations)
	{
		PLOG(logDEBUG) << "VSE: " << _mu << ", WeatherFactor: " << _weatherFactor <<  ", ReactionTime: " << _reactionTime;
		PLOG(logDEBUG) << "CrossingDistance: " << crossingDistance << ", SafetyStopDistance: " << safetyStopDistance << ", ExpectedStopDistance: " << expectedStopDistance;
	}

	if (!_enteredArea)
	{
		//not active
		if (InHRI(lat, lon) || _preemption)
		{
			//vehicle is in HRI or in a lane with preemption
			//activate
			_enteredArea = true;
			SendApplicationActive();
		}
	}

	if (_enteredArea)
	{
		//active
		if (InHRI(lat, lon))
		{
			//in HRI
			//should be either active or HRI warning
			if(speed <= _HRIWarningThresholdSpeed)
			{
				//speed is below threshold, HRI warning conditions met
				if(!_inHRISevereWarning)
				{
					//HRI warning has not been sent
					//send HRI warning
					_inHRISevereWarning = true;
					SendInHRISevereWarning();
					//clear safe stop alert flag if set
					_warningActive = false;
				}
			}
			else
			{
				//speed is above threshold, HRI warning conditions NOT met
				if(_inHRISevereWarning)
				{
					//clear HRI warning if set
					SendInHRIWarningAlertCleared();
					_inHRISevereWarning = false;
				}
				if (_warningActive)
				{
					//clear safe stop alert if set
					SendAlertCleared();
					_warningActive = false;
				}
			}
		}
		else if (_preemption)
		{
			//in lane with preemption
			//should be either active or safe stop alert
			if(crossingDistance < safetyStopDistance || (expectedStopDistance > crossingDistance && _useDeceleration))
			{
				//safe stop alert conditions met
				if (!_warningActive)
				{
					//clear safe stop alert if set
					SendAlert();
					_warningActive = true;
					//clear HRI warning flag if set
					_inHRISevereWarning = false;
				}
			}
			else
			{
				//safe stop alert conditions NOT met
				if(_inHRISevereWarning)
				{
					//clear HRI warning if set
					SendInHRIWarningAlertCleared();
					_inHRISevereWarning = false;
				}
				if (_warningActive)
				{
					//clear safe stop alert if set
					SendAlertCleared();
					_warningActive = false;
				}
			}
		}
		else
		{
			//deactivate
			_enteredArea = false;
			_inHRISevereWarning = false;
			_warningActive = false;
			SendApplicationInactive();
		}
	}

}



/**
 * Function decides whether the vehicle needs to be alerted,
 * and if it does sends the proper alert to the vehicle. If
 * the vehicle is already decelerating the alerts are suppressed.
 * If the vehicle is still outside of the safe stopping zone
 * the alerts are also suppressed.
 */
void RCVWPlugin::AlertVehicle()
{
	bool logCalculations = false;

	double speed;
	double prevSpeed;
	uint64_t speedTime;
	uint64_t prevSpeedTime;
	double lat;
	double lon;
	{
		std::lock_guard<mutex> lock(_locationLock);
		speed = _speed;
		prevSpeed = _prevSpeed;
		speedTime = _speedTime;
		prevSpeedTime = _prevSpeedTime;
		lat = _lat;
		lon = _long;
	}
	if (_lastLoggedspeed != 0 || speed != 0)
	{
		PLOG(logDEBUG) << "Latitude: " << lat << ", Longitude: " << lon << ", Speed: " << speed << ", PrevSpeed: " << prevSpeed << ", Preemption: " << _preemption;
		double lastLoggedspeed = speed;
		_lastLoggedspeed = lastLoggedspeed;
		logCalculations = true;
	}

	//first check if stopped in HRI (HRI on track warning)

	if(CheckForOnTrackWarning(lat, lon, speed))
		return;


	// If the vehicle is decelerating, no warning should be issued.
	// If a warnign has been issued and the vehicle begins to decelerate
	// remove the warning.
//	if(IsDecelerating())
//	{
//		PLOG(logDEBUG) << "Vehicle Decelerating";
//		if(_warningActive)
//		{
//			SendAlertCleared();
//
//			_warningActive = false;
//		}
//		return;
//	}

	//calculate crossing distance and safe stopping distance
	double mu = _mu * _weatherFactor;
	double safetyStopDistance = GetStoppingDistance(speed, mu, 0.0) * _safetyOffset;
	double crossingDistance = GetDistanceToCrossing(lat, lon);
	double expectedStopDistance = 0;
	double acceleration;

	if(speed < prevSpeed)
	{
		//calculate expected stop distance due to deceleration
		acceleration = (speed - prevSpeed) / (((double)(speedTime - prevSpeedTime)) / 1000);
		expectedStopDistance = (-1 * (speed * speed)) / (2 * acceleration);
		PLOG(logDEBUG) << "SpeedTime: " << speedTime << ", PrevSpeedTime: " << prevSpeedTime <<  ", Acceleration: " << acceleration;
	}

	if (logCalculations)
	{
		PLOG(logDEBUG) << "VSE: " << _mu << ", WeatherFactor: " << _weatherFactor <<  ", ReactionTime: " << _reactionTime;
		PLOG(logDEBUG) << "CrossingDistance: " << crossingDistance << ", SafetyStopDistance: " << safetyStopDistance << ", ExpectedStopDistance: " << expectedStopDistance;
	}

	if(crossingDistance == -1 && !InHRI(lat, lon))
	{
		//The vehicle is not currently in an HRI or a valid lane

		if(_enteredArea)
		{
			//If the vehicle was previously in an HRI, send a System Inactive message to the UI
			SendApplicationInactive();

			_enteredArea = false;
		}
		return;
	}
	else
	{
		//The vehicle has entered the map in a valid lane with preemption.
		if(!_enteredArea && _preemption)
		{
			SendApplicationActive();

			_enteredArea = true;
		}
	}

	__enteredArea_mon.check();

	// The distance to the crossing is less than the safe stopping distance,
	// The preemption signal was found in the spat message i.e. the train may be present.
	// OR we are in the HRI but traveling faster than the threshold

	if((crossingDistance < safetyStopDistance || expectedStopDistance > crossingDistance) && _preemption)
	{
		PLOG(logDEBUG) << "Send Safe Stop Alert";

		SendAlert();

		_lastWarning = GetMsTimeSinceEpoch();
		_warningActive = true;
	}
	else
	{
		if (_warningActive)
		{
			PLOG(logDEBUG) << "Cancel Safe Stop Alert";
			if (_preemption)
			{
				SendAlertCleared();
			}
			else if (!InHRI(lat, lon))
			{
				SendApplicationInactive();
				_enteredArea = false;
			}
			_warningActive = false;
		}
		else if (_enteredArea && !_preemption && !InHRI(lat, lon))
		{
			SendApplicationInactive();
			_enteredArea = false;
		}
	}

	//_preemption = true;  I don't know why this is here.  I added an else in the GetDistanceToCrossing to set to true
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
	while(!_configSet)
	{
		usleep(1000);
	}


	PLOG(logINFO) << "Starting Plugin";

	while (_plugin->state != IvpPluginState_error)
	{
		if(!_mapReceived || !_spatReceived)
		{
			CheckForErrorCondition(_lat, _long);
			usleep(1000000);
			continue;
		}
		else
		{
			//No longer in Error Condition if we get here, cancel any errors and move on
			if(_errorActive)
			{
				PLOG(logDEBUG) << "SendErrorCleared";
				SendErrorCleared();
				_errorActive = false;
				if (!_enteredArea)
				{
					SendApplicationInactive();
				}
			}
		}

		uint64_t curTime = GetMsTimeSinceEpoch();
		bool messageCheck = false;

		if(curTime - _lastSpat > _messageExpiration)
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

		if(curTime - _lastLocation > _messageExpiration)
		{
			if(_locationReceived)
			{
				SendError("Location Data Invalid");
				SetStatus("Location Received", false);
				_locationReceived = false;
			}
			messageCheck = true;
		}




		//Vehicle is located near a known equipped HRI, but no MAP has been received



		//If any of the messages have expired skip the other calculations.
		if(messageCheck)
		{
			usleep(500000);
			continue;
		}

		AlertVehicle_2();
		usleep(100000);
	}


	return 0;
}


void RCVWPlugin::CheckForErrorCondition(double lat, double lon)
{
	bool isInRangeOfHRI = IsLocationInRangeOfEquippedHRI(lat, lon);

	bool isErrorCondition = false;
	string errorMessage = "";

	//TODO Check for error condition
	if(isInRangeOfHRI && !_mapReceived)
	{
		isErrorCondition = true;
		errorMessage = "MAP Data Not Received";
	}
	else if(isInRangeOfHRI && !_spatReceived)
	{
		isErrorCondition = true;
		errorMessage = "SPAT Data Not Received";
	}
	else if(!isInRangeOfHRI)
	{
		isErrorCondition = false;
	}

	if(isErrorCondition && !_errorActive)
	{
		SendError(errorMessage);
		_errorActive = true;
	}

	if(!isErrorCondition && _errorActive)
	{
		_errorActive = false;
		uint64_t curTime = GetMsTimeSinceEpoch();
		if(_inHRISevereWarning)
		{
			PLOG(logDEBUG) << "Clear Error And Send In HRI Severe Warning";
			SendInHRISevereWarning();
			_lastWarning = curTime;
		}
		else if(_warningActive)
		{
			PLOG(logDEBUG) << "Clear Error And Send Safe Stop Alert";
			SendAlert();
			_lastWarning = curTime;
		}
		else if(_enteredArea)
		{
			PLOG(logDEBUG) << "Clear Error And Send Application Active";
			SendApplicationActive();
		}
		else
		{
			PLOG(logDEBUG) << "Clear Error And Send Application Inactive";
			SendApplicationInactive();
		}
	}
}



} /* namespace RCVWPlugin */

int main(int argc, char *argv[])
{
	return run_plugin<RCVWPlugin::RCVWPlugin>("RCVWPlugin", argc, argv);
}
