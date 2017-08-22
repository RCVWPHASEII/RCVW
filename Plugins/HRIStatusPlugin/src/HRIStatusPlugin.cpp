//============================================================================
// Name        : HRIStatusPlugin.cpp
// Author      : Battelle Memorial Institute - Matt Cline (cline@battelle.org)
// Version     :
// Copyright   : Battelle 2016
// Description : HRI Status Plugin - sends out a SPAT message every 10ms
//============================================================================

#include <iostream>
#include <atomic>
#include <thread>
#include <mutex>

#include <boost/algorithm/string.hpp>

#include "PluginClient.h"
#include <tmx/j2735_messages/SpatMessage.hpp>
#include <tmx/j2735_messages/BasicSafetyMessage.hpp>
#include <tmx/messages/message_document.hpp>
#include <FrequencyThrottle.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <aiousb/aiousb.h>
#ifdef __cplusplus
}
#endif

using namespace std;
using namespace tmx;
using namespace tmx::utils;
using namespace tmx::messages;

namespace HRIStatusPlugin
{

/**
 * <summary>
 * 	This plugin sends out the HRI Status in a SPAT Message
 * </summary>
 */

class HRIStatusPlugin: public PluginClient
{
public:
	HRIStatusPlugin(std::string);
	virtual ~HRIStatusPlugin();
	int Main();

protected:
	void UpdateConfigSettings();

	// virtual method overrides
	void OnConfigChanged(const char *key, const char *value);
	void OnStateChanged(IvpPluginState state);

	void HandleBSMMessage(BsmMessage &msg, routeable_message &routeableMsg);

private:
	//Config Values
	uint64_t _frequency = 100;
	uint64_t _monitorFreq = 100;
	uint64_t _intersectionID = 1929;
	bool _alwaysSend = true;
	std::string _intersectionName = "Rail Crossing";
	unsigned int _railPinNumber = 0;
	std::vector<std::pair<int, std::string>> _laneMapping;

	bool _isReceivingBsms = false;

	std::atomic<bool> _checkConfig;

	uint64_t _lastSendTime = 0;
	std::mutex _dataLock;

	std::atomic<bool> _trainComing;
	bool _previousState = false;

	bool _muteDsrcRadio = false;
	//ESP Functions
	bool EspSetup();
	bool GetPinState(int pinNumber);
	int GetBufferPosition(int pin);
	void MonitorRailSignal();

	//Spat Gereration Functions
	void UpdateMovementState(message_document &md);
	void UpdateIntersectionID(message_document &md);
	void UpdateIntersectionName(message_document &md);

	FrequencyThrottle<int> _throttle;
};

/**
 * Creates an instance of the HRI Status Plugin
 *
 * @param name Name to identify the plugin instance
 */
HRIStatusPlugin::HRIStatusPlugin(std::string name) : PluginClient(name)
{
	AddMessageFilter<BsmMessage>(this, &HRIStatusPlugin::HandleBSMMessage);
	SubscribeToMessages();

	EspSetup();
	_trainComing = true;
	_checkConfig = true;



	_throttle.set_Frequency(std::chrono::milliseconds(2000));
}

/**
 * Default Deconstructor
 */
HRIStatusPlugin::~HRIStatusPlugin()
{
	AIOUSB_Exit();

}

/**
 * Update the configuration parameters from the database
 */
void HRIStatusPlugin::UpdateConfigSettings()
{
	std::string lanes;
	std::vector<std::string> tokens;

	GetConfigValue<uint64_t>("Frequency", _frequency, &_dataLock);
	GetConfigValue<uint64_t>("Monitor Frequency", _monitorFreq, &_dataLock);
	GetConfigValue<unsigned int>("RailPinNumber", _railPinNumber, &_dataLock);
	GetConfigValue<std::string>("Lane Map", lanes);
	GetConfigValue<uint64_t>("Intersection ID", _intersectionID, &_dataLock);
	GetConfigValue<std::string>("Intersection Name", _intersectionName, &_dataLock);

	GetConfigValue<bool>("Always Send", _alwaysSend);

	boost::split(tokens, lanes, boost::is_any_of(",:"));

	std::lock_guard<mutex> lock(_dataLock);
	_laneMapping.clear();
	for(size_t i = 0; i < tokens.size(); i+=2)
	{
		_laneMapping.push_back(std::pair<int, std::string>(std::stoi(tokens[i]), tokens[i+1]));
	}

	_checkConfig = true;
}


void HRIStatusPlugin::HandleBSMMessage(BsmMessage &msg, routeable_message &routeableMsg)
{
	if(!_isReceivingBsms)
	{
		_isReceivingBsms = true;
		SetStatus("Receiving Bsms", _isReceivingBsms);
	}
	_throttle.Touch(0);
}

/**
 * Actions to perform when a the configuration parameters
 * change in the database
 *
 * @param key
 * @param value
 */
void HRIStatusPlugin::OnConfigChanged(const char *key, const char *value)
{
	PluginClient::OnConfigChanged(key, value);
	UpdateConfigSettings();
}

/**
 * Actions to perform when the state of the plugin changes
 *
 * @param state
 */
void HRIStatusPlugin::OnStateChanged(IvpPluginState state)
{
	PluginClient::OnStateChange(state);

	if (IvpPluginState::IvpPluginState_registered == state)
	{
		_isReceivingBsms = false;
		SetStatus("Receiving Bsms", _isReceivingBsms);
		_muteDsrcRadio = true;
		SetSystemConfigValue("MuteDsrcRadio", _muteDsrcRadio, false);
		SetStatus("MuteDsrcRadio", _muteDsrcRadio);

		_trainComing = false;

		this->SetStatus<std::string>("Train", "Crossing is clear");
		_previousState = _trainComing;

		UpdateConfigSettings();
	}
}

/**
 * Generates a time to use for timestamps.
 *
 * @return integer timestamp in ms
 */
uint64_t GetMsTimeSinceEpoch()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (uint64_t) ((double) (tv.tv_sec) * 1000
				+ (double) (tv.tv_usec) / 1000);
}

/**
 * Function to setup the ESP to read signals
 *
 * @return true if a device is found successfully false otherwise
 */
bool HRIStatusPlugin::EspSetup()
{
	unsigned long resultCode;
	resultCode = 30; // unknown error code during api initialization

	resultCode = AIOUSB_Init();

	if(resultCode == 0)
	{
		PLOG(logINFO) << "Connected to ESP box successfully";
		return true;
	}
	else
	{
		PLOG(logINFO) << "ESP box not initialized successfully";
		return false;
	}
}

/**
 * Function to get the state of a pin on ESP box
 *
 * @param pin pin number to query
 *
 * @return true if the pin is high, false otherwise.
 */
bool HRIStatusPlugin::GetPinState(int pinNumber)
{

	DIOBuf* readBuffer = NewDIOBuf(16);
	char * pinStatusString;
	unsigned long result = DIO_ReadIntoDIOBuf(diFirst, readBuffer);
	if(result == 0)
	{
		//PLOG(logINFO) << "Pins were read successfully. Preparing result...";
		pinStatusString = DIOBufToString(readBuffer);
		//cout << "The status of pin " << pinNumber << " is: " << (int)pinStatusString[GetBufferPosition(pinNumber)] << endl;
		if(pinStatusString[GetBufferPosition(pinNumber)] == '1')
			return true;
		else
			return false;
	}
	else
	{
		PLOG(logINFO) << "Error reading the pin";
	}
	return false;
}

int HRIStatusPlugin::GetBufferPosition(int pin)
{
	int position = 0;
	if (pin < 8)
	{
		position = 23 - pin;
	}
	else
	{
		position = 31 - pin + 8;
	}
	return position;
}

/**
 * Function to monitor the rail signal on a separate thread. If the pin
 * is voltage low the train is coming.
 */
void HRIStatusPlugin::MonitorRailSignal()
{
	while(true)
	{
		if(!GetPinState(_railPinNumber))
		{
			// sets a global variable. Should it send an Application Message?
			_trainComing = true; //Atomic wrapper does not need mutex locked.

			if(_trainComing != _previousState)
			{
				PLOG(logINFO) << "Train is present at the crossing.";
				this->SetStatus<std::string>("Train", "Train present at crossing.");
				_previousState = _trainComing;
			}

		}
		else
		{
			_trainComing = false;
			if(_trainComing != _previousState)
			{
				PLOG(logINFO) << "Crossing is clear.";
				this->SetStatus<std::string>("Train", "Crossing is clear");
				_previousState = _trainComing;
			}
		}


		usleep(_monitorFreq * 1000); // check 10 times per second
	}
}


void HRIStatusPlugin::UpdateMovementState(message_document &md)
{
	pugi::xpath_node intersectionState = md.select_node("//IntersectionState");

	intersectionState.node().remove_child("states");

	pugi :: xml_node states = intersectionState.node().append_child("states");

	std::lock_guard<mutex> lock(_dataLock);
	for(std::pair<int, std::string> newState : _laneMapping)
	{
		pugi::xml_node movementState = states.append_child("MovementState");
		movementState.append_child("signalGroup").append_child(pugi::node_pcdata).set_value(std::to_string(newState.first).c_str());
		pugi::xml_node movementEvent = movementState.append_child("state-time-speed").append_child("MovementEvent");
		pugi::xml_node eventState = movementEvent.append_child("eventState");

		if(_trainComing)
		{
			if(newState.second == "tracked")
			{
				eventState.append_child("protected-Movement-Allowed");
			}
			else
			{
				eventState.append_child("stop-And-Remain");
			}
		}
		else
		{
			if(newState.second == "tracked")
			{
				eventState.append_child("stop-And-Remain");
			}
			else
			{
				eventState.append_child("permissive-Movement-Allowed");
			}
		}

		pugi::xml_node timing = movementEvent.append_child("timing");
		timing.append_child("minEndTime").append_child(pugi::node_pcdata).set_value("32850");
		timing.append_child("maxEndTime").append_child(pugi::node_pcdata).set_value("32850");
	}

}

/**
 * Updates the intersection id in the spat message
 *
 * @param md reference to the message document that is constructing the spat message
 */
void HRIStatusPlugin::UpdateIntersectionID(message_document &md)
{
	pugi::xpath_node id = md.select_node("//IntersectionState/id");
	for(pugi::xml_node value : id.node().children())
	{
		id.node().remove_child(value);
	}
	id.node().append_child("id").append_child(pugi::node_pcdata).set_value(std::to_string(_intersectionID).c_str());
}


/**
 * Updates the intersection name in the spat message
 *
 * @param md reference to the message document that is constructing the spat message
 */
void HRIStatusPlugin::UpdateIntersectionName(message_document &md)
{
	pugi::xpath_node name = md.select_node("//IntersectionState/name");
	for(pugi::xml_node value : name.node().children())
	{
		name.node().remove_child(value);
	}
	name.node().append_child(pugi::node_pcdata).set_value(_intersectionName.c_str());
}

/**
 * Plugin Logic to run after initialization
 *
 * @return normal exit code
 */
int HRIStatusPlugin::Main()
{
	PLOG(logINFO) << "Starting Plugin";

	usleep(100000); // Allow Configurations to Update

	std::thread trainWatch(&HRIStatusPlugin::MonitorRailSignal, this);

	message_container_type c;
	c.get_storage().get_tree().clear();
	c.load<XML>("SampleIntersection.xml");

	SpatMessage spat;
	spat.set_contents(c);

	message_document md(spat);

	usleep(2000000); //Allow 2 seconds for initialization


	while (_plugin->state != IvpPluginState_error)
	{
		if(_checkConfig)
		{
			UpdateIntersectionID(md);
			UpdateIntersectionName(md);
			_checkConfig = false;
		}

		UpdateMovementState(md);

		md.flush();

		//md.print(cout);
		SpatEncodedMessage spatEnc;
		spatEnc.initialize(spat);
		spatEnc.set_flags(IvpMsgFlags_RouteDSRC);
		spatEnc.addDsrcMetadata(172, 0x8002);

		//cout << spatEnc;

		// Broadcast the message
		if (_throttle.Monitor(0))
		{
			PLOG(logDEBUG) << "BSMs Not Found";
			if(_isReceivingBsms)
			{
				_isReceivingBsms = false;
				SetStatus("Receiving Bsms", _isReceivingBsms);
			}
		}



		if(_alwaysSend)
		{
			if(_muteDsrcRadio)
			{
				_muteDsrcRadio = true;
				SetSystemConfigValue("MuteDsrcRadio", _muteDsrcRadio, false);
				SetStatus("MuteDsrcRadio", _muteDsrcRadio);
			}

		}
		else
		{
			if(_trainComing || _isReceivingBsms)
			{
				if(_muteDsrcRadio)
				{
					_muteDsrcRadio = false;
					SetSystemConfigValue("MuteDsrcRadio", _muteDsrcRadio, false);
					SetStatus("MuteDsrcRadio", _muteDsrcRadio);
				}
			}
			else
			{
				if(!_muteDsrcRadio)
				{
					_muteDsrcRadio = true;
					SetSystemConfigValue("MuteDsrcRadio", _muteDsrcRadio, false);
					SetStatus("MuteDsrcRadio", _muteDsrcRadio);
				}
			}
		}

		BroadcastMessage(static_cast<routeable_message &>(spatEnc));
		usleep(_frequency * 1000);
	}

	trainWatch.join();

	return 0;
}

} /* namespace HRIStatusPlugin */

int main(int argc, char *argv[])
{
	return run_plugin<HRIStatusPlugin::HRIStatusPlugin>("HRIStatusPlugin", argc, argv);
}
