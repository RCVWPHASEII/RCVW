//============================================================================
// Name        : VehicleInterfacePlugin.cpp
// Author      : Battelle Memorial Institute - Matthew Cline (cline@battelle.org)
// Version     :
// Copyright   : Battelle - 2016
// Description : Plugin gets information from the vehicle CAN bus using
//				 SocketCAN and sends out a Vehicle Message
//============================================================================

#include <iostream>
#include <mutex>

#include "PluginClient.h"
#include "VehicleFileAdaptor.hpp"
#include "VehicleConnection.h"

#include <boost/filesystem.hpp>
#include <VehicleBasicMessage.h>
#include <FrequencyThrottle.h>

using namespace std;
using namespace tmx;
using namespace tmx::utils;
using namespace tmx::messages;
using namespace vehicleparam;

namespace VehicleInterfacePlugin {

/**
 * <summary>
 * Vehicle Interface gets messages from the CAN bus and creates
 * a VehicleBasicMessage to send out to the core.
 * </summary>
 */

class VehicleInterfacePlugin: public PluginClient
{
public:
	VehicleInterfacePlugin(std::string name);
	virtual ~VehicleInterfacePlugin();
	int Main();

	void HandleVehicleBasicMessage(tmx::messages::VehicleBasicMessage &, tmx::routeable_message &);

protected:
	void UpdateConfigSettings();

	//Virtual method overrides
	void OnConfigChanged(const char* key, const char* value);
	void OnStateChange(IvpPluginState state);
private:
	std::atomic<uint64_t> _frequency { 0 };

	std::mutex _lock;
	VehicleBasicMessage _msg;

	void SendVehicleMessage();
};

static VehicleInterfacePlugin *_vehicleInterface = NULL;

/**
 * Default constructor. Good place to initialize atomic globals.
 */
VehicleInterfacePlugin::VehicleInterfacePlugin(std::string name):
		PluginClient(name)
{
	// Should not preclude external sources of the CAN data
	AddMessageFilter<VehicleBasicMessage>(this, &VehicleInterfacePlugin::HandleVehicleBasicMessage);
	SubscribeToMessages();

	_vehicleInterface = this;
}

/**
 * Default Deconstructor
 */
VehicleInterfacePlugin::~VehicleInterfacePlugin() { }

/**
 * Function updates all of the configureation params
 */
void VehicleInterfacePlugin::UpdateConfigSettings()
{
	GetConfigValue("Frequency", _frequency);

	VehicleDataAdaptor data;

	string tmp;
	if (GetConfigValue("Make", tmp))
		data.set_make(tmp);
	if (GetConfigValue("Model", tmp))
		data.set_model(tmp);
	if (GetConfigValue("Year", tmp))
		data.set_year(tmp);

	std::string drivers;
	std::string inputs;
	std::string cfgDir;

	GetConfigValue("Drivers", drivers);
	GetConfigValue("Inputs", inputs);
	GetConfigValue("ConfigDir", cfgDir);

	// Create a map of selected files
	map<string, bool> enabledFile;
	string delimiter = ",";

	for (string s: { drivers, inputs } )
	{
		size_t pos = 0;
		string token;
		while ((pos = s.find(delimiter)) != std::string::npos) {
			token = s.substr(0, pos);

			enabledFile[token] = true;
			s.erase(0, pos + delimiter.length());
		}

		// Always include the last token, which is the remaining string with no delimiters
		enabledFile[s] = true;
	}

	// Iterator over all the input JSON config files to come up with a single vehicle data file
	try
	{
		boost::filesystem::path dir(cfgDir);
		for (boost::filesystem::directory_iterator itr(dir); itr != boost::filesystem::directory_iterator(); itr++)
		{
			boost::filesystem::path jsonFile = itr->path();
			if (jsonFile.extension() != ".json" && jsonFile.extension() != ".vinfo")
				continue;

			PLOG(logDEBUG) << "Found vehicle file: " << jsonFile << endl;

			message_container_type c;
			c.load<JSON>(jsonFile.string());

			VehicleFileAdaptor veh;
			veh.set_contents(c);

			if (!enabledFile.count(veh.get_name()) || !enabledFile[veh.get_name()])
				continue;

			PLOG(logDEBUG1) << "Including vehicle file: " << veh << endl;
			data.add_file(veh);
		}

		PLOG(logDEBUG) << "Complete data file: " << data;

		VehicleConnection::GetConnection()->HandleVehicleData(data);
	}
	catch (exception &ex)
	{
		this->HandleException(ex, false);
	}
}

/**
 * Function logic is executed when the plugin has a config param that is changed.
 *
 * @param key the name of the config param that was changed
 * @param value the new value of the config param that was changed
 */
void VehicleInterfacePlugin::OnConfigChanged(const char* key, const char* value)
{
	PluginClient::OnConfigChanged(key, value);

	if (_plugin->state == IvpPluginState_registered)
		UpdateConfigSettings();
}

/**
 * Function logic is executed when the plugin state changes. Good place to
 * initialize statuses because they will be updated once the plugin is
 * registered and can interact with the database.
 *
 * @param state the new plugin state
 */
void VehicleInterfacePlugin::OnStateChange(IvpPluginState state)
{
	PluginClient::OnStateChange(state);
	if (state == IvpPluginState_registered)
		UpdateConfigSettings();
}

void VehicleInterfacePlugin::HandleVehicleBasicMessage(VehicleBasicMessage &vehMsg, routeable_message &rMsg) {
	PLOG(logDEBUG) << "Received updated to Vehicle Basic Message: " << vehMsg;

	// Need to lock the data to prevent race conditions or out of sequence updates
	// Suggest using an asynchronous call to this function to avoid delays while waiting for the lock
	lock_guard<mutex> lock(_lock);

	// Merge the trees
	message_tree_type tree = _msg.get_container().get_storage().get_tree();
	message_tree_type newTree = vehMsg.get_container().get_storage().get_tree();

	for (auto iter = newTree.begin(); iter != newTree.end(); iter++) {
		PLOG(logDEBUG2) << "Adding " << iter->first << "=" <<
				newTree.get<string>(iter->first) << " to vehicle message";

		tree.put(iter->first, newTree.get<string>(iter->first, "Unknown"));
	}

	_msg.set_contents(tree);
}

int VehicleInterfacePlugin::Main()
{
	PLOG(logINFO) << "Starting Plugin.";

	VehicleBasicMessage vehMsg;

	FrequencyThrottle<int> throttle;
	FrequencyThrottle<int, chrono::seconds> statusThrottle;
	statusThrottle.set_Frequency(chrono::seconds(2));

	int freq = 0;

	while(_plugin->state != IvpPluginState_error)
	{
		if (freq != _frequency) {
			freq = _frequency;
			throttle.set_Frequency(chrono::milliseconds(freq));
		}

		// Send the message only when it is time
		if (throttle.Monitor(0)) {
			{
				lock_guard<mutex> lock(_lock);
				vehMsg = _msg;
			}

			if (!vehMsg.is_empty())
				this->BroadcastMessage(vehMsg);
		}

		if (statusThrottle.Monitor(0)) {
			message_tree_type keys = vehMsg.get_container().get_storage().get_tree();
			keys.sort();
			for (auto iter = keys.begin(); iter != keys.end(); iter++)
				SetStatus(iter->first.c_str(),
					keys.get<string>(iter->first, "Unknown").c_str());
		}

		this_thread::sleep_for(chrono::milliseconds(freq / 5));
	}

	return 0;
}

void VehicleConnection::BroadcastMessage(VehicleBasicMessage &msg) {
	static routeable_message empty;

	if (_vehicleInterface)
		_vehicleInterface->HandleVehicleBasicMessage(msg, empty);
}

} /* namespace VehicleInterfacePlugin */

int main(int argc, char* argv[])
{
	return run_plugin<VehicleInterfacePlugin::VehicleInterfacePlugin>("VehicleInterfacePlugin", argc, argv);
}
