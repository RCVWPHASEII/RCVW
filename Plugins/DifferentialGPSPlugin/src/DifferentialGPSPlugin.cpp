/*
 * DifferentialGPSPlugin.cpp
 *
 *  Created on: Jul 20, 2018
 *      @author: Gregory M. Baumgardner
 */

#define USE_STD_CHRONO

#include <atomic>
#include <FrequencyThrottle.h>
#include <fstream>
#include <LocationMessage.h>
#include <mutex>
#include <PluginClient.h>
#include <rtcm/RtcmMessage.h>
#include <tmx/j2735_messages/RtcmMessage.hpp>

namespace DifferentialGPSPlugin {

class DifferentialGPSPlugin: public tmx::utils::PluginClient {
public:
	DifferentialGPSPlugin(std::string name);
	~DifferentialGPSPlugin();

	void HandleLocation(tmx::messages::LocationMessage &msg, tmx::routeable_message &routeableMsg);
	void HandleRTCMMessage(tmx::messages::TmxRtcmMessage &msg, tmx::routeable_message &routeableMsg);
	void HandleRTCMCorrectionMessage(tmx::messages::RtcmMessage &msg, tmx::routeable_message &routeableMsg);
protected:
	void UpdateConfigSettings();
	void OnConfigChanged(const char *key, const char *value);
	void OnMessageReceived(IvpMessage *msg);
	void OnStateChange(IvpPluginState state);
private:
	std::atomic<bool> _cfgChanged { false };
	std::atomic<bool> _doWrite { false };
	std::atomic<uint64_t> _msgCount { 0 };
	std::atomic<uint64_t> _byteCount { 0 };

	std::string _device;
	std::string _rtcmVer;

	tmx::utils::FrequencyThrottle<int, std::chrono::seconds> _statusThrottle;

	void Handle(tmx::messages::TmxRtcmEncodedMessage &encodedMsg);
};

std::mutex _cfgLock;
std::mutex _writeLock;

using namespace std;
using namespace tmx;
using namespace tmx::messages;
using namespace tmx::utils;

DifferentialGPSPlugin::DifferentialGPSPlugin(string name): PluginClient(name) {
	this->AddMessageFilter<RtcmMessage>(this, &DifferentialGPSPlugin::HandleRTCMCorrectionMessage);
	this->AddMessageFilter<LocationMessage>(this, &DifferentialGPSPlugin::HandleLocation);

	this->SubscribeToMessages();

	_statusThrottle.set_Frequency(std::chrono::seconds(3));
}

DifferentialGPSPlugin::~DifferentialGPSPlugin() { }

void DifferentialGPSPlugin::UpdateConfigSettings() {
	lock_guard<mutex> lock(_cfgLock);

	this->GetConfigValue("Device", _device);
	this->GetConfigValue("RTCM Version", _rtcmVer);

	_cfgChanged = true;
}

void DifferentialGPSPlugin::OnConfigChanged(const char *key, const char *value) {
	PluginClient::OnConfigChanged(key, value);

	UpdateConfigSettings();
}

void DifferentialGPSPlugin::OnStateChange(IvpPluginState state) {
	PluginClient::OnStateChange(state);

	if (state == IvpPluginState_registered)
		UpdateConfigSettings();
}

void DifferentialGPSPlugin::Handle(TmxRtcmEncodedMessage &encodedMsg) {
	PLOG(logDEBUG1) << "Incoming message " << encodedMsg;

	for (auto iter = encodedMsg.begin(); iter != encodedMsg.end(); iter++) {
		if (*iter)
			this->HandleRTCMMessage(**iter, encodedMsg);
	}
}

void DifferentialGPSPlugin::OnMessageReceived(IvpMessage *msg) {
	if (msg && ::strcmp("RTCM", msg->type) == 0) {
		routeable_message rMsg(msg);
		rMsg.get_payload_str();

		TmxRtcmEncodedMessage encodedMsg(rMsg);
		this->Handle(encodedMsg);
		return;
	}

	PluginClient::OnMessageReceived(msg);
}

void DifferentialGPSPlugin::HandleRTCMCorrectionMessage(RtcmMessage &msg, routeable_message &routeableMsg) {
	PLOG(logDEBUG) << "Received " << msg;

	auto ptr = msg.get_j2735_data();
	if (ptr) {
		TmxRtcmEncodedMessage encodedMsg;
		byte_stream bytes;
#if SAEJ2735_SPEC < 63
		switch (ptr->rev) {
		case RTCM_Revision_rtcmRev2_x:
		case RTCM_Revision_rtcmRev2_0:
		case RTCM_Revision_rtcmRev2_1:
		case RTCM_Revision_rtcmRev2_3:
				encodedMsg.set_subtype(rtcm::RtcmVersionName(rtcm::RTCM_VERSION::SC10402_3));
		case RTCM_Revision_rtcmRev3_0:
		case RTCM_Revision_rtcmRev3_1:
				encodedMsg.set_subtype(rtcm::RtcmVersionName(rtcm::RTCM_VERSION::SC10403_3));
		default:
				encodedMsg.set_subtype(rtcm::RtcmVersionName(rtcm::RTCM_VERSION::UNKNOWN));
		}

		for (size_t i = 0; i < ptr->rtcmSets.list.count; i++) {
			for (size_t j = 0; j < ptr->rtcmSets.list.array[i]->payload.size; j++)
				bytes.push_back(ptr->rtcmSets.list.array[i]->payload.buf[j]);
		}
#else
		if (ptr->rev != RTCM_Revision_reserved)
			encodedMsg.set_subtype(rtcm::RtcmVersionName((rtcm::RTCM_VERSION)(ptr->rev)));

		for (size_t i = 0; i < ptr->msgs.list.count; i++) {
			for (size_t j = 0; j < ptr->msgs.list.array[i]->size; j++)
				bytes.push_back(ptr->msgs.list.array[i]->buf[j]);
		}
#endif
		encodedMsg.set_payload_bytes(bytes);
		this->Handle(encodedMsg);
	}
}

void DifferentialGPSPlugin::HandleRTCMMessage(TmxRtcmMessage &msg, routeable_message &routeableMsg) {
	static string ver;
	static string device;

	if (_cfgChanged) {
		lock_guard<mutex> lock(_cfgLock);
		ver = _rtcmVer;
		device = _device;
	}

	if (device.empty())
		return;

	if (!_doWrite)
		return;

	PLOG(logDEBUG) << "Received RTCM message " << msg;

	string err = "";
	if (msg.is_Valid() && msg.get_VersionName() == ver) {
		byte_stream bytes = msg.get_contents();

		PLOG(logDEBUG) << "Writing " << bytes.size() << " bytes to " << device;

		lock_guard<mutex> lock(_writeLock);
		ofstream out;
		out.open(device.c_str(), ios::binary | ios::out);
		out.write((const char *)bytes.data(), bytes.size());

		if (out.fail()) {
			err = "Failed to write RTCM " + ::to_string(msg.get_MessageType()) +
					" message";
		} else {
			_msgCount++;
			_byteCount += bytes.size();
		}

		out.close();
	}

	if (_statusThrottle.Monitor(0)) {
		SetStatus("RTCM Message Written", (uint64_t)_msgCount);
		SetStatus("RTCM Bytes Written", (uint64_t)_byteCount);
		SetStatus("Error", err);
	}
}

void DifferentialGPSPlugin::HandleLocation(LocationMessage &msg, routeable_message &routeableMsg) {
	_doWrite = (msg.get_FixQuality() >= location::FixTypes::ThreeD);
}

} /* End namespace DifferentialGPSPlugin */

int main(int argc, char **argv) {
	return tmx::utils::run_plugin<DifferentialGPSPlugin::DifferentialGPSPlugin>("DifferentialGPSPlugin", argc, argv);
}
