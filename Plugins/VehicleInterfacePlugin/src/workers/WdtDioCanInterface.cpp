/*
 * POC351CanInterface.cpp
 *
 *  Created on: Apr 4, 2019
 *      Author: gmb
 */

#include "WdtDioCanInterface.h"
#include "CanData.hpp"

#include "ODBIIMessage.hpp"

#include <LockFreeThread.h>
#include <Measurement.h>
#include <thread>
#include <tmx/messages/auto_message.hpp>
#include <vector>

using namespace std;
using namespace tmx;
using namespace tmx::messages;
using namespace tmx::utils;

namespace VehicleInterfacePlugin {
namespace Can {

static VehicleConnection::TaskAllocatorImpl<WdtDioCanInterface> _wdtDioAlloc;

static constexpr const size_t ODBII_ADDR_LEN = sizeof(ODBII::ODBIIMessage::Address::data_type);

ODBII::ODBIIMessage ToODB2Message(const CAN_MSG *canMsg)
{
	ODBII::ODBIIMessage msg;

	if (canMsg)
	{
		byte_stream bytes(canMsg->len + ODBII_ADDR_LEN);
		memcpy(&bytes.data()[ODBII_ADDR_LEN], canMsg->data, canMsg->len);

		msg.decode(bytes);
		msg.set_Address(canMsg->id);
	}

	return msg;
}

CAN_MSG *FromODB2Message(ODBII::ODBIIMessage &msg, CAN_MSG *canMsg)
{
	if (canMsg)
	{
		canMsg->id = msg.get_Address();

		byte_stream bytes = msg.encode();
		canMsg->len = bytes.size() - ODBII_ADDR_LEN;
		memcpy(canMsg->data, &bytes.data()[ODBII_ADDR_LEN], canMsg->len);
	}

	return canMsg;
}

WdtDioCanInterface::WdtDioCanInterface(const message &config): _canData(config.get_container())
{
	Measurement<units::Time, units::Time::ms> msFreq = _canData.get_untyped("frequency", "500 ms");

	if (msFreq > 0)
		_throttle.set_Frequency(std::chrono::milliseconds(static_cast<int64_t>(msFreq.get_value())));

	myId = strtol(_canData.get_id().c_str(), NULL, 0);
}

WdtDioCanInterface::~WdtDioCanInterface()
{
}

void WdtDioCanInterface::doWork(std::shared_ptr<CAN_MSG> &canMsg)
{
	PLOG(logDEBUG4) << Id() << ": In doWork";

	if (!canMsg) return;

	VehicleBasicMessage vbm;

	if (_canData.get_type() == ODB_TYPE::ODBII)
	{
		ODBII::ODBIIMessage odb2 = ToODB2Message(canMsg.get());

		PLOG(logDEBUG) << Id() << ": Received ODBII response: " << odb2;

		vbm = _canData.decode_VBM(odb2.get_Value_bytes());
	}
	else
	{
		byte_stream bytes(canMsg->len);
		memcpy(bytes.data(), canMsg->data, canMsg->len);

		vbm = _canData.decode_VBM(bytes);
	}

	VehicleConnection::GetConnection()->BroadcastMessage(vbm);
}

void WdtDioCanInterface::idle()
{
	PLOG(logDEBUG4) << Id() << ": In idle";

	// Handle ODB-II requests
	if (_canData.get_enabled() && _canData.get_type() == ODB_TYPE::ODBII && _throttle.Monitor(0))
	{
		ODBII::ODBIIMessage odb2;
		odb2.set_Request(CanId());

		PLOG(logDEBUG2) << Id() << ": Sending ODBII request message: " << odb2;

		CAN_MSG *canMsg = (CAN_MSG *)malloc(sizeof(CAN_MSG));
		FromODB2Message(odb2, canMsg);

		this->push_out(std::shared_ptr<CAN_MSG> { canMsg, [](CAN_MSG *p){ free(p); } });
	}

	PLOG(logDEBUG4) << Id() << ": Waiting for next CAN message";

	unique_lock<mutex> lock(_lock);
	this->_cv.wait(lock, bind(&WdtDioCanInterface::inQueueSize, this));
}

bool WdtDioCanInterface::accept(const std::shared_ptr<CAN_MSG> &canMsg)
{
	PLOG(logDEBUG4) << this_thread::get_id() << ": In accept";

	if (!canMsg || !_canData.get_enabled())
		return false;

	// Only process this message if it was destined for this thread
	if (_canData.get_type() == ODB_TYPE::ODBII)
	{
		ODBII::ODBIIMessage odb2;
		odb2.set_Request(CanId());

		ODBII::ODBIIMessage msg = ToODB2Message(canMsg.get());

		PLOG(logDEBUG2) << "Comparing receieved ODB-II message: " << msg << " to request: " << odb2;

		// Can only compare a request message to a request message since service numbers are encoded different
		ODBII::ODBIIMessage odb2resp;
		odb2resp.set_Request(msg);

		return (odb2.get_Service() == odb2resp.get_Service() && odb2.get_PID() == odb2resp.get_PID());
	}
	else
	{
		return (CanId() == canMsg->id);
	}
}

void WdtDioCanInterface::notify() {
	this->_cv.notify_all();
}

WdtDioCanInterface *WdtDioFindThread(DWORD id)
{
	auto conn = VehicleConnection::GetConnection();
	if (!conn)
		return NULL;

	static ThreadGroupAssignment<uint8_t, uint8_t> _tga { *conn };
	int thread = _tga.assign((id >> 8) & 0xFF, id & 0xFF);
	if (thread < 0)
		return NULL;
	else
		return dynamic_cast<WdtDioCanInterface *>((*conn)[thread]);
}

void __stdcall WdtDioCanReceived(CAN_MSG *IpMsg, DWORD cbMsg)
{
	PLOG(logDEBUG4) << this_thread::get_id() << ": In WdtDioCanReceived.";

	if (!IpMsg) return;

	PLOG(logDEBUG3) << "Received " << IpMsg->id;

	WdtDioCanInterface *canThread = WdtDioFindThread(IpMsg->id);
	if (canThread) {
		CAN_MSG *copy = (CAN_MSG *) malloc(sizeof(CAN_MSG));
		if (copy) {
			*copy = *IpMsg;
			canThread->push(shared_ptr<CAN_MSG> { copy, [](CAN_MSG *p) { if (p) free(p); p = NULL; } });
			canThread->notify();
		}
	}
}

class WdtDioTxRxThread: public ThreadWorker {
public:
	CAN_SETUP setup;

	void DoWork()
	{
		PLOG(logINFO) << "WdtDioRxThread (" << this_thread::get_id() << ") is starting.";

		auto conn = VehicleConnection::GetConnection();

		// Pre-assign all the threads in the group so the IDs are automatically
		// matched when received
		for (size_t i = 0; conn && i < conn->size(); i++)
		{
			WdtDioCanInterface *canThread = dynamic_cast<WdtDioCanInterface *>((*conn)[i]);
			if (canThread)
				WdtDioFindThread(canThread->CanId());
		}

		// Open a connection to the CAN through the WDT_DIO driver
		if (!CAN_RegisterReceived(0, &WdtDioCanReceived))
		{
			TmxException regFailed("Failed to register callback");
			BOOST_THROW_EXCEPTION(regFailed);
		}


		if (!CAN_Setup(0, &setup, sizeof(setup)))
		{
			TmxException setupFailed("Failed to setup CAN device 0");
			BOOST_THROW_EXCEPTION(setupFailed);
		}

		if (!CAN_Start(0))
		{
			TmxException startFailed("Failed to start CAN device 0");
			BOOST_THROW_EXCEPTION(startFailed);
		}

		while (this->IsRunning())
		{
			// Await next CAN messages to send
			shared_ptr<CAN_MSG> canMsg;
			for (size_t i = 0; conn && i < conn->size(); i++)
			{
				WdtDioCanInterface *ifc = dynamic_cast<WdtDioCanInterface *>(conn->operator[](i));
				if (ifc && ifc->pop(canMsg))
				{
					if (canMsg && !CAN_Send(0, canMsg.get(), sizeof(CAN_MSG)))
					{
						byte_stream bytes(canMsg->len);
						memcpy(bytes.data(), canMsg->data, canMsg->len);

						PLOG(logERROR) << "Unable to send CAN message " << bytes << " to " << canMsg->id;
					}
				}
			}

			this_thread::sleep_for(chrono::milliseconds(5));
		}

		CAN_Stop(0);

		PLOG(logWARNING) << "WdtDioRxThread (" << this_thread::get_id() << ") has stopped.";
	}
};

DWORD WdtDioCanInterface::CanId() {
	return this->myId;
}

void WdtDioCanInterface::Start()
{
	// Start up the thread
	LockFreeThread::Start();
	if (this->_thread)
	{
		auto tId = this->_thread->get_id();
		int threadId = VehicleConnection::GetConnection()->this_thread(tId);

		PLOG(logINFO) << "Thread " << threadId << " (" << tId << ") for monitoring " <<
			(_canData.get_comment().empty() ? _canData.get_name() : _canData.get_comment()) << " has been started.";
	}
	else
	{
		PLOG(logERROR) << "Unable to start thread for monitoring " <<
				(_canData.get_comment().empty() ? _canData.get_name() : _canData.get_comment());
		LockFreeThread::Stop();
		return;
	}

	static std::atomic<bool> _startMgrThread { true };
	if (_startMgrThread.exchange(false))
	{
		auto wdtDioThread = new WdtDioTxRxThread();

		auto_message data = _canData;
		data.auto_attribute<CAN_SETUP>(wdtDioThread->setup.bitRate, "bitRate");
		data.auto_attribute<CAN_SETUP>(wdtDioThread->setup.recvConfig, "recvConfig");
		data.auto_attribute<CAN_SETUP>(wdtDioThread->setup.recvId,  "recvId");
		data.auto_attribute<CAN_SETUP>(wdtDioThread->setup.recvMask, "recvMask");

		PLOG(logDEBUG) << "Connecting to WDT_DIO CAN bus with " << data;

		mgrThread = wdtDioThread;
		mgrThread->Start();
	}
}

void WdtDioCanInterface::Stop() {
	LockFreeThread::Stop();

	if (mgrThread)
	{
		delete mgrThread;
		mgrThread = NULL;
	}
}

} /* namespace Can */
} /* namespace VehicleInterfacePlugin */
