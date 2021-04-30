/*
 * SocketCanInterface.cpp
 *
 *  Created on: Apr 3, 2017
 *      @author: gmb
 */

#include "../workers/SocketCanInterface.hpp"

#include <mutex>

#include <PluginLog.h>
#include <tmx/TmxException.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can/raw.h>

using namespace std;
using namespace tmx;
using namespace tmx::messages;
using namespace tmx::utils;
using namespace VehicleInterfacePlugin;

namespace VehicleInterfacePlugin {
namespace Can {

// Create and register an allocator for the socket CAN interface
static VehicleConnection::TaskAllocatorImpl<SocketCanInterface> _socketCanAllocator;

SocketCanInterface::SocketCanInterface(const message &config): _socket(0), _canData(config.get_container())
{
}

SocketCanInterface::~SocketCanInterface()
{
	if (this->_socket > 0)
		::close(this->_socket);
}

int SocketCanInterface::InitializeSocketCan(const char* ifname)
{
	int receiveOwnMessages = 0;
	struct sockaddr_can addr;
	struct ifreq ifr;

	PLOG(logDEBUG) << this_thread::get_id() << ": Creating socketCAN interface to " << ifname;

	int sock = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);

	if (sock > 0)
	{
		PLOG(logDEBUG) << "Socket Initialized Successfully";
	}
	else
	{
		PLOG(logERROR) << "Error initializing the socket.";
		return sock;
	}

	memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
	strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
	if(ioctl(sock, SIOCGIFINDEX, &ifr) < 0)
	{
		PLOG(logERROR) << "Could not find interface index.";
		return -1;
	}
	else
	{
		PLOG(logDEBUG) << "Interface index found.";
	}

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	if(bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
	{
		PLOG(logERROR) << "Error binding the socket.";
		return -1;
	}
	else
	{
		PLOG(logDEBUG) << "Socket binded successfully.";
	}

	setsockopt(sock, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &receiveOwnMessages, sizeof(receiveOwnMessages));
	PLOG(logDEBUG) << "Socket configured to capture CAN Bus data.";

	return sock;
}

void SocketCanInterface::DoWork()
{
	int threadId = VehicleConnection::GetConnection()->this_thread();

	string bus = _canData.get_untyped("bus", "can0");

	while (!_canData.is_empty() && IsRunning())
	{
		struct can_frame frm;
		struct sockaddr_can addr;
		struct can_filter filter;

		socklen_t len;

		memset(frm.data, 0, 8);

		if (this->_socket <= 0)
		{
			// TODO: Obtain the request information for this thread

			filter.can_id = strtol(_canData.get_id().c_str(), NULL, 0);
			filter.can_mask = MaskByName(_canData.get_mask());

			this->_socket = InitializeSocketCan(bus.c_str());

			if (this->_socket <= 0)
			{
				PLOG(logDEBUG) << "Thread " << threadId << " (" << this_thread::get_id() << ") failed to start.";

				this->_socket = 0;
				break;
			}

			setsockopt(this->_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));

			PLOG(logINFO) << "Thread " << threadId << " (" << this_thread::get_id() << ") for monitoring " <<
					(_canData.get_comment().empty() ? _canData.get_name() : _canData.get_comment()) << " has been started.";
		}

		int ret = recvfrom(this->_socket, &frm, sizeof(struct can_frame), 0, (struct sockaddr*)&addr, &len);

		if(ret < 0)
		{
			PLOG(logERROR) << _canData.get_name() << ": Problem receiving from " << bus << " socket";

			// Try again after a few milliseconds
			this_thread::sleep_for(std::chrono::milliseconds(200));
			return;
		}

		PLOG(logDEBUG2) << this_thread::get_id() << ": Received " << frm.can_id << " frame of " << (int)frm.can_dlc << " bytes.";

		// Make sure this is the correct set of bytes
		/*if (frm.can_id != filter.can_id) {
			PLOG(logDEBUG4) << _canData.get_name() << "Dropping mismatch CAN ID " << frm.can_id;
			continue;
		}*/

		byte_stream frameBytes{frm.data[0],frm.data[1],frm.data[2],frm.data[3],frm.data[4],frm.data[5],frm.data[6],frm.data[7]};
		frameBytes.resize(frm.can_dlc);

		auto vbm = _canData.decode_VBM(frameBytes);
		VehicleConnection::GetConnection()->BroadcastMessage(vbm);
	}

	PLOG(logINFO) << "Thread " << threadId << " (" << this_thread::get_id() << ") is exiting.";

	this->_active = false;
}

} /* End namespace Can */
} /* End namespace VehicleInterfacePlugin */
