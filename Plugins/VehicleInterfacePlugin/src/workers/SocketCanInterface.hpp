/*
 * SocketCanInterface.hpp
 *
 *  Created on: Apr 3, 2017
 *      @author: Gregory M. Baumgardner
 */

#ifndef WORKERS_SOCKETCANINTERFACE_HPP_
#define WORKERS_SOCKETCANINTERFACE_HPP_

#include "../VehicleConnection.h"
#include "../workers/CanData.hpp"

namespace VehicleInterfacePlugin {
namespace Can {

class SocketCanInterface: public tmx::utils::ThreadWorker {
public:
	static constexpr const char *TaskName = "socketCAN";

	SocketCanInterface(const tmx::message &config);
	virtual ~SocketCanInterface();

	/**
	 * Initialize the socket to the given CAN interface
	 */
	int InitializeSocketCan(const char* ifname);

	/**
	 * Main thread function that connects to the socket and processes messages.
	 */
	void DoWork();

private:
	int _socket;
	CanDataAdaptor _canData;
};

} /* End namespace Can */
} /* End namespace VehicleInterfacePlugin */

#endif /* WORKERS_SOCKETCANINTERFACE_HPP_ */
