/*
 * POC351CanInterface.h
 *
 *  Created on: Apr 4, 2019
 *      Author: gmb
 */

#ifndef WORKERS_WDTDIOCANINTERFACE_H_
#define WORKERS_WDTDIOCANINTERFACE_H_

#include "../VehicleConnection.h"

#define USE_STD_CHRONO
#include <chrono>
#include <condition_variable>
#include <FrequencyThrottle.h>
#include <LockFreeThread.h>
#include <memory>
#include <mutex>
#include <tmx/messages/message.hpp>
#include <wdt_dio.h>
#include "../workers/CanData.hpp"

namespace VehicleInterfacePlugin {
namespace Can {

class WdtDioCanInterface: public tmx::utils::LockFreeThread<std::shared_ptr<CAN_MSG> > {
public:
	static constexpr const char *TaskName = "WDT_DIO";

	WdtDioCanInterface(const tmx::message &);
	virtual ~WdtDioCanInterface();

	bool accept(const std::shared_ptr<CAN_MSG> &canMsg);
	void doWork(std::shared_ptr<CAN_MSG> &canMsg);
	void idle();
	void notify();

	void Start();
	void Stop();

	DWORD CanId();
private:
	CanDataAdaptor _canData;
	DWORD myId { 0 };

	tmx::utils::ThreadWorker *mgrThread = NULL;
	tmx::utils::FrequencyThrottle<int> _throttle;

	std::condition_variable _cv;
	std::mutex _lock;
};

} /* namespace Can */
} /* namespace VehicleInterfacePlugin */

#endif /* WORKERS_WDTDIOCANINTERFACE_H_ */
