/*
 * VehicleConnection.h
 *
 *  Created on: Nov 26, 2018
 *      @author: Gregory M. Baumgardner
 */

#ifndef VEHICLECONNECTION_H_
#define VEHICLECONNECTION_H_

#include "VehicleDataAdaptor.hpp"

#include <map>
#include <PluginLog.h>
#include <ThreadGroup.h>
#include <VehicleBasicMessage.h>

namespace VehicleInterfacePlugin {

/**
 * Generic class to handle construction of the Vehicle Basic message
 * from the disjoint worker threads.  This class uses a TmxMessageManager
 * as the thread structure.
 */
class VehicleConnection: public tmx::utils::ThreadGroup {
public:
	virtual ~VehicleConnection();

	void HandleVehicleData(const VehicleDataAdaptor &data);
	VehicleDataAdaptor &GetVehicleData() const;

	void Start();
	void Stop();
	bool IsRunning();

	void BroadcastMessage(tmx::messages::VehicleBasicMessage &msg);
	void BroadcastMessage(const tmx::message_tree_type &tree);

	static VehicleConnection *GetConnection();

	class TaskAllocator {
	public:
		virtual ~TaskAllocator() { }
		virtual tmx::utils::ThreadWorker *Allocate(tmx::message config) = 0;
		virtual std::string GetName() = 0;

		void Register();
	};

	template <class Task>
	class TaskAllocatorImpl: public TaskAllocator {
	public:
		TaskAllocatorImpl() {
			this->Register();
		}

		Task *Allocate(tmx::message config) { return new Task(config); }
		std::string GetName() { return Task::TaskName; }
	};

private:
	VehicleConnection();

	VehicleDataAdaptor _data;
};

} /* End namespace VehicleInterfacePlugin */

#endif /* VEHICLECONNECTION_H_ */
