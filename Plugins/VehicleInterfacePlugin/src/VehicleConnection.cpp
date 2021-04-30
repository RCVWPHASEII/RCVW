/*
 * VehicleConnection.cpp
 *
 *  Created on: Nov 26, 2018
 *      Author: gmb
 */

#include "VehicleConnection.h"

#include <map>
#include <mutex>
#include <thread>

#include <Clock.h>
#include <tmx/messages/routeable_message.hpp>

using namespace std;
using namespace tmx;
using namespace tmx::messages;
using namespace tmx::utils;

namespace VehicleInterfacePlugin {

std::atomic<bool> _dataUpdate { false };
std::mutex _dataLock;		// For data contents
std::mutex _threadLock;		// For thread group contents

map<string, VehicleConnection::TaskAllocator *> &taskAllocators()
{
	static map<string, VehicleConnection::TaskAllocator *> _map;
	return _map;
}

void VehicleConnection::TaskAllocator::Register() {
	taskAllocators()[this->GetName()] = this;
}

void AbortAll()
{
	FILE_LOG(logWARNING) << "Aborting all threads!";

	VehicleConnection::GetConnection()->Stop();
	abort();
}

VehicleConnection::VehicleConnection()
{
	//set_terminate(AbortAll);
}

VehicleConnection::~VehicleConnection() { }

VehicleConnection *VehicleConnection::GetConnection() {
	static VehicleConnection _conn;
	return &_conn;
}

void VehicleConnection::HandleVehicleData(const VehicleDataAdaptor &data)
{
	Stop();

	{
		std::lock_guard<mutex> lock(_dataLock);
		_data.set_contents(data.get_container());
		_dataUpdate = true;
	}

	Start();
}

VehicleDataAdaptor &VehicleConnection::GetVehicleData() const
{
	static VehicleDataAdaptor _cache;

	if (_dataUpdate) {
		std::lock_guard<mutex> lock(_dataLock);

		_cache.set_contents(_data.get_container());
		_dataUpdate = false;
	}

	return _cache;
}

void VehicleConnection::BroadcastMessage(const message_tree_type &tree)
{
	VehicleBasicMessage vbm;
	vbm.set_contents(tree);
	this->BroadcastMessage(vbm);
}

void VehicleConnection::Start()
{
	auto data = GetVehicleData();
	auto drivers = data.get_drivers();

	{
		// Need a lock to start the threads
		lock_guard<mutex> lock(_threadLock);

		for (auto iter = drivers.begin(); iter != drivers.end(); iter++)
		{
			if (!iter->get_enabled())
				continue;

			std::string task = iter->get_name();
			message_tree_type driverTree = iter->get_container().get_storage().get_tree();

			// Make sure this task type allocator exists
			if (!taskAllocators().count(task))
				continue;

			PLOG(logINFO) << "Initializing " << task << " " << iter->get_type() << " connection tasks.";

			// Break apart each request into a new thread of this task type
			for (auto request : data.get_array<message>(iter->get_type()))
			{
				PLOG(logDEBUG2) << request;
				message_tree_type reqTree = request.get_container().get_storage().get_tree();

				// Merge the global task configuration data with the specific request configuration
				// This is done so configuration parameters may be declared at a global level instead
				// of repeated for each request.  This will assume that the specific request
				// parameter overrides the global one.
				bool changed = false;

				// Loop through everything in the tree, looking for repeats
				for (auto dIter = driverTree.begin(); dIter != driverTree.end(); dIter++)
				{
					if (reqTree.get(dIter->first, "__N/A__") == "__N/A__")
					{
						reqTree.put_child(dIter->first, dIter->second);
						changed = true;
					}
				}

				if (changed)
					request.set_contents(reqTree);

				PLOG(logDEBUG) << "Starting new " << task << " " << iter->get_type() << " task for " << request;

				int i = this->push_back(taskAllocators()[task]->Allocate(request));

				auto *t = this->operator [](i);
				if (t)
					t->Start();
			}
		}
	}

	// Wait a few to make sure all the threads have successfully started and none have died
	this_thread::sleep_for(std::chrono::seconds(3));

	// Abort if the threads are not all running
	if (!this->IsRunning())
	{
		Stop();

		TmxException ex("Unable to initialize vehicle connection worker threads within the alloted time.  Aborting");

		BOOST_THROW_EXCEPTION(ex);
		return;
	}

	PLOG(logDEBUG) << "Started " << this->size() << " vehicle connection worker threads.";
}

void VehicleConnection::Stop()
{
	lock_guard<mutex> lock(_threadLock);

	PLOG(logDEBUG) << "Stopping " << this->size() << " vehicle connection worker threads";

	for (size_t i = 0; i < this->size(); i++)
	{
		auto *t = this->operator [](i);
		if (t)
		{
			delete t;
			t = NULL;
		}
	}

	this->Clear();
}

bool VehicleConnection::IsRunning()
{
	lock_guard<mutex> lock(_threadLock);

	return ThreadGroup::size() && ThreadGroup::IsRunning();
}

} /* End namespace VehicleInterfacePlugin */
