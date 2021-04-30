/*
 * VehicleDataAdapter.hpp
 *
 *  Created on: Apr 3, 2017
 *      @author: gmb
 */

#ifndef VEHICLEDATAADAPTOR_HPP_
#define VEHICLEDATAADAPTOR_HPP_

#include "VehicleDataDriverAdaptor.hpp"
#include "VehicleFileAdaptor.hpp"

#include <tmx/messages/message.hpp>

namespace VehicleInterfacePlugin {

class VehicleDataAdaptor: public tmx::message {
public:
	VehicleDataAdaptor(): tmx::message() {};
	VehicleDataAdaptor(const tmx::message_container_type &contents): tmx::message(contents) {}
	virtual ~VehicleDataAdaptor() {};

	// Make of the vehicle
	std_attribute(this->msg, std::string, make, "", );

	// Model of the vehicle
	std_attribute(this->msg, std::string, model, "", );

	// Year of the vehicle
	std_attribute(this->msg, std::string, year, "", );

	void add_file(VehicleFileAdaptor &vehicleFile) {
		auto myTree = this->as_tree();

		// Combine the key data to elements within this data adaptor
		auto vehTree = vehicleFile.get_container().get_storage().get_tree();
		std::string lastElem = vehTree.back().first;

		auto copy = vehTree.get_child(lastElem);

		boost::optional<tmx::message_tree_type &> ref = myTree.get().get_child_optional(lastElem);
		if (ref)
		{
			// The referenced array is already an element in my tree, so append the individual elements
			for (auto iter = copy.begin(); iter != copy.end(); iter++)
				if (iter->first.empty())
					ref.get().push_back(std::make_pair(iter->first, iter->second));
		}
		else
		{
			// Add the whole array to my tree
			myTree.get().put_child(lastElem, copy);
		}
	}

	std::vector<VehicleDataDriverAdaptor> get_drivers() {
		return this->get_array<VehicleDataDriverAdaptor>("drivers");
	}
};

} /* namespace VehicleInterfacePlugin */

#endif /* VEHICLEDATAADAPTOR_HPP_ */
