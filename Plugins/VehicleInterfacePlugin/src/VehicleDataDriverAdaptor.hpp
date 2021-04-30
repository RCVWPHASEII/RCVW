/*
 * VehicleDataInfoAdapter.hpp
 *
 *  Created on: Apr 3, 2017
 *      @author: gmb
 */

#ifndef VEHICLEDATADRIVERADAPTOR_HPP_
#define VEHICLEDATADRIVERADAPTOR_HPP_

#include <tmx/messages/message.hpp>

namespace VehicleInterfacePlugin {

class VehicleDataDriverAdaptor: public tmx::message {
public:
	VehicleDataDriverAdaptor(): tmx::message() {};
	VehicleDataDriverAdaptor(const tmx::message_container_type &contents): tmx::message(contents) {}
	virtual ~VehicleDataDriverAdaptor() {};

	// Name of the driver
	std_attribute(this->msg, std::string, name, "", );

	// Type of this driver, used to obtain configuration data
	std_attribute(this->msg, std::string, type, "", );

	// A comment for the driver
	std_attribute(this->msg, std::string, comment, "", );

	// True if this driver is enabled, false otherwise
	std_attribute(this->msg, bool, enabled, true, );

	static tmx::message_tree_type to_tree(VehicleDataDriverAdaptor message)
	{
		return tmx::message::to_tree(message);
	}

	static VehicleDataDriverAdaptor from_tree(const tmx::message_tree_type &tree)
	{
		return VehicleDataDriverAdaptor(tmx::message::from_tree(tree).get_container());
	}
};

} /* namespace VehicleInterfacePlugin */

#endif /* VEHICLEDATADRIVERADAPTOR_HPP_ */
