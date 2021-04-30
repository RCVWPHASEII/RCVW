/*
 * VehicleFileAdapter.hpp
 *
 *  Created on: Apr 3, 2017
 *      @author: gmb
 */

#ifndef VEHICLEFILEADAPTOR_HPP_
#define VEHICLEFILEADAPTOR_HPP_

#include <tmx/messages/message.hpp>

namespace VehicleInterfacePlugin {

class VehicleFileAdaptor: public tmx::message {
public:
	typedef VehicleFileAdaptor type;
	VehicleFileAdaptor(): tmx::message() {};
	virtual ~VehicleFileAdaptor() {};

	// Name of the vehicle file
	std_attribute(this->msg, std::string, name, "", );

	// Revision of the vehicle file
	std_attribute(this->msg, std::string, revision, "", );

	// Comment for the vehicle file
	std_attribute(this->msg, std::string, comment, "", );

};

} /* namespace VehicleInterfacePlugin */

#endif /* VEHICLEFILEADAPTOR_HPP_ */
