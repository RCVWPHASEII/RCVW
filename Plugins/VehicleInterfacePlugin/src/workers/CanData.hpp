/*
 * SocketCanData.hpp
 *
 *  Created on: Nov 26, 2018
 *      @author: Gregory M. Baumgardner
 */

#ifndef CONNECTIONS_SOCKETCANDATA_HPP_
#define CONNECTIONS_SOCKETCANDATA_HPP_

#include <linux/can.h>

#include "ODBEnumType.hpp"

#include <algorithm>
#include <tmx/messages/byte_stream.hpp>
#include <tmx/messages/message.hpp>
#include <Measurement.h>
#include <VehicleBasicMessage.h>

namespace VehicleInterfacePlugin {
namespace Can {

static inline canid_t MaskByName(std::string name) {
	if (name == "EFF")
		return CAN_EFF_MASK;
	else if (name == "SFF")
		return CAN_SFF_MASK;
	else
		return 0;
}

class CanElementDataAdaptor: public tmx::message {
public:
	CanElementDataAdaptor(): tmx::message() {}
	CanElementDataAdaptor(const tmx::message_container_type &contents): tmx::message(contents) {}
	virtual ~CanElementDataAdaptor() {}

	std_attribute(this->msg, std::string, name, "Unknown", );
	std_attribute(this->msg, std::string, comment, "", );
	std_attribute(this->msg, bool, enabled, true, );
	std_attribute(this->msg, size_t, byte, 0, );
	std_attribute(this->msg, int16_t, len, 1, );
	std_attribute(this->msg, std::string, datatype, "enum", );
	std_attribute(this->msg, std::string, unit, "", );
	std_attribute(this->msg, std::string, mask, "", );
	std_attribute(this->msg, bool, signedval, false, );
	std_attribute(this->msg, uint16_t, repeat, 0, );
	std_attribute(this->msg, double, scale, 1.0, );
	std_attribute(this->msg, double, adjust, 0.0, );

	std::string evaluate(const tmx::byte_stream &bytes);

	static tmx::message_tree_type to_tree(CanElementDataAdaptor message)
	{
		return tmx::message::to_tree(message);
	}

	static CanElementDataAdaptor from_tree(const tmx::message_tree_type &tree)
	{
		return CanElementDataAdaptor(tmx::message::from_tree(tree).get_container());
	}
};

class CanDataAdaptor: public tmx::message {
public:
	typedef tmx::messages::Measurement<tmx::messages::units::Time> timeMeas;

	CanDataAdaptor(): tmx::message() {}
	CanDataAdaptor(const tmx::message_container_type &contents): tmx::message(contents) {}
	virtual ~CanDataAdaptor() {}

	std_attribute(this->msg, std::string, name, "Unknown", );
	std_attribute(this->msg, std::string, comment, "", );
	std_attribute(this->msg, bool, enabled, true, );
	std_attribute(this->msg, tmx::Enum<ODB_TYPE>, type, 0, );
	std_attribute(this->msg, std::string, id, "", );
	std_attribute(this->msg, std::string, mask, "EFF", if (MaskByName(value)));

public:
	tmx::messages::VehicleBasicMessage decode_VBM(const tmx::byte_stream &bytes);

	std::vector<CanElementDataAdaptor > get_elements() {
		return this->template get_array<CanElementDataAdaptor>("elements");
	}

	static tmx::message_tree_type to_tree(CanDataAdaptor message)
	{
		return tmx::message::to_tree(message);
	}

	static CanDataAdaptor from_tree(const tmx::message_tree_type &tree)
	{
		return CanDataAdaptor(tmx::message::from_tree(tree).get_container());
	}
};

} /* End namespace Can */
} /* End namespace VehicleInterfacePlugin */

#endif /* CONNECTIONS_SOCKETCANDATA_HPP_ */
