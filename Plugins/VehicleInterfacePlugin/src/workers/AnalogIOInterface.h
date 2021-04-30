/*
 * AnalogIOInterface.h
 *
 *  Created on: Sep 20, 2017
 *      Author: gmb
 */

#ifndef ANALOGIOINTERFACE_H_
#define ANALOGIOINTERFACE_H_

#include "../VehicleConnection.h"

#include <atomic>
#include <DigitalDevice.h>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <unistd.h>

#define ADC_MAX_PINS 8
#define ADC_ENCODING_BITS 12

namespace VehicleInterfacePlugin {

class AnalogIOElementDataAdaptor: public tmx::message {
public:
	AnalogIOElementDataAdaptor() { }
	virtual ~AnalogIOElementDataAdaptor() { }

	std_attribute(this->msg, std::string, name, "", )
	std_attribute(this->msg, std::string, comment, "", )
	std_attribute(this->msg, short, pin, -1, )

	std::vector<tmx::message> get_states() {
		return this->template get_array<tmx::message>("states");
	}

	tmx::message_tree_type &get_tree() {
		return this->as_tree().get();
	}

	static tmx::message_tree_type to_tree(AnalogIOElementDataAdaptor message)
	{
		return tmx::message::to_tree(message);
	}

	static AnalogIOElementDataAdaptor from_tree(const tmx::message_tree_type &tree)
	{
		AnalogIOElementDataAdaptor msg;
		msg.set_contents(tmx::message::from_tree(tree).get_container());
		return msg;
	}
};

template <typename Format = tmx::TMX_DEFAULT_MESSAGE_FORMAT>
class AnalogIODataAdaptor: public tmx::tmx_message<Format> {
public:
	AnalogIODataAdaptor() { }
	virtual ~AnalogIODataAdaptor() { }

	std_attribute(this->msg, std::string, name, "", )
	std_attribute(this->msg, std::string, device, "", )
	std_attribute(this->msg, short, pins, -1, )

	std::vector<AnalogIOElementDataAdaptor> get_elements() {
		return this->template get_array<AnalogIOElementDataAdaptor>("elements");
	}

	static tmx::message_tree_type to_tree(AnalogIODataAdaptor<Format> message)
	{
		return tmx::tmx_message<Format>::to_tree(message);
	}

	static AnalogIODataAdaptor<Format> from_tree(const tmx::message_tree_type &tree)
	{
		AnalogIODataAdaptor<Format> msg;
		msg.set_contents(tmx::tmx_message<Format>::from_tree(tree).get_container());
		return msg;
	}
};

class AnalogIOInterface: public VehicleConnection {
public:
	AnalogIOInterface();
	virtual ~AnalogIOInterface();

private:
	void Monitor();

	// SPI interface
	uint8_t mode;
	uint8_t bits;
	uint32_t speed;
	uint16_t delay;

	int InitSpi(std::string device);
	void Transfer(int fd, tmx::byte_stream &tx, tmx::byte_stream &rx, int length);
};

} /* namespace VehicleInterfacePlugin */

#endif /* ANALOGIOINTERFACE_H_ */
