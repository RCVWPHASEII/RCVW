/*
 * ODBIIMessage.h
 *
 *  Created on: Apr 4, 2019
 *      Author: gmb
 */

#ifndef WORKERS_ODBIIMESSAGE_HPP_
#define WORKERS_ODBIIMESSAGE_HPP_

#define ECU_BROADCAST_ADDR 0x7DF

#include <tmx/messages/message.hpp>
#include <tmx/messages/byte_stream.hpp>
#include <tmx/TmxException.hpp>

namespace VehicleInterfacePlugin {
namespace Can {
namespace ODBII {

/**
 * Message for communication over ODB-II protocol.  This class supports the PID request and response.
 */
class ODBIIMessage: public tmx::message {
public:
	/**
	 * Construct a new ODB-II message
	 */
	ODBIIMessage() {}

	/**
	 * Copy an ODB-II message
	 */
	ODBIIMessage(const ODBIIMessage &copy): tmx::message(copy) { }
	virtual ~ODBIIMessage() {}

	static constexpr const char *MsgType = "ODB-II";
	static constexpr const char *ReqMsgSubType = "Query";
	static constexpr const char *RespMsgSubType = "Response";

	static constexpr size_t size = 8;

	// 11-bit PID type
	std_attribute(this->msg, uint16_t,          Address, 0, );

	// One byte length, must be 0-4
	std_attribute(this->msg, unsigned short,    Length,  0, );

	// One byte mode or service, must be 01-0A
	std_attribute(this->msg, unsigned short,    Service, 0, );

	// One or two byte PID
	std_attribute(this->msg, uint16_t,          PID,     0, );

	// Four byte data value
	std_attribute(this->msg, uint32_t,          Value,   0, );

	/**
	 * @return True if this message is a request type, as determined by the service number
	 */
	bool isRequest() {
		return this->get_Service() < 0x40;
	}

	/**
	 * @return True if this message is an SAE standard request or response, as determined by the service number
	 */
	bool isSAEStandard() {
		// SAE Standard requests and services have known service values
		if (this->isRequest())
			return this->get_Service() <= 0x0A;
		else
			return this->get_Service() <= 0x4A;
	}

	/**
	 * @return The PID as a stream of bytes
	 */
	tmx::byte_stream get_PID_bytes() {
		tmx::byte_stream bytes;
		this->to_bytes(this->get_PID(), bytes, this->getNumPIDBytes());
		return bytes;
	}

	/**
	 * Sets the PID from a stream of bytes
	 *
	 * @param bytes The bytes
	 */
	void set_PID_bytes(const tmx::byte_stream &bytes) {
		auto iter = bytes.end();
		if (bytes.size() > this->getNumPIDBytes())
			iter = bytes.begin() + this->getNumPIDBytes();

		this->set_PID(this->to_value<PID::data_type>(bytes.begin(), iter));
	}

	/**
	 * @return The data value as a stream of bytes
	 */
	tmx::byte_stream get_Value_bytes() {
		tmx::byte_stream bytes;
		if (!this->isRequest())
				this->to_bytes(this->get_Value(), bytes, this->get_Length() - 1 - this->getNumPIDBytes());
		return bytes;
	}

	/**
	 * Sets the data value from a stream of bytes
	 *
	 * @param bytes The bytes
	 */
	void set_Value_bytes(const tmx::byte_stream &bytes) {
		auto iter = bytes.end();
		if (bytes.size() > this->getNumDataBytes())
			iter = bytes.begin() + this->getNumDataBytes();

		this->set_Value(this->to_value<Value::data_type>(bytes.begin(), iter));
	}

	/**
	 * Decodes the stream of bytes to an ODB-II message
	 *
	 * @param bytes The bytes
	 */
	void decode(const tmx::byte_stream &bytes) {
		static constexpr const size_t addrSz = sizeof(Address::data_type);

		tmx::byte_stream copy(bytes);

		if (copy.size() < addrSz)
			return;

		this->set_Address(this->to_value<Address::data_type>(copy.begin(), copy.begin() + addrSz));
		copy.erase(copy.begin(), copy.begin() + addrSz);

		if (copy.size() < 1)
			return;

		this->set_Length(this->to_value<Length::data_type>(copy.begin(), copy.begin() + 1));
		copy.erase(copy.begin());

		// Check the remaining bytes matches the correct length.
		int bytesLeft = this->get_Length();
		if (bytesLeft <= 0 || bytesLeft > (int)(size - 1))
			return;

		// Truncate any unnecessary bytes
		copy.resize(bytesLeft);

		if (copy.size() < 1)
			return;

		this->set_Service(this->to_value<Service::data_type>(copy.begin(), copy.begin() + 1));
		copy.erase(copy.begin());

		this->set_PID_bytes(copy);
		copy.erase(copy.begin(), copy.begin() + this->getNumPIDBytes());

		if (copy.size() > 0)
			this->set_Value_bytes(copy);
	}

	/**
	 * Encodes the ODB-II message as a stream of bytes
	 *
	 * @return The encoded bytes
	 */
	tmx::byte_stream encode() {
		Validate();

		tmx::byte_stream bytes;
		this->to_bytes(this->get_Address(), bytes);
		this->to_bytes(this->get_Length(), bytes, 1);
		this->to_bytes(this->get_Service(), bytes, 1);

		tmx::byte_stream pidBytes = get_PID_bytes();
		bytes.insert(bytes.end(), pidBytes.begin(), pidBytes.end());

		tmx::byte_stream dataBytes = get_Value_bytes();
		bytes.insert(bytes.end(), dataBytes.begin(), dataBytes.end());

		for (size_t i = bytes.size() - sizeof(Address::data_type); i < size; i++)
			this->to_bytes(0x55, bytes, 1);

		return bytes;
	}

	/**
	 * Build a broadcast request message to the given service and PID
	 *
	 * @param PID The PID number to request
	 * @param service The service number, defaults to 01 = show current data
	 */
	void set_Request(PID::data_type pid, Service::data_type service = 1) {
		this->clear();

		this->set_Address(ECU_BROADCAST_ADDR);
		this->set_Service(service);
		this->set_PID(pid);

		this->set_Length((size_t)1 + this->getNumPIDBytes());
	}

	void set_Request(uint32_t svcPid) {
		if ((svcPid & 0xFFFF) == svcPid)
		{
			set_Request(svcPid & 0xFF, (svcPid >> 8) & 0xFF);
		}
		else
		{
			set_Request(svcPid & 0xFFFF, (svcPid >> 16) & 0xFF);
		}
	}

	/**
	 * Build a secondary request message to the source of the incoming response
	 *
	 * @param response The ODB-II response message to follow up to
	 */
	void set_Request(ODBIIMessage &response) {
		this->clear();

		this->set_Address(response.get_Address());
		this->set_Service(response.get_Service() - 0x40);
		this->set_PID(response.get_PID());

		this->set_Length((size_t)1 + this->getNumPIDBytes());
	}

private:
	void Validate() {
		if (	(this->get_Address() > 0 && (this->get_Address() & 0x8000) == 0x0) &&
				(this->get_Length() > 0 && this->get_Length() < size) &&
				(this->get_Service() > 0 && this->get_PID() >= 0)	)
			return;

		// Otherwise, throw an exception
		std::stringstream ss;
		ss << "Malformed ODB-II message: " << this->to_string();
		tmx::TmxException ex(ss.str());
		BOOST_THROW_EXCEPTION(ex);
	}

	size_t getNumPIDBytes() {
		return this->isSAEStandard() ? 1 : 2;
	}

	size_t getNumDataBytes() {
		int n = this->get_Length() - (size_t)1 - this->getNumPIDBytes();
		return (n > 0) ? (size_t)n : 0;
	}

	template <typename T>
	void to_bytes(T val, tmx::byte_stream &bytes, size_t numBytes = sizeof(T)) {
		if (numBytes == 1) {
			bytes.push_back(val & 0xFF);
		} else {
			for (size_t i = 0; i < numBytes; i++) {
				if (i > 0)
					val >>= 8;

				bytes.insert(bytes.begin(), val & 0xFF);
			}
		}
	}

	template <typename T, typename IterS, typename IterE>
	T to_value(IterS begin, IterE end) {
		T ret = 0;

		for (auto iter = begin; iter != end; iter++) {
			if (iter != begin)
				ret <<= 8;

			ret |= *iter;
		}

		return ret;
	}
};

} /* namespace ODBII */
} /* namespace Can */
} /* namespace VehicleInterfacePlugin */

#include <tmx/messages/routeable_message.hpp>

namespace tmx {

template <>
template <>
inline VehicleInterfacePlugin::Can::ODBII::ODBIIMessage
routeable_message::get_payload<VehicleInterfacePlugin::Can::ODBII::ODBIIMessage>()
{
	VehicleInterfacePlugin::Can::ODBII::ODBIIMessage ret;
	if (this->get_encoding().empty() || this->get_encoding() == IVP_ENCODING_BYTEARRAY)
		ret.decode(this->get_payload_bytes());
	else
		ret.set_contents(this->get_payload_str());
	return ret;
}

template <>
template <>
inline void routeable_message::initialize<VehicleInterfacePlugin::Can::ODBII::ODBIIMessage>(VehicleInterfacePlugin::Can::ODBII::ODBIIMessage &Message,
		std::string source, unsigned int sourceId, unsigned int flags) {
	this->initialize(Message.MsgType, Message.isRequest() ? Message.ReqMsgSubType : Message.RespMsgSubType, source, sourceId, flags);
	if (this->get_encoding().empty() || this->get_encoding() == IVP_ENCODING_BYTEARRAY)
		this->set_payload_bytes(Message.encode());
	else
		this->set_payload(Message.to_string());
}

} /* namespace tmx */

#endif /* WORKERS_ODBIIMESSAGE_HPP_ */
