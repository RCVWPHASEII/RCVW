/*
 * SocketCanData.cpp
 *
 *  Created on: Dec 6, 2018
 *      @author: Gregory M. Baumgardner
 */

#include "CanData.hpp"

#include "ODBIIMessage.hpp"

#include <algorithm>
#include <bitset>
#include <cctype>
#include <cmath>
#include <ostream>
#include <PluginLog.h>
#include <tuple>
#include <type_traits>
#include <vector>

#define ENUM_MISSING "*"

using namespace std;
using namespace tmx;
using namespace tmx::messages;
using namespace tmx::utils;

namespace VehicleInterfacePlugin {
namespace Can {

struct Enum {};

typedef std::tuple<void, Enum, int, double> _dataTypes;

template <typename T>
struct SocketCanDataEvaluator {
	static T evaluate(CanElementDataAdaptor &config, const byte_stream &data) {
		uint64_t _data = 0;
		for (size_t i = 0; i < data.size(); i++) {
			if (i > 0) _data <<= 8;
			_data |= data[i];
		}

		uint64_t mask = -1;
		if (!config.get_mask().empty())
			mask = strtoul(config.get_mask().c_str(), NULL, 0);

		_data &= mask;

		T ret = static_cast<T>(_data);

		if (config.get_signedval()) {
			// If this is a signed number, may need to convert the sign
			size_t numBits = std::bitset<64>(mask).count();
			int64_t maxVal = pow(2, numBits - 1);

			if (_data >= (uint64_t)maxVal) {
				// This is a negative number.  Take two's complement
				_data = (std::bitset<64>(_data).flip().to_ullong() & mask) + 1;
				ret = static_cast<T>(-1) * _data;
			}
		}

		return ret * config.get_scale() + config.get_adjust();
	};
};

template <>
struct SocketCanDataEvaluator<Enum> {
	static std::string evaluate(CanElementDataAdaptor &config, const byte_stream &data) {
		unsigned val = SocketCanDataEvaluator<unsigned>::evaluate(config, data);

		static message_tree_type _cachedConfig;

		//static string _lastVal = "";
		//static uint8_t _count = 0;

		boost::optional<message_tree_type &> thisConfig = _cachedConfig.get_child_optional(config.get_name());
		if (!thisConfig || thisConfig.get().get("_VERSION_", -100) < config.get_container().get_storage_version()) {
			// Lazy initialization.  Dump in the configuration data for this enumeration
			message_tree_type t;

			message_container_type c = config.get_container();
			boost::optional<message_tree_type &> states = c.get_storage().get_tree().get_child_optional("states");
			if (states) {
				for (auto iter = states.get().begin(); iter != states.get().end(); iter++) {
					const string& s = iter->first;

					string val = states.get().get<string>(s, "");
					if (!val.empty()) {
						// Convert all numeric types to decimal equivalent
						if (::isdigit(val[0]))
							t.put(::to_string(strtol(val.c_str(), NULL, 0)), s);
						else
							t.put(val, s);
					}
				}
			}

			t.put("_VERSION_", config.get_container().get_storage_version());
			_cachedConfig.put_child(config.get_name(), t);
			thisConfig = _cachedConfig.get_child_optional(config.get_name());
		}

		string valStr = thisConfig.get().get<string>(::to_string(val), "");

		if (valStr.empty())
			valStr = thisConfig.get().get<string>(ENUM_MISSING, "");

		return valStr;
	}
};

template <size_t N = std::tuple_size<_dataTypes>::value - 1, typename T = typename tuple_element<N, _dataTypes>::type>
string evalDispatch(CanElementDataAdaptor &config, const tmx::byte_stream &bytes) {
	static string typeName = battelle::attributes::type_name<T>();

	// Make sure the first character is lowercase
	typeName[0] = ::tolower(typeName[0]);

	// Only use the first word also
	size_t n = typeName.find_first_of(' ');
	if (n != string::npos)
		typeName.resize(n);

	if (typeName == config.get_datatype()) {
		ostringstream ss;
		ss << SocketCanDataEvaluator<T>::evaluate(config, bytes);
		if (!config.get_unit().empty())
			ss << (::isalnum(config.get_unit()[0]) ? " " : "") << config.get_unit();
		return ss.str();
	} else {
		return evalDispatch<N-1>(config, bytes);
	}
}

template <>
string evalDispatch<0, void>(CanElementDataAdaptor &config, const tmx::byte_stream &bytes) {
	return "";
}

string CanElementDataAdaptor::evaluate(const byte_stream &bytes) {
	// Take the slice of bytes for this data
	// A negative length means reverse the order of the bytes
	auto len = abs((long)this->get_len());
	tmx::byte_stream dataBytes(len);
	if (this->get_len() < 0)
		reverse_copy(bytes.data() + this->get_byte(),
				bytes.data() + this->get_byte() + len, dataBytes.begin());
	else
		copy(bytes.data() + this->get_byte(),
				bytes.data() + this->get_byte() + len, dataBytes.begin());

	return evalDispatch(*this, dataBytes);
}

VehicleBasicMessage CanDataAdaptor::decode_VBM(const byte_stream &bytes)
{
	VehicleBasicMessage vbm;

	message_tree_type vbmTree;
	auto elements = this->get_elements();

	FILE_LOG(logDEBUG1) << this->get_name() << ": Found " << this->get_id() << ": " << bytes;

	for (CanElementDataAdaptor element: elements) {
		if (element.get_enabled())
		{
			string val = element.evaluate(bytes);
			if (!val.empty())
				vbmTree.put(element.get_name(), val);
		}
	}

	vbm.set_contents(vbmTree);

	return vbm;
}

} /* End namespace Can */
} /* End namespace VehicleInterfacePlugin */

