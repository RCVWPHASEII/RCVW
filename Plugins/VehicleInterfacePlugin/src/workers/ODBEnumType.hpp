/*
 * ODBEnumType.hpp
 *
 *  Created on: Jun 5, 2019
 *      Author: gmb
 */

#ifndef WORKERS_ODBENUMTYPE_HPP_
#define WORKERS_ODBENUMTYPE_HPP_

#if __cplusplus >= 201103L
#include <tmx/utils/Enum.hpp>
#endif /* __cplusplus >= 201103L */

namespace VehicleInterfacePlugin {
namespace Can {

enum class ODB_TYPE {
	PASSIVE = 0,
	ODBII = 1,
	ODB = 2,
	ODB_ODBII = 3,
	ODBI = 4,
	ODB_NONCOMPLIANT = 5,
	EODB = 6,
	EODB_ODBII = 7,
	EODB_ODB = 8,
	EODB_ODB_ODBII = 9,
	JODB = 10,
	JODB_ODBII = 11,
	JODB_EODB = 12,
	JODB_EODB_ODBII = 13,
	RESERVED14 = 14,
	RESERVED15 = 15,
	RESERVED16 = 16,
	EMD = 17,
	EMDPLUS = 18,
	HD_ODBC = 19,
	HDB_ODB = 20,
	WWH_ODB = 21,
	RESERVED = 22
};


static constexpr const char *ODB_TYPE_PASSIVE_NAME = "Passive";
static constexpr const char *ODB_TYPE_ODBII_NAME = "ODB-II";
static constexpr const char *ODB_TYPE_ODB_NAME = "ODB";
static constexpr const char *ODB_TYPE_ODB_ODBII_NAME = "ODB/ODB-II";
static constexpr const char *ODB_TYPE_ODBI_NAME = "ODB-I";
static constexpr const char *ODB_TYPE_ODB_NONCOMPLIANT = "Not-ODB-compliant";
static constexpr const char *ODB_TYPE_EODB_NAME = "EODB";
static constexpr const char *ODB_TYPE_EODB_ODBII_NAME = "EODB/ODB-II";
static constexpr const char *ODB_TYPE_EODB_ODB_NAME = "EODB/ODB";
static constexpr const char *ODB_TYPE_EODB_ODB_ODBII = "EODB/ODB/ODB-II";
static constexpr const char *ODB_TYPE_JODB_NAME = "JODB";
static constexpr const char *ODB_TYPE_JODB_ODBII_NAME = "JODB/ODB-II";
static constexpr const char *ODB_TYPE_JODB_EODB_NAME = "JODB/EODB";
static constexpr const char *ODB_TYPE_JODB_EODB_ODBII_NAME = "JODB/EODB/ODB-II";
static constexpr const char *ODB_TYPE_RESERVED14_NAME = "Reserved14";
static constexpr const char *ODB_TYPE_RESERVED15_NAME = "Reserved15";
static constexpr const char *ODB_TYPE_RESERVED16_NAME = "Reserved16";
static constexpr const char *ODB_TYPE_EMD_NAME = "EMD";
static constexpr const char *ODB_TYPE_EMDPLUS_NAME = "EMD+";
static constexpr const char *ODB_TYPE_HD_ODBC_NAME = "HD-ODB-C";
static constexpr const char *ODB_TYPE_HD_ODB_NAME = "HD-ODB";
static constexpr const char *ODB_TYPE_WWH_ODB_NAME = "WWH-ODB";
static constexpr const char *ODB_TYPE_RESERVED = "Reserved";

static constexpr const char *ODB_TYPE_ALL_STRINGS[] = {
	ODB_TYPE_PASSIVE_NAME,
	ODB_TYPE_ODBII_NAME,
	ODB_TYPE_ODB_NAME,
	ODB_TYPE_ODB_ODBII_NAME,
	ODB_TYPE_ODBI_NAME,
	ODB_TYPE_ODB_NONCOMPLIANT,
	ODB_TYPE_EODB_NAME,
	ODB_TYPE_EODB_ODBII_NAME,
	ODB_TYPE_EODB_ODB_NAME,
	ODB_TYPE_EODB_ODB_ODBII,
	ODB_TYPE_EODB_ODB_NAME,
	ODB_TYPE_EODB_ODB_ODBII,
	ODB_TYPE_JODB_NAME,
	ODB_TYPE_JODB_ODBII_NAME,
	ODB_TYPE_JODB_EODB_NAME,
	ODB_TYPE_JODB_EODB_ODBII_NAME,
	ODB_TYPE_RESERVED14_NAME,
	ODB_TYPE_RESERVED15_NAME,
	ODB_TYPE_RESERVED16_NAME,
	ODB_TYPE_EMD_NAME,
	ODB_TYPE_EMDPLUS_NAME,
	ODB_TYPE_HD_ODBC_NAME,
	ODB_TYPE_HD_ODB_NAME,
	ODB_TYPE_WWH_ODB_NAME,
	ODB_TYPE_RESERVED
};

} /* End namespace Can */
} /* End namespace VehicleInterfacePlugin */

namespace tmx {

template <VehicleInterfacePlugin::Can::ODB_TYPE V> struct EnumName<VehicleInterfacePlugin::Can::ODB_TYPE, V> {
	static constexpr const char *name = VehicleInterfacePlugin::Can::ODB_TYPE_ALL_STRINGS[static_cast<size_t>(V)];
};
template <> struct EnumSequenceBuilder<VehicleInterfacePlugin::Can::ODB_TYPE> {
	typedef EnumSequence<VehicleInterfacePlugin::Can::ODB_TYPE,
			VehicleInterfacePlugin::Can::ODB_TYPE::PASSIVE,
			VehicleInterfacePlugin::Can::ODB_TYPE::ODBII,
			VehicleInterfacePlugin::Can::ODB_TYPE::ODB,
			VehicleInterfacePlugin::Can::ODB_TYPE::ODB_ODBII,
			VehicleInterfacePlugin::Can::ODB_TYPE::ODBI,
			VehicleInterfacePlugin::Can::ODB_TYPE::ODB_NONCOMPLIANT,
			VehicleInterfacePlugin::Can::ODB_TYPE::EODB,
			VehicleInterfacePlugin::Can::ODB_TYPE::EODB_ODBII,
			VehicleInterfacePlugin::Can::ODB_TYPE::EODB_ODB,
			VehicleInterfacePlugin::Can::ODB_TYPE::EODB_ODB_ODBII,
			VehicleInterfacePlugin::Can::ODB_TYPE::JODB,
			VehicleInterfacePlugin::Can::ODB_TYPE::JODB_ODBII,
			VehicleInterfacePlugin::Can::ODB_TYPE::JODB_EODB,
			VehicleInterfacePlugin::Can::ODB_TYPE::JODB_EODB_ODBII,
			VehicleInterfacePlugin::Can::ODB_TYPE::RESERVED14,
			VehicleInterfacePlugin::Can::ODB_TYPE::RESERVED15,
			VehicleInterfacePlugin::Can::ODB_TYPE::RESERVED16,
			VehicleInterfacePlugin::Can::ODB_TYPE::EMD,
			VehicleInterfacePlugin::Can::ODB_TYPE::EMDPLUS,
			VehicleInterfacePlugin::Can::ODB_TYPE::HD_ODBC,
			VehicleInterfacePlugin::Can::ODB_TYPE::HDB_ODB,
			VehicleInterfacePlugin::Can::ODB_TYPE::WWH_ODB,
			VehicleInterfacePlugin::Can::ODB_TYPE::RESERVED> type;
};

} /* End namespace tmx */


#endif /* WORKERS_ODBENUMTYPE_HPP_ */
