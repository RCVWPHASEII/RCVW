PROJECT ( VehicleInterfacePlugin VERSION 3.3.0 LANGUAGES CXX )

IF (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../tmx-build)
	SET (CMAKE_INCLUDE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../tmx-build ${CMAKE_INCLUDE_PATH})
ENDIF ()

SET (TMX_PLUGIN_NAME "VehicleInterface")

# Determine the WDT_DIO include directory
FIND_PATH (WDT_DIO_INCLUDE	wdt_dio.h
		   PATH_SUFFIXES neousys/wdt_dio/include)
		   
FILE (GLOB WDT_DIO_LIBRARY_PATH "${WDT_DIO_INCLUDE}/../*")
FIND_LIBRARY (WDT_DIO_LIBRARY wdt_dio ${WDT_DIO_LIBRARY_PATH})

BuildTmxPlugin ( )

TARGET_INCLUDE_DIRECTORIES (${PROJECT_NAME} PRIVATE ${WDT_DIO_INCLUDE})
TARGET_LINK_LIBRARIES (${PROJECT_NAME} tmxutils ${WDT_DIO_LIBRARY})

# Vehicle configuration files
INSTALL (FILES ClevelandBus.json 
		 DESTINATION ../../../usr/local/share/tmx/config COMPONENT cfg-clevelandbus)
INSTALL (FILES KiaSedona.json CANSim.json
		 DESTINATION ../../../usr/local/share/tmx/config COMPONENT cfg-vehicletesting)
INSTALL (FILES SAE_J1979.json
		 DESTINATION ../../../usr/local/share/tmx/config COMPONENT cfg-j1979)
INSTALL (FILES HondaCAN.json HondaGW.json
		 DESTINATION ../../../usr/local/share/tmx/config COMPONENT cfg-honda)
INSTALL (FILES VolvoCAN.json
		 DESTINATION ../../../usr/local/share/tmx/config COMPONENT cfg-dumptruck)

# Vehicle device drivers
INSTALL (FILES POC-351VTC.json
		 DESTINATION ../../../usr/local/share/tmx/config COMPONENT cfg-poc351vtc)