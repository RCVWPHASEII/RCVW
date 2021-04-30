/*
 * CompositeVehicle.h
 *
 *  Created on: Sep 22, 2017
 *      Author: gmb
 */

#ifndef COMPOSITEVEHICLE_H_
#define COMPOSITEVEHICLE_H_

#include "../VehicleConnection.h"

#include <vector>

namespace VehicleInterfacePlugin {

class CompositeVehicle: public VehicleConnection {
public:
	CompositeVehicle();
	virtual ~CompositeVehicle();

	void Start();
	void Stop();
private:
	std::vector<VehicleConnection *> _vehicles;
};

} /* namespace VehicleInterfacePlugin */

#endif /* COMPOSITEVEHICLE_H_ */
