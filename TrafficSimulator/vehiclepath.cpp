#include "vehiclepath.h"

VehiclePath::VehiclePath(const std::string &name, double speedLimit) : name(name), speedLimit(speedLimit) {
}

VehiclePath::~VehiclePath() {
}

std::string VehiclePath::getName() const {
	return this->name;
}

void VehiclePath::setName(const std::string &name) {
	this->name = name;
}

double VehiclePath::getSpeedLimit() const {
    return this->speedLimit;
}

void VehiclePath::setSpeedLimit(double speedLimit) {
    this->speedLimit = speedLimit;
}
