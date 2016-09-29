#include "roadobject.h"

#include "vehicleposition.h"

#include <math.h>

RoadObject::RoadObject(double vehicleWidth, double vehicleLength, VehiclePosition *position, double speed, double maxAcceleration, double comfortableDeceleration, double safetyDistance, bool isStationary, const std::string &objectAheadInfo) :
	vehicleWidth(fabs(vehicleWidth)), vehicleLength(fabs(vehicleLength)), currentPosition(position), speed(fabs(speed)), maxAcceleration(fabs(maxAcceleration)), comfortableDeceleration(fabs(comfortableDeceleration)), safetyDistance(fabs(safetyDistance)), stationary(isStationary), objectAheadInfo(objectAheadInfo) {
}

RoadObject::RoadObject(const RoadObject &o) : RoadObject(o.getVehicleWidth(), o.getVehicleLength(), o.getPosition() != nullptr ? o.getPosition()->clone() : nullptr, o.getSpeed(), o.getMaxAcceleration(), o.getComfortableDeceleration(), o.getSafetyDistance(), o.isStationary(), o.getObjectAheadInfo()) {
}

RoadObject::~RoadObject() {
    if (this->currentPosition != nullptr) {
        delete this->currentPosition;
	}
}

double RoadObject::getVehicleWidth() const {
	return this->vehicleWidth;
}

double RoadObject::getVehicleLength() const {
    return this->vehicleLength;
}

VehiclePosition *RoadObject::getPosition() const {
    return this->currentPosition;
}

double RoadObject::getSpeed() const {
    return this->speed;
}

double RoadObject::getMaxAcceleration() const {
    return this->maxAcceleration;
}

double RoadObject::getComfortableDeceleration() const {
    return this->comfortableDeceleration;
}

double RoadObject::getSafetyDistance() const {
	return this->safetyDistance;
}

bool RoadObject::isStationary() const {
	return this->stationary;
}

std::string RoadObject::getObjectAheadInfo() const {
	return this->objectAheadInfo;
}

void RoadObject::setVehicleWidth(double vehicleWidth) {
	this->vehicleWidth = vehicleWidth;
}

void RoadObject::setVehicleLength(double vehicleLength) {
    this->vehicleLength = fabs(vehicleLength);
}

void RoadObject::setPosition(VehiclePosition *currentPosition) {
	if (this->currentPosition != nullptr && this->currentPosition != currentPosition) {
        delete this->currentPosition;
    }
    this->currentPosition = currentPosition;
}

void RoadObject::setSpeed(double speed) {
    this->speed = fabs(speed);
}

void RoadObject::setMaxAcceleration(double maxAcceleration) {
    this->maxAcceleration = fabs(maxAcceleration);
}

void RoadObject::setComfortableDeceleration(double comfortableDeceleration) {
    this->comfortableDeceleration = fabs(comfortableDeceleration);
}

void RoadObject::setSafetyDistance(double safetyDistance) {
	this->safetyDistance = fabs(safetyDistance);
}

void RoadObject::setStationary(bool isStationary) {
	this->stationary = isStationary;
}

void RoadObject::setObjectAheadInfo(const std::string &objectAheadInfo) {
	this->objectAheadInfo = objectAheadInfo;
}
