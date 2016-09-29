#ifndef VEHICLEPATH_H
#define VEHICLEPATH_H

#include <string>

class TrafficLight;

class VehiclePath {
	std::string name;
	double speedLimit;

public:
	VehiclePath(const std::string &name, double speedLimit);
    virtual ~VehiclePath();

	std::string getName() const;
	void setName(const std::string &name);
	virtual double getSpeedLimit() const;
	virtual void setSpeedLimit(double speedLimit);
};

#endif // VEHICLEPATH_H
