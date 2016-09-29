#ifndef ROADOBJECT_H
#define ROADOBJECT_H

#include <string>

class VehiclePosition;

class RoadObject {
	double vehicleWidth;
	double vehicleLength;
	VehiclePosition *currentPosition;
	float speed;
	double maxAcceleration;
	double comfortableDeceleration;
	double safetyDistance;
	bool stationary;
	std::string objectAheadInfo;

public:
	RoadObject(double vehicleWidth, double vehicleLength, VehiclePosition* position, double speed, double maxAcceleration, double comfortableDeceleration, double safetyDistance, bool isStationary, const std::string &objectAheadInfo = "");
    RoadObject(const RoadObject &o);
    virtual ~RoadObject();


	virtual double getVehicleWidth() const;
    virtual double getVehicleLength() const;
    virtual VehiclePosition* getPosition() const;
    virtual double getSpeed() const;
    virtual double getMaxAcceleration() const;
    virtual double getComfortableDeceleration() const;
    virtual double getSafetyDistance() const;
	virtual bool isStationary() const;
	virtual std::string getObjectAheadInfo() const;

protected:
	virtual void setVehicleWidth(double vehicleWidth);
    virtual void setVehicleLength(double vehicleLength);
    virtual void setPosition(VehiclePosition *currentPosition);
    virtual void setSpeed(double speed);
    virtual void setMaxAcceleration(double maxAcceleration);
    virtual void setComfortableDeceleration(double comfortableDeceleration);
	virtual void setSafetyDistance(double safetyDistance);
	virtual void setStationary(bool isStationary);
	virtual void setObjectAheadInfo(const std::string &objectAheadInfo);
};

#endif // ROADOBJECT_H
