#ifndef VEHICLEROUTE_H
#define VEHICLEROUTE_H

#include "direction.h"

#include <QMap>
#include <QVector>

#include <memory>

class VehiclePath;
class VehiclePosition;
class Intersection;
class TrafficEngine;

class VehicleRoute {
	std::string description;
	QMap<Intersection*, Direction::Cardinal> path;
	Intersection *firstIntersection;
	VehiclePath *destination;

public:
	static VehicleRoute* newVehicleRoute(const TrafficEngine *trafficEngine, VehiclePosition *currentPosition, VehiclePath *destination);

	std::string getDescription() const;
	void setDescription(const std::string &description);

	bool hasIntersection(Intersection* intersection) const;
	Direction::Cardinal getDirection(Intersection *intersection, Direction::Cardinal defaultDirection) const;
	void addIntersection(Intersection *intersection, Direction::Cardinal destinationDirection);
	bool updateSource(VehiclePosition *currentPosition);

	Direction::Cardinal *getNextIntersectionDirection(VehiclePath *currentPath) const;

	VehiclePath *getDestination() const;
	bool isValid(VehiclePosition *currentPosition = nullptr) const;
	bool isOnRoute(VehiclePosition *position) const;
	QVector<Intersection *> getIntersections() const;
	QVector<Direction::Cardinal> getDirections() const;

	static Direction::Cardinal getDirectionOfIntersection(Intersection *source, Intersection *directedIntersection, const Direction::Cardinal &defaultDirection);

private:
	VehicleRoute(const QMap<Intersection*, Direction::Cardinal> &path, Intersection *firstIntersection, VehiclePath *destination);

	static Intersection *getNextIntersection(const TrafficEngine *trafficEngine, VehiclePosition *position);
	static std::unique_ptr<QMap<Intersection*, Direction::Cardinal>> findDestinationFloodFill(Intersection *start, VehiclePath *finish, const Direction::Cardinal &startSourceDirection);
	static Intersection* findDestinationFloodFillRecursive(Intersection *start, VehiclePath *finish, QMap<Intersection *, Direction::Cardinal> &visited, const QMap<Intersection *, Direction::Cardinal> &justAdded);
	static Intersection* getOtherIntersection(Intersection *intersection, const Direction::Cardinal &cardinal);
};

#endif // VEHICLEROUTE_H
