#ifndef VEHICLEPOSITION_H
#define VEHICLEPOSITION_H

#include "direction.h"

#include <memory>
#include <math.h>

class Intersection;
class IntersectionLane;
class Road;
class RoadObject;
class TrafficEngine;
class TrafficLight;
class Vehicle;
class VehiclePath;
class VehicleRoute;

class QPointF;

class VehiclePosition {
    VehiclePath *vehiclePath;

protected:
    class SinCurveParameters;
	class EllipseParameters;

    VehiclePosition();

public:
    virtual ~VehiclePosition();

    virtual VehiclePath* getPath() const = 0;
	virtual VehiclePath* getNextPath() const = 0;

    virtual int getLane() const = 0;
    virtual int getChangeToLane() const = 0;
	virtual int getNextPathEnterLane() const = 0;
    virtual Direction::Cardinal getSourceDirection() const = 0;
    virtual Direction::Cardinal getDestinationDirection() const = 0;

    /**
     * @brief getLocation Gets the location of the vehicle in the parent coordinates
     * @return the location of the vehicle
     */
    virtual QPointF getLocation() const = 0;

    /**
     * @brief getDirection Gets the direction in radians of the direction of the vehicle in the parent coordinates
     * @return the direction in radians
     */
    virtual float getDirection() const = 0;

	virtual VehiclePosition* getNextPosition(double t) const = 0;

    virtual bool isAtEnd() const = 0;

	virtual VehiclePosition* getPathEndConnection(VehicleRoute *route = nullptr) const = 0;

	virtual bool requestLaneChangeLeft(double distance, Vehicle *vehicle) = 0;
	virtual bool requestLaneChangeRight(double distance, Vehicle *vehicle) = 0;

    /**
     * @brief requestLaneChange Attempt to change lanes
     * The new lane must be adjacent to the current lane.
     * @param newLane the new lane to change to
     * @param distance the distance used to make the lane change
	 * @param vehicle the vehicle requesting the lane change
     * @return true if the request is granted; otherwise, false
     */
	virtual bool requestLaneChange(int newLane, double distance, Vehicle *vehicle) = 0;

    /**
     * @brief getDistanceAlongPath Returns a relative value representing the distance travelled along the current path.
     * Although the value can be arbitary, any position that is closer to its destination on the path must return a higher value than one that is farther from its destination.
     * @return a value representing the distance travelled along the path
     */
    virtual double getDistanceAlongPath() const = 0;

    virtual double getPathLength() const = 0;

    /**
     * @brief getObjectAhead Gets the closest road object ahead of currentObject that occupy the same lane or transitioning to the same lane on the same path.
     * If the road object is making a transition to another lane, the road object returned will occupy the lane before the transition.
     * The client is responsible for deleting the returned object.
	 * @param trafficEngine the traffic engine managing all vehicles
     * @return the closest road object that is ahead of the current road object or nullptr if no object is ahead
     */
	std::pair<RoadObject*, double> getObjectAhead(TrafficEngine *trafficEngine, VehicleRoute *route = nullptr) const;

	std::pair<RoadObject*, double> getFirstObjectAfter(TrafficEngine *trafficEngine, double distanceTravelled, Intersection *intersection, const IntersectionLane &sourceLane) const;

    virtual TrafficLight* getEndOfPathTrafficLight() const = 0;

    virtual VehiclePosition* getEndOfPathTrafficLightPosition() const = 0;

	virtual RoadObject* getEndOfPathVehicleAheadRoadObject(int i) const;
	virtual RoadObject* getEndOfPathTrafficLightRoadObject() const;
	virtual RoadObject* getEndOfPathTrafficLightRoadObject(Road* road, bool movingFixedToFree, int lane) const;
	virtual RoadObject* getEndOfPathNoConnectionRoadObject() const;
	virtual RoadObject* getEndOfPathNoConnectionRoadObject(Road* road, bool movingFixedToFree, int lane) const;
	virtual RoadObject* getEndOfPathNoConnectionRoadObject(VehiclePosition *roadObjectPosition) const;

    virtual VehiclePosition* clone() const = 0;


	std::pair<RoadObject*, double> getObjectAhead2(TrafficEngine *trafficEngine, VehicleRoute *route = nullptr) const;

private:
	std::pair<RoadObject*, double> getObjectAheadIntersection(TrafficEngine *trafficEngine, Intersection *intersection, const IntersectionLane &sourceLane, const IntersectionLane &destinationLane, double minimumDistance, double accumulateDistance, QVector<VehiclePath*> &visitedPaths, bool sourceLaneOnly, bool checkVehicleBeyondTrafficLights, VehicleRoute *route = nullptr) const;
	std::pair<RoadObject*, double> getObjectAheadRoad(TrafficEngine *trafficEngine, Road *road, int lane, bool movingFixedToFree, double minimumDistance, double accumulateDistance, QVector<VehiclePath*> &visitedPaths, VehicleRoute *route = nullptr) const;
};

class VehiclePosition::SinCurveParameters {
    const double X1;
    const double Y1;
    const double X2;
    const double Y2;
    const double DOMAIN_LENGTH;

public:
    SinCurveParameters(double x1, double y1, double x2, double y2, double domainLength);
    double getX1() const;
    double getY1() const;
    double getX2() const;
    double getY2() const;
    double getDomainLength() const;
    double evaluate(double x) const;
    double derivative(double x) const;
    double arcLengthIntegral(double x) const;
    double arcLengthAt0() const;
    double arcLength2DerivativeAt0() const;
    double arcLength4DerivativeAt0() const;
    double arcLengthAntiDerivative(double x) const;
	/**
	 * @brief getLength Gets the length of the curve in 1 period from 0 to 2 PI
	 * @return the arc length of the curve from 0 to 2 PI
	 */
	double getLength(double tolerance = NAN) const;
    /**
     * @brief getLengthAfterPositionBisection Gets the x position such that f(x) - f(positionX) = curveDistance +/- maxCurveDistanceError where f is sinCurveArcLengthAntiDerivative
     * This function uses the bisection method to find the position
     * @param positionX the reference position
     * @param curveDistance the distance traveled along the curve
     * @param maxCurveDistanceError the maximum error in curve distance
     * @param maxIterations the maximum number of iterations before returning a value or -1 for no limit
     * @return the position x
     */
    double getLengthAfterPositionBisection(double positionX, double curveDistance, double maxCurveDistanceError, int maxIterations = 100) const;
    double getLengthAfterPositionNewton(double positionX, double curveDistance, double maxCurveDistanceError, int maxIterations = 100) const;

private:
	static double ellipticComplete2(double m, double tolerance = NAN);
};

class VehiclePosition::EllipseParameters {
    const double A_RADIUS;
    const double B_RADIUS;

public:
    EllipseParameters(double aRadius, double bRadius);

    double getARadius() const;
    double getBRadius() const;

	QPointF evaluate(double t) const;
	QPointF evaluateDerivative(double t) const;
    double arcLengthEstimate(double t1, double t2, int divisions = 10) const;
    double arcLengthEstimatePartialDerivative(double t1, double t2, bool relativeToX) const;
    double arcLengthEstimateDerivativeX(double t1, double t2, int divisions = 3) const;
    double arcLengthEstimateDerivativeY(double t1, double t2, int divisions = 3) const;
    double getLengthAfterPositionNewton(double t1, double curveDistance, double maxCurveDistanceError, int divisions = 3, int maxIterations = 100) const;
};

#endif // VEHICLEPOSITION_H
