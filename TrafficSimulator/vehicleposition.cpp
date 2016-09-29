#include "vehicleposition.h"

#include "intersection.h"
#include "road.h"
#include "roadobject.h"
#include "trafficengine.h"
#include "trafficlight.h"
#include "vehicle.h"
#include "vehicleroute.h"

#include <QVector>
#include <QDebug>

VehiclePosition::VehiclePosition() : vehiclePath(nullptr) {
}

VehiclePosition::~VehiclePosition() {
}

std::pair<RoadObject*, double> VehiclePosition::getObjectAhead(TrafficEngine *trafficEngine, VehicleRoute *) const {
	RoadObject *objectAhead = nullptr;
	const QVector<Vehicle *> objects = trafficEngine->getVehicles();
	double pathLength = getPathLength();
	double distanceAlongPath = getDistanceAlongPath();
	int currentlane = getLane();
	double shortestDistance = INFINITY;
	for (auto &o : objects) {
		VehiclePosition *position = o->getPosition();
		if (this != position && o != objectAhead && position != nullptr && position->getPath() == getPath() && (position->getLane() == currentlane || position->getChangeToLane() == currentlane)) {
			double distance = position->getDistanceAlongPath();
			if (distance > distanceAlongPath && distance < shortestDistance) {
				shortestDistance = distance;
				objectAhead = o;
			}
		}
	}
	if (objectAhead != nullptr) {
		return std::pair<RoadObject*, double>(new RoadObject(*objectAhead), shortestDistance - distanceAlongPath);
	} else {
		double remainingDistance = pathLength - distanceAlongPath;
		TrafficLight* trafficLight = getEndOfPathTrafficLight();
		if (trafficLight != nullptr) {
			TrafficLight::TrafficLightState state = trafficLight->getCurrentState();
			QVector<TrafficLightColor> goColors;
			goColors.push_back(TrafficLightColor::GREEN);
			goColors.push_back(TrafficLightColor::YELLOW);
			IntersectionLane sourceLane(getDestinationDirection(), getNextPathEnterLane());
			const QVector<Vehicle *> &vehiclesInIntersection = trafficEngine->getVehicles(trafficLight->getIntersection());
			if (vehiclesInIntersection.size() > 0) {
				for (auto &v : vehiclesInIntersection) {
					VehiclePosition *vehiclePosition = v->getPosition();
					if (vehiclePosition->getSourceDirection() != sourceLane.getDirection() && !state.hasPath(vehiclePosition->getSourceDirection(), vehiclePosition->getDestinationDirection())) {
						return std::pair<RoadObject*, double>(getEndOfPathVehicleAheadRoadObject(-1), remainingDistance);
					} else if (vehiclePosition->getLane() != IntersectionPosition::getLane(sourceLane) && vehiclePosition->getDestinationDirection() != sourceLane.getDirection() && vehiclePosition->getNextPathEnterLane() == getNextPathEnterLane()) {
						return std::pair<RoadObject*, double>(getEndOfPathVehicleAheadRoadObject(v->getID()), remainingDistance);
					}
				}
			}
			if (trafficLight->getDestinationPathsCount(sourceLane) == 0) {
				return std::pair<RoadObject*, double>(getEndOfPathTrafficLightRoadObject(), remainingDistance);
			} else {
				std::pair<RoadObject*, double> o = getFirstObjectAfter(trafficEngine, 0, trafficLight->getIntersection(), sourceLane);
				return std::pair<RoadObject*, double>(o.first != nullptr ? new RoadObject(*(o.first)) : nullptr, pathLength + o.second - distanceAlongPath);
			}
		} else {
			VehiclePath* nextPath = getNextPath();
			if (nextPath == nullptr) {
				return std::pair<RoadObject*, double>(nullptr, NAN);
			} else {
				// Current path is intersection
				int destinationLane = getNextPathEnterLane();
				double shortestDistance = INFINITY;
				for (auto &o : objects) {
					VehiclePosition *position = o->getPosition();
					if (o != objectAhead && position != nullptr && position->getPath() == nextPath && (position->getLane() == destinationLane || position->getChangeToLane() == destinationLane)) {
						double distance = position->getDistanceAlongPath();
						if (distance < shortestDistance) {
							shortestDistance = distance;
							objectAhead = o;
						}
					}
				}
				if (objectAhead != nullptr) {
					return std::pair<RoadObject*, double>(new RoadObject(*objectAhead), remainingDistance + shortestDistance);
				}
			}
		}
	}
	// TODO keep looking
	return std::pair<RoadObject*, double>(nullptr, NAN);
}

std::pair<RoadObject*, double> VehiclePosition::getFirstObjectAfter(TrafficEngine *trafficEngine, double distanceTravelled, Intersection *intersection, const IntersectionLane &sourceLane) const {
    RoadObject *objectAfter = nullptr;
	double closestDistance = INFINITY;
    const QVector<Vehicle *> objects = trafficEngine->getVehicles();
	for (auto &o : objects) {
        VehiclePosition *position = o->getPosition();
        if (position != nullptr && position->getPath() == intersection && position->getLane() == IntersectionPosition::getLane(sourceLane)) {
            double distance = position->getDistanceAlongPath();
			if (distance >= distanceTravelled && distance < closestDistance) {
				closestDistance = distance;
                objectAfter = o;
            }
        }
    }
	if (objectAfter == nullptr) {
		const IntersectionLane intersectionSourceLane(getDestinationDirection(), getNextPathEnterLane());
		const std::vector<IntersectionLane> &possiblePaths = intersection->getTrafficLight()->getPossiblePaths(intersectionSourceLane);
		for (IntersectionLane l : possiblePaths) {
			Road *road = intersection->getConnection(l.getDirection());
			if (road != nullptr && road != getPath()) {
				for (auto &o : objects) {
					VehiclePosition *position = o->getPosition();
					if (position != nullptr && position->getPath() == road && position->getNextPath() != intersection && road->getLanes() - 1 - position->getNextPathEnterLane() == l.getLane()) {
						double distance = intersection->getPathLength(intersectionSourceLane, l) + position->getDistanceAlongPath();
						if (distance < closestDistance) {
							closestDistance = distance;
							objectAfter = o;
						}
					}
				}
			}
		}
	}
	return std::pair<RoadObject*, double>(objectAfter, closestDistance);
}

RoadObject *VehiclePosition::getEndOfPathVehicleAheadRoadObject(int i) const {
	return new RoadObject(0, 0, getEndOfPathTrafficLightPosition(), 0, 0, 0, 0, true, "Vehicle Ahead " + std::to_string(i));
}

RoadObject *VehiclePosition::getEndOfPathTrafficLightRoadObject() const {
	TrafficLight* trafficLight = getEndOfPathTrafficLight();
	if (trafficLight != nullptr) {
		Intersection *intersection = trafficLight->getIntersection();
		VehiclePath *path = getPath();
		Road *road = nullptr;
		for (int i = 0; i < Direction::DIRECTIONS_COUNT; i++) {
			Road *connection = intersection->getConnection(Direction::getCardinal(i));
			if (connection == path) {
				road = connection;
				break;
			}
		}
		bool movingFixedToFree = road == nullptr || road->getFixedIntersection() != intersection;
		return getEndOfPathTrafficLightRoadObject(road, movingFixedToFree, getLane());
	} else {
		return getEndOfPathTrafficLightRoadObject(nullptr, true, 0);
	}
}

RoadObject *VehiclePosition::getEndOfPathTrafficLightRoadObject(Road* road, bool movingFixedToFree, int lane) const {
	if (road == nullptr) {
		return new RoadObject(0, 0, getEndOfPathTrafficLightPosition(), 0, 0, 0, 0, true, "Red Traffic Light");
	} else {
		return new RoadObject(0, 0, road->getEndOfPathTrafficLightPosition(movingFixedToFree, lane), 0, 0, 0, 0, true, "Red Traffic Light");
	}
}

RoadObject *VehiclePosition::getEndOfPathNoConnectionRoadObject() const {
	return getEndOfPathNoConnectionRoadObject(getEndOfPathTrafficLightPosition());
}

RoadObject *VehiclePosition::getEndOfPathNoConnectionRoadObject(Road *road, bool movingFixedToFree, int lane) const {
	if (road == nullptr) {
		return getEndOfPathNoConnectionRoadObject(getEndOfPathTrafficLightPosition());
	} else {
		return getEndOfPathNoConnectionRoadObject(road->getEndOfPathTrafficLightPosition(movingFixedToFree, lane));
	}
}

RoadObject *VehiclePosition::getEndOfPathNoConnectionRoadObject(VehiclePosition *roadObjectPosition) const {
	return new RoadObject(0, 0, roadObjectPosition, 0, 0, 0, 0, true, "No Connection");
}

std::pair<RoadObject*, double> VehiclePosition::getObjectAhead2(TrafficEngine *trafficEngine, VehicleRoute *route) const {
	double distanceTravelled = getDistanceAlongPath();
	VehiclePath *currentPath = getPath();
	for (auto &intersection : trafficEngine->getIntersections()) {
		if (currentPath == intersection) {
			const IntersectionLane &sourceLane = IntersectionPosition::getLane(getLane());
			const IntersectionLane &destinationLane = IntersectionPosition::getLane(getChangeToLane());
			QVector<VehiclePath*> visitedPaths;
			return getObjectAheadIntersection(trafficEngine, intersection, sourceLane, destinationLane, distanceTravelled, -distanceTravelled, visitedPaths, false, false, route);
		}
		for (int i = 0; i < Direction::DIRECTIONS_COUNT; i++) {
			Road *r = intersection->getConnection(Direction::getCardinal(i));
			if (currentPath == r) {
				TrafficLight *endOfPathTrafficLight = getEndOfPathTrafficLight();
				bool isFixedIntersection = endOfPathTrafficLight != nullptr && endOfPathTrafficLight->getIntersection() == r->getFixedIntersection();
				QVector<VehiclePath*> visitedPaths;
				return getObjectAheadRoad(trafficEngine, r, getLane(), !isFixedIntersection, distanceTravelled, -distanceTravelled, visitedPaths, route);
			}
		}
	}
	return std::pair<RoadObject*, double>(nullptr, 0);
}

std::pair<RoadObject*, double> VehiclePosition::getObjectAheadIntersection(TrafficEngine *trafficEngine, Intersection *intersection, const IntersectionLane &sourceLane, const IntersectionLane &destinationLane, double minimumDistance, double accumulateDistance, QVector<VehiclePath*> &visitedPaths, bool sourceLaneOnly, bool checkVehicleBeyondTrafficLights, VehicleRoute *route) const {
	RoadObject *objectAhead = nullptr;
	double closestDistance = INFINITY;
	for (auto &v : trafficEngine->getVehicles()) {
		VehiclePosition *position = v->getPosition();
		if (this == position) {
			if (visitedPaths.contains(intersection)) {
				return std::pair<RoadObject*, double>(nullptr, 0);
			}
		} else if (position != nullptr && position->getPath() == intersection && position->getLane() == IntersectionPosition::getLane(sourceLane) && (sourceLaneOnly || position->getChangeToLane() == IntersectionPosition::getLane(destinationLane))) {
			double distance = position->getDistanceAlongPath() - v->getVehicleLength() / 2.0;
			if (distance + v->getVehicleLength() / 2.0 >= minimumDistance && distance < closestDistance) {
				closestDistance = distance;
				objectAhead = v;
			}
		}
	}
	if (objectAhead != nullptr) {
		return std::pair<RoadObject*, double>(new RoadObject(*objectAhead), accumulateDistance + closestDistance + objectAhead->getVehicleLength() / 2.0);
	} else if (checkVehicleBeyondTrafficLights || visitedPaths.contains(intersection)) {
		return std::pair<RoadObject*, double>(nullptr, 0);
	} else {
		Road *nextConnection = intersection->getConnection(destinationLane.getDirection());
		if (nextConnection == nullptr) {
			return std::pair<RoadObject*, double>(getEndOfPathNoConnectionRoadObject(intersection->getEndOfPathPosition(sourceLane, destinationLane)), 0);
		} else {
			visitedPaths.push_back(intersection);
			bool isFixedIntersection = nextConnection->getFixedIntersection() == intersection;
			int lane = isFixedIntersection ? destinationLane.getLane() : nextConnection->getLanes() - 1 - destinationLane.getLane();
			return getObjectAheadRoad(trafficEngine, nextConnection, lane, isFixedIntersection, 0, accumulateDistance + intersection->getPathLength(sourceLane, destinationLane), visitedPaths, route);
		}
	}
}

std::pair<RoadObject*, double> VehiclePosition::getObjectAheadRoad(TrafficEngine *trafficEngine, Road *road, int lane, bool movingFixedToFree, double minimumDistance, double accumulateDistance, QVector<VehiclePath*> &visitedPaths, VehicleRoute *route) const {
	RoadObject *objectAhead = nullptr;
	double closestDistance = INFINITY;
	for (auto &v : trafficEngine->getVehicles()) {
		VehiclePosition *position = v->getPosition();
		if (this == position) {
			if (visitedPaths.contains(road)) {
				return std::pair<RoadObject*, double>(nullptr, 0);
			}
		} else if (position != nullptr && position->getPath() == road && position->getLane() == lane) { // TODO changing lanes
			double distance = position->getDistanceAlongPath() - v->getVehicleLength() / 2.0;
			if (distance + v->getVehicleLength() / 2.0 >= minimumDistance && distance < closestDistance) {
				closestDistance = distance;
				objectAhead = v;
			}
		}
	}
	if (objectAhead != nullptr) {
		return std::pair<RoadObject*, double>(new RoadObject(*objectAhead), accumulateDistance + closestDistance + objectAhead->getVehicleLength() / 2.0);
	} else if (visitedPaths.contains(road)) {
		return std::pair<RoadObject*, double>(nullptr, 0);
	} else {
		Intersection *nextIntersection = movingFixedToFree ? road->getFreeIntersection() : road->getFixedIntersection();
		if (nextIntersection == nullptr) {
			return std::pair<RoadObject*, double>(getEndOfPathNoConnectionRoadObject(road, movingFixedToFree, lane), 0);
		} else {
			visitedPaths.push_back(road);
			IntersectionLane sourceLane(movingFixedToFree ? road->getFreeIntersectionDirection() : road->getFixedIntersectionDirection(), movingFixedToFree ? road->getLanes() - 1 - lane : lane);
			TrafficLight *trafficLight = nextIntersection->getTrafficLight();
			if (trafficLight != nullptr) {
				const TrafficLight::TrafficLightState &currentState = trafficLight->getCurrentState();
				if (!TrafficLight::mustStop(currentState.getLightColor(sourceLane.getDirection(), sourceLane.getLane()))) {
					std::vector<Direction::Cardinal> possibleDirections;
					if (route != nullptr && route->hasIntersection(nextIntersection)) {
						possibleDirections.push_back(route->getDirection(nextIntersection, Direction::Cardinal::NORTH));
					} else {
						for (int i = 0; i < Direction::DIRECTIONS_COUNT; i++) {
							possibleDirections.push_back(Direction::getCardinal(i));
						}
					}
					bool foundClosest = false;
					std::pair<RoadObject*, double> closest(nullptr, INFINITY);
					for (auto &direction : possibleDirections) {
						if (currentState.hasPath(sourceLane.getDirection(), direction)) {
							std::vector<std::pair<RoadObject*, double>> objectsAhead;
							for (int i = 0; i < nextIntersection->getLanes(direction); i++) {
								if (!TrafficLight::mustStop(currentState.getLightColor(sourceLane.getDirection(), sourceLane.getLane(), direction, i))) {
									QVector<VehiclePath*> visitedPaths2(visitedPaths);
									const std::pair<RoadObject*, double> &o = getObjectAheadIntersection(trafficEngine, nextIntersection, sourceLane, IntersectionLane(direction, i), 0, accumulateDistance + road->getPathLength(lane), visitedPaths2, false, route);
									objectsAhead.push_back(o);
								}
							}
							for (auto &o : objectsAhead) {
								if (o.second < closest.second) {
									foundClosest = true;
									closest.first = o.first;
									closest.second = o.second;
								}
							}
						}
					}
					if (foundClosest) {
						if (closest.first != nullptr) {
							closest.first = new RoadObject(*(closest.first));
						}
						return closest;
					}
				}
				const std::pair<RoadObject*, double> &o = getObjectAheadIntersection(trafficEngine, nextIntersection, sourceLane, IntersectionLane(Direction::Cardinal::NORTH, 0), 0, 0, visitedPaths, true, true, route);
				if (o.first == nullptr || o.second - o.first->getVehicleLength() / 2.0 > 0) {
					return std::pair<RoadObject*, double>(getEndOfPathTrafficLightRoadObject(road, movingFixedToFree, lane), accumulateDistance + road->getPathLength(lane));
				} else {
					return std::pair<RoadObject*, double>(o.first, accumulateDistance + road->getPathLength(lane) + o.second);
				}
			} else {
				return std::pair<RoadObject*, double>(getEndOfPathNoConnectionRoadObject(road, movingFixedToFree, lane), 0);
			}
		}
	}
}

VehiclePosition::SinCurveParameters::SinCurveParameters(double x1, double y1, double x2, double y2, double domainLength) : X1(x1), Y1(y1), X2(x2), Y2(y2), DOMAIN_LENGTH(domainLength) {
}

double VehiclePosition::SinCurveParameters::getX1() const {
	return X1;
}

double VehiclePosition::SinCurveParameters::getY1() const {
	return Y1;
}

double VehiclePosition::SinCurveParameters::getX2() const {
	return X2;
}

double VehiclePosition::SinCurveParameters::getY2() const {
	return Y2;
}

double VehiclePosition::SinCurveParameters::getDomainLength() const {
	return DOMAIN_LENGTH;
}

double VehiclePosition::SinCurveParameters::evaluate(double x) const {
	return ((Y2 - Y1) * sin(M_PI / (X2 - X1) * (x - (X2 + X1) / 2.0)) + (Y2 + Y1)) / 2.0;
}

double VehiclePosition::SinCurveParameters::derivative(double x) const {
	return M_PI * (Y2 - Y1) / (X2 - X1) * cos(M_PI / (X2 - X1) * (x - (X2 + X1) / 2.0)) / 2.0;
}

double VehiclePosition::SinCurveParameters::arcLengthIntegral(double x) const {
	x -= (X1 +  X2) / 2.0;
	return sqrt(1 + pow(M_PI * (Y2 - Y1) / (X2 - X1) / 48.0 * (24 - 12 * pow(x * M_PI / (X2 - X1), 2) + pow(x * M_PI / (X2 - X1), 4)), 2));
}

double VehiclePosition::SinCurveParameters::arcLengthAt0() const {
	return sqrt(1 + pow(M_PI / 2.0 * (Y2 - Y1) / (X2 - X1), 2));
}

double VehiclePosition::SinCurveParameters::arcLength2DerivativeAt0() const {
	return -pow(M_PI / (X2 - X1), 4) * pow(Y2 - Y1, 2) / sqrt(1 + pow(M_PI / 2.0 * (Y2 - Y1) / (X2 - X1), 2)) / 4.0;
}

double VehiclePosition::SinCurveParameters::arcLength4DerivativeAt0() const {
	double secondDerivative = arcLength2DerivativeAt0();
	return -pow(2 * M_PI / (X2 - X1), 2) * secondDerivative - 3 / arcLengthAt0() * pow(secondDerivative, 2);
}

double VehiclePosition::SinCurveParameters::arcLengthAntiDerivative(double x) const {
	double shiftedX = x - (X1 + X2) / 2.0;
	return arcLengthAt0() * shiftedX + arcLength2DerivativeAt0() / 6.0 * pow(shiftedX, 3) + arcLength4DerivativeAt0() / 120.0 * pow(shiftedX, 5);
}

double VehiclePosition::SinCurveParameters::getLength(double tolerance) const {
	double k = (Y2 - Y1) / (X2 - X1) * M_PI / 2.0;
	k *= k;
	return 4.0 * (X2 - X1) / M_PI * std::sqrt(1.0 + k) * ellipticComplete2(k / (1.0 + k), tolerance);
}


double VehiclePosition::SinCurveParameters::getLengthAfterPositionBisection(double positionX, double curveDistance, double maxCurveDistanceError, int maxIterations) const {
	maxCurveDistanceError = std::max(fabs(maxCurveDistanceError), 0.0001);
	double targetValue = arcLengthAntiDerivative(positionX) + curveDistance;
	double boundLower = X1;
	double boundLowerValue = arcLengthAntiDerivative(boundLower);
	double boundUpper = X2;
	double boundUpperValue = arcLengthAntiDerivative(boundUpper);
	if ((boundLowerValue < targetValue) == (boundUpperValue < targetValue)) {
		return NAN;
	}
	double x = (boundLower + boundUpper) / 2.0;
	double xValue = arcLengthAntiDerivative(x);
	for (int i = 0; fabs(xValue - targetValue) > maxCurveDistanceError && (i < maxIterations || maxIterations < 0); i++) {
		if ((xValue < targetValue) == (boundLowerValue < targetValue)) {
			boundLower = x;
			boundLowerValue = xValue;
		} else {
			boundUpper = x;
			boundUpperValue = xValue;
		}
		x = (boundLower + boundUpper) / 2.0;
		xValue = arcLengthAntiDerivative(x);
	}
	return x;
}

double VehiclePosition::SinCurveParameters::getLengthAfterPositionNewton(double positionX, double curveDistance, double maxCurveDistanceError, int maxIterations) const {
	if (curveDistance == 0) {
		return positionX;
	} else {
		maxCurveDistanceError = std::max(fabs(maxCurveDistanceError), 0.0001);
		double targetValue = arcLengthAntiDerivative(positionX) + curveDistance;
		double initialGuess = (X1 + X2) / 2.0;
		if (curveDistance > 0) {
			initialGuess = positionX + (X2 - positionX) / 200.0;
		} else if (curveDistance < 0) {
			initialGuess = positionX - (positionX - X1) / 200.0;
		}
		double initialGuessValue = arcLengthAntiDerivative(initialGuess);
		for (int i = 0; i < maxIterations || maxIterations < 0; i++) {
			double nextGuess = initialGuess - (initialGuessValue - targetValue) / arcLengthIntegral(initialGuess);
			double nextGuessValue = arcLengthAntiDerivative(nextGuess);
			if (fabs(nextGuessValue - targetValue) <= maxCurveDistanceError) {
				return nextGuess;
			} else {
				initialGuess = nextGuess;
				initialGuessValue = nextGuessValue;
			}
		}
		return initialGuess;
	}
}

double VehiclePosition::SinCurveParameters::ellipticComplete2(double m, double tolerance) {
	if (m == 0.0) {
		return M_PI / 2.0;
	} else if (m == 1.0) {
		return 1.0;
	} else {
		tolerance = std::fabs(tolerance);
		double v = (1.0 + std::sqrt(1.0 - m)) / 2.0;
		double w = m / v / 4.0;
		double s = v * v;
		double f = 1.0;
		do {
			v = (v + std::sqrt((v - w) * (v + w))) / 2.0;
			w = w * w / v / 4.0;
			f *= 2.0;
			s -= f * w * w;
		} while (std::fabs(w) > (std::isnan(tolerance) ? 0.0001 : tolerance));
		return M_PI / 2.0 * s / v;
	}
}

VehiclePosition::EllipseParameters::EllipseParameters(double aRadius, double bRadius) : A_RADIUS(aRadius), B_RADIUS(bRadius) {
	if (aRadius == 0 && bRadius == 0) throw std::runtime_error("null ellipse");
	if (std::isnan(aRadius) || std::isnan(bRadius)) throw std::runtime_error("nan ellipse");
}

double VehiclePosition::EllipseParameters::getARadius() const {
	return A_RADIUS;
}

double VehiclePosition::EllipseParameters::getBRadius() const {
	return B_RADIUS;
}

QPointF VehiclePosition::EllipseParameters::evaluate(double t) const {
	return QPointF(A_RADIUS * std::cos(t), B_RADIUS * std::sin(t));
}

QPointF VehiclePosition::EllipseParameters::evaluateDerivative(double t) const
{
	return QPointF(-A_RADIUS * std::sin(t), B_RADIUS * std::cos(t));
}

double VehiclePosition::EllipseParameters::arcLengthEstimate(double t1, double t2, int divisions) const {
	if (t1 == t2) {
		return 0;
	}
	double arcLength = 0;
	if (t2 >= t1 + M_PI) {
		double h = pow((A_RADIUS - B_RADIUS) / (A_RADIUS + B_RADIUS), 2);
		arcLength += M_PI / 2 * (A_RADIUS + B_RADIUS) * (1 + (3 * h) / (10 + sqrt(4 - 3 * h)));
		t1 += M_PI;
		if (t1 == t2) {
			return arcLength;
		}
	}
	double sum = 0;
	for (int i = 0; i < divisions; i++) {
		double t_1 = t1 + (i * (t2 - t1) / divisions);
		double t_2 = t1 + ((i + 1) * (t2 - t1) / divisions);
		sum += (t_1 - t_2) * sqrt(pow(A_RADIUS * (cos(t_2) - cos(t_1)), 2) + pow(B_RADIUS * (sin(t_2) - sin(t_1)), 2)) / (2 * sin((t_1 - t_2) / 2));
	}
	return sum;
}

double VehiclePosition::EllipseParameters::arcLengthEstimatePartialDerivative(double t1, double t2, bool relativeToX) const {
	double arcLength = 0;
	if (t2 >= t1 + M_PI) {
		double h = pow((A_RADIUS - B_RADIUS) / (A_RADIUS + B_RADIUS), 2);
		arcLength += M_PI / 2 * (A_RADIUS + B_RADIUS) * (1 + (3 * h) / (10 + sqrt(4 - 3 * h)));
		t1 += M_PI;
	}
	double k = pow(A_RADIUS * (cos(t2) - cos(t1)), 2) + pow(B_RADIUS * (sin(t2) - sin(t1)), 2);
	if (relativeToX) {
		return sqrt(k) / (2 * sin((t1 - t2) / 2)) * ((t1 - t2) * ((pow(B_RADIUS, 2) * cos(t2) * (sin(t2) - sin(t1)) - pow(A_RADIUS, 2) * sin(t2) * (cos(t2) - cos(t1))) / k - 1 / (2 * tan((t1 - t2) / 2))) + 1);
	} else {
		return sqrt(k) / (2 * sin((t1 - t2) / 2)) * ((t1 - t2) * ((pow(B_RADIUS, 2) * cos(t2) * (sin(t2) - sin(t1)) - pow(A_RADIUS, 2) * sin(t2) * (cos(t2) - cos(t1))) / k + 1 / (2 * tan((t1 - t2) / 2))) - 1);
	}
}

double VehiclePosition::EllipseParameters::arcLengthEstimateDerivativeX(double t1, double t2, int divisions) const {
	double arcLength = 0;
	if (t2 >= t1 + M_PI) {
		double h = pow((A_RADIUS - B_RADIUS) / (A_RADIUS + B_RADIUS), 2);
		arcLength += M_PI / 2 * (A_RADIUS + B_RADIUS) * (1 + (3 * h) / (10 + sqrt(4 - 3 * h)));
		t1 += M_PI;
	}
	double sum = 0;
	for (int i = 0; i < divisions; i++) {
		double t_1 = t1 + (i * (t2 - t1) / divisions);
		double t_2 = t1 + ((i + 1) * (t2 - t1) / divisions);
		sum += arcLengthEstimatePartialDerivative(t_1, t_2, true) * (1.0 - i * 1.0 / divisions) + arcLengthEstimatePartialDerivative(t_1, t_2, false) * (1.0 - (i + 1.0) * 1.0 / divisions);
	}
	return sum;
}

double VehiclePosition::EllipseParameters::arcLengthEstimateDerivativeY(double t1, double t2, int divisions) const {
	double arcLength = 0;
	if (t2 >= t1 + M_PI) {
		double h = pow((A_RADIUS - B_RADIUS) / (A_RADIUS + B_RADIUS), 2);
		arcLength += M_PI / 2 * (A_RADIUS + B_RADIUS) * (1 + (3 * h) / (10 + sqrt(4 - 3 * h)));
		t1 += M_PI;
	}
	double sum = 0;
	for (int i = 0; i < divisions; i++) {
		double t_1 = t1 + (i * (t2 - t1) / divisions);
		double t_2 = t1 + ((i + 1) * (t2 - t1) / divisions);
		sum += i * arcLengthEstimatePartialDerivative(t_1, t_2, true) / divisions + (i + 1) * arcLengthEstimatePartialDerivative(t_1, t_2, false) / divisions;
	}
	return sum;
}

double IntersectionPosition::EllipseParameters::getLengthAfterPositionNewton(double t1, double curveDistance, double maxCurveDistanceError, int divisions, int maxIterations) const {
	if (curveDistance == 0) {
		return t1;
	}
	maxCurveDistanceError = std::max(fabs(maxCurveDistanceError), 0.0001);
	double initialGuess = t1 + (curveDistance >= 0 ? 0.0001 : -0.0001);
	double initialGuessValue = arcLengthEstimate(t1, initialGuess, divisions);
	curveDistance = fabs(curveDistance);
	for (int i = 0; i < maxIterations || maxIterations < 0; i++) {
		double d = arcLengthEstimateDerivativeY(t1, initialGuess, divisions);
		double nextGuess = initialGuess - (initialGuessValue - curveDistance) / d;
		double nextGuessValue = arcLengthEstimate(t1, nextGuess, divisions);
		if (fabs(nextGuessValue - curveDistance) <= maxCurveDistanceError) {
			return nextGuess;
		} else {
			initialGuess = nextGuess;
			initialGuessValue = nextGuessValue;
		}
	}
	return initialGuess;
}
