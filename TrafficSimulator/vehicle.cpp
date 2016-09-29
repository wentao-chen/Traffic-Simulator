#include "vehicle.h"

#include "constants.h"
#include "general.h"
#include "intersection.h"
#include "linearroad.h"
#include "roadobject.h"
#include "vehicleroute.h"
#include "trafficengine.h"
#include "trafficlight.h"

#include <QGraphicsSceneMouseEvent>
#include <QPainter>

#include <assert.h>

unsigned int Vehicle::idAssigner = 0;
int Vehicle::highlightBorderSize = 5;

Vehicle::Vehicle(TrafficEngine *trafficEngine, const std::string &name, const TrafficEngine::Models &model, double vehicleWidth, double vehicleLength, double maxAcceleration, double comfortableDeceleration, double safetyDistance, QGraphicsItem *parent) :
        QGraphicsItem(parent),
		RoadObject(vehicleWidth, vehicleLength, nullptr, 0, maxAcceleration, comfortableDeceleration, safetyDistance, false, name + " " + std::to_string(idAssigner)),
		id(idAssigner++),
		name(name),
		model(model),
        trafficEngine(trafficEngine),
        temporaryPosition(nullptr),
		temporaryNextPathPosition(nullptr),
		temporarySpeed(0),
		highlight(false),
		timer(QElapsedTimer()),
		route(std::unique_ptr<VehicleRoute>(nullptr)) {

	this->timer.start();
}

Vehicle::~Vehicle() {
}

unsigned int Vehicle::getID() const {
	return this->id;
}

std::string Vehicle::getName() const {
	return this->name;
}

TrafficEngine::Models Vehicle::getModel() const {
	return this->model;
}

void Vehicle::setModel(TrafficEngine::Models m) {
	this->model = m;
}

QPainterPath Vehicle::shape() const {
	QPainterPath path;
	path.addRect(0, 0, getVehicleWidth(), getVehicleLength());
	return path;
}

QRectF Vehicle::boundingRect() const {
	return QRectF(-Vehicle::highlightBorderSize, -Vehicle::highlightBorderSize, getVehicleWidth() + Vehicle::highlightBorderSize * 2, getVehicleLength() + Vehicle::highlightBorderSize * 2);
}

void Vehicle::paint(QPainter *painter, const QStyleOptionGraphicsItem*, QWidget*) {
	for (auto &v : this->trafficEngine->getVehicles()) {
		if (v != this && collidesWithItem(v)) {
			painter->fillRect(boundingRect(), Qt::yellow);
		}
	}
	if (isHighlighted()) {
		painter->fillRect(boundingRect(), Qt::green);
	}
    painter->fillRect(0, 0, getVehicleWidth(), getVehicleLength() / 4, Qt::blue);
    painter->fillRect(0, getVehicleLength() / 4, getVehicleWidth(), getVehicleLength() * 3 / 4, Qt::red);
}

void Vehicle::setPosition(VehiclePosition *currentPosition) {
	VehicleRoute *previousRoute = this->trafficEngine->getHighlightedRoute();
	VehiclePosition *previousPosition = getPosition();
	VehiclePath *path = previousPosition != nullptr ? previousPosition->getPath() : nullptr;
    RoadObject::setPosition(currentPosition);
	VehicleRoute *route = getRoute();
	if (route != nullptr) {
		if (currentPosition->getPath() == route->getDestination()) {
			clearRoute();
		} else if (path != currentPosition->getPath()) {
			if (!route->isOnRoute(currentPosition)) {
				setRoute(route->getDestination());
				this->trafficEngine->highlightRoute(nullptr);
				this->trafficEngine->highlightRoute(this);
			} else if (route->updateSource(currentPosition) && previousRoute != nullptr && route == previousRoute) {
				this->trafficEngine->highlightRoute(nullptr);
				this->trafficEngine->highlightRoute(this);
			}
		}
	}
	if (route == nullptr && CustomRandom::getRand() < Speeds::LANE_CHANGE_RATE) {
		const double laneChangeDistance = Distances::LANE_CHANGE_DISTANCE;
		double pathLength = currentPosition->getPathLength();
		if (pathLength - currentPosition->getDistanceAlongPath() >= Distances::LANE_CHANGE_DISTANCE + Distances::LANE_CHANGE_BUFFER_DISTANCE) {
			if (CustomRandom::getRand() < 0.5) {
				if (!getPosition()->requestLaneChangeLeft(laneChangeDistance, this)) {
					getPosition()->requestLaneChangeRight(laneChangeDistance, this);
				}
			} else {
				if (!getPosition()->requestLaneChangeRight(laneChangeDistance, this)) {
					getPosition()->requestLaneChangeLeft(laneChangeDistance, this);
				}
			}
		}
	}
	updatePosition();
}

bool Vehicle::isOnPath(VehiclePath *path) {
    VehiclePosition* currentPosition = getPosition();
    return currentPosition != nullptr && path != nullptr && path == currentPosition->getPath();
}

void Vehicle::updatePosition() {
    VehiclePosition* currentPosition = getPosition();
    if (currentPosition != nullptr) {
        float a = currentPosition->getDirection() + M_PI / 2;
        setRotation(a / M_PI * 180);
        setPos(currentPosition->getLocation() - QPointF(getVehicleWidth() / 2 * cos(a) - getVehicleLength() / 2 * sin(a), getVehicleWidth() / 2 * sin(a) + getVehicleLength() / 2 * cos(a)));
    }
}

void Vehicle::prepareToMove(float timeElapsed) {
    if (this->temporaryPosition != nullptr) {
        delete this->temporaryPosition;
	}

	VehiclePosition* currentPosition = getPosition();
	const double speed = getSpeed();
	const double acceleration = getMaxAcceleration();
	double deceleration = getComfortableDeceleration();
	double currentDistance = currentPosition->getDistanceAlongPath();
	double remainingDistance = currentPosition->getPathLength() - currentDistance - getVehicleLength() / 2.0;

	bool vehicleDescriptionUsed = false;
	std::string vehicleDescription = Description::NO_VEHICLE_AHEAD_MESSAGE;
	VehiclePath *currentPath = currentPosition->getPath();
	double speedLimit = currentPath->getSpeedLimit();
	VehiclePath *nextPath = currentPosition->getNextPath();

	std::pair<RoadObject*, double> objectAheadInfo = currentPosition->getObjectAhead2(this->trafficEngine, getRoute());
	std::unique_ptr<RoadObject> objectAhead(objectAheadInfo.first);
	double objectAheadDistance = objectAheadInfo.second;
	if (objectAhead.get() != nullptr) {
		vehicleDescriptionUsed = true;
		if (remainingDistance < 0) {
			setTemporaryNextPathPosition();
			if (this->temporaryNextPathPosition != nullptr) {
				remainingDistance += this->temporaryNextPathPosition->getPathLength();
			}
		}
	} else if (nextPath == nullptr) {
		if (!vehicleDescriptionUsed) {
			vehicleDescriptionUsed = true;
			vehicleDescription = "No Connection";
		}
		if (speed == 0) {
			this->trafficEngine->removeVehicle(this);
			return;
		}
	}
	if ((objectAhead.get() == nullptr || objectAheadDistance - objectAhead->getVehicleLength() / 2.0 > remainingDistance) && nextPath != nullptr && remainingDistance >= 0) {
		TrafficLight* trafficLight = currentPosition->getEndOfPathTrafficLight();
		if (trafficLight != nullptr) {
			const TrafficLight::TrafficLightState &currentTrafficLightState = trafficLight->getCurrentState();
			const IntersectionLane intersectionSourceLane(currentPosition->getDestinationDirection(), currentPosition->getNextPathEnterLane());
			const TrafficLightColor &currentLightColor = currentTrafficLightState.getLightColor(intersectionSourceLane.getDirection(), intersectionSourceLane.getLane());
			// Check if the next traffic light can be made
			if (nextPath == nullptr) {
				// Must stop
				objectAhead.reset(currentPosition->getEndOfPathNoConnectionRoadObject());
				objectAheadDistance = remainingDistance + getVehicleLength() / 2;
			} else if (currentLightColor == TrafficLightColor::ILLEGAL || currentLightColor == TrafficLightColor::RED) {
				objectAhead.reset(currentPosition->getEndOfPathTrafficLightRoadObject());
				objectAheadDistance = remainingDistance + getVehicleLength() / 2;
			} else {
				double nextPathSpeedLimit = nextPath->getSpeedLimit();
				double maximumSpeedAtIntersection = std::min(speedLimit, std::min(nextPathSpeedLimit, std::sqrt(speed * speed + 2 * acceleration * remainingDistance)));

				double maximumSpeed = std::min(speedLimit, std::sqrt((2 * acceleration * deceleration * remainingDistance - acceleration * std::pow(maximumSpeedAtIntersection, 2) + deceleration * std::pow(speed, 2)) / (acceleration + deceleration)));
				maximumSpeedAtIntersection = std::min(maximumSpeedAtIntersection, maximumSpeed);
				const double accelerateTime = (maximumSpeed - speed) / acceleration;
				const double decelerateTime = (maximumSpeed - maximumSpeedAtIntersection) / deceleration;
				const double accelerateDistance = (speed + maximumSpeed) / 2.0 * accelerateTime;
				const double decelerateDistance = (maximumSpeed + maximumSpeedAtIntersection) / 2.0 * decelerateTime;
				assert(speed >= 0);
				assert(maximumSpeedAtIntersection >= 0);
				assert(maximumSpeed >= 0);
				assert(maximumSpeed >= maximumSpeedAtIntersection);
				assert(decelerateTime >= 0);

				double fastestTimeToReachIntersection = 0;
				if (remainingDistance <= 0) {
					fastestTimeToReachIntersection = 0;
				} else if (accelerateDistance > 0 && decelerateDistance > 0 && accelerateDistance + decelerateDistance < remainingDistance) {
					// Car will accelerate to maximum speed, maintain speed, then decelerate to next path
					fastestTimeToReachIntersection = accelerateTime + (remainingDistance - accelerateDistance - decelerateDistance) / maximumSpeed + decelerateTime;
				} else if (accelerateDistance >= remainingDistance || decelerateDistance <= 0) {
					// Car will accelerate to next path
					fastestTimeToReachIntersection = 2 * remainingDistance / (speed + std::min(maximumSpeedAtIntersection, maximumSpeed));
				} else if (accelerateTime >= 0) {
					// Car will accelerate to maximum speed then immediately decelerate to next path
					fastestTimeToReachIntersection = accelerateTime + decelerateTime;
				} else {
					// Car will decelerate immediately to next path
					fastestTimeToReachIntersection = remainingDistance * 2.0 / (speed + maximumSpeedAtIntersection);
				}
				assert(fastestTimeToReachIntersection >= 0);

				double timeRemaining = currentTrafficLightState.getTimeToNextLight(TrafficLightColor::ILLEGAL, intersectionSourceLane);
				if (fastestTimeToReachIntersection >= timeRemaining) {
					objectAhead.reset(currentPosition->getEndOfPathTrafficLightRoadObject());
					objectAheadDistance = remainingDistance + getVehicleLength() / 2;
					if (!vehicleDescriptionUsed) {
						vehicleDescriptionUsed = true;
						vehicleDescription = "Cannot make current green light";
					}
					double requiredMinDeceleration = speed * speed / 2.0 / remainingDistance;
					if (requiredMinDeceleration > deceleration) {
						deceleration = requiredMinDeceleration;
						vehicleDescriptionUsed = true;
						vehicleDescription = "Cannot make current green light. Emergency Brake=" + std::to_string(deceleration) + "m/s^2";
					}
				}
			}
		}
	}
	VehicleRoute *route = getRoute();
	if (route != nullptr) {
		TrafficLight *trafficLight = currentPosition->getEndOfPathTrafficLight();
		if (trafficLight != nullptr) {
			Intersection *nextIntersection = trafficLight->getIntersection();
			if (nextIntersection != nullptr && route->hasIntersection(nextIntersection)) {
				const Direction::Cardinal &destinationDirection = currentPosition->getDestinationDirection();
				const Direction::Cardinal &nextDirection = route->getDirection(nextIntersection, destinationDirection);
				Road *currentRoad = nextIntersection->getConnection(destinationDirection);
				if (currentRoad != nullptr) {
					const double laneChangeDistance = Distances::LANE_CHANGE_DISTANCE;
					if (remainingDistance >= Distances::LANE_CHANGE_DISTANCE + Distances::LANE_CHANGE_BUFFER_DISTANCE) {
						bool isFixedIntersection = currentRoad->getFixedIntersection() == nextIntersection;
						int sourceLane = isFixedIntersection ? currentPosition->getLane() : currentRoad->getLanes() - 1 - currentPosition->getLane();
						if (!trafficLight->hasPath(currentPosition->getDestinationDirection(), sourceLane, nextDirection)) {
							if (nextDirection == Direction::getLeft(destinationDirection)) {
								currentPosition->requestLaneChangeLeft(laneChangeDistance, this);
							} else if (nextDirection == Direction::getRight(destinationDirection)) {
								currentPosition->requestLaneChangeRight(laneChangeDistance, this);
							} else if (nextDirection == Direction::getOpposite(destinationDirection)) {
								std::array<int, 2> availableLanes = currentRoad->getExitingLanesRange(isFixedIntersection);
								bool turnLeft = isFixedIntersection;
								for (int i = availableLanes.at(0); i < sourceLane; i++) {
									if (trafficLight->hasPath(destinationDirection, i, nextDirection)) {
										turnLeft = !turnLeft;
										break;
									}
								}
								if (turnLeft) {
									currentPosition->requestLaneChangeLeft(laneChangeDistance, this);
								} else {
									currentPosition->requestLaneChangeRight(laneChangeDistance, this);
								}
							}
						}
					} else if (currentPosition->getLane() == currentPosition->getChangeToLane()) {
						const std::vector<Direction::Cardinal> &possibleDirections = trafficLight->getPossibleDirections(IntersectionLane(currentPosition->getDestinationDirection(), currentPosition->getNextPathEnterLane()));
						if (std::find(possibleDirections.begin(), possibleDirections.end(), nextDirection) == possibleDirections.end()) {
							setRoute(route->getDestination());
						}
					}
				}
			}
		}
	}

	double distanceTravelled = speed * timeElapsed + acceleration * timeElapsed * timeElapsed / 2.0;
	double velocityChanged = acceleration * timeElapsed;
	if (objectAhead.get() != nullptr) {
		const double safetyDistance = objectAhead->getSafetyDistance();
		VehiclePosition *position = objectAhead->getPosition();
		if (position != nullptr) {
			double vehicleAheadDistance = currentDistance + objectAheadDistance;
			vehicleDescription += " (" + std::to_string(vehicleAheadDistance - currentDistance) + ")";
			if (vehicleAheadDistance > currentDistance) {
				const std::pair<double, double> &velocityDistanceChange = this->model.apply(this, currentDistance, speed, acceleration, deceleration, objectAhead.get(), vehicleAheadDistance, speedLimit, timeElapsed, safetyDistance, remainingDistance);
				if (!std::isnan(velocityDistanceChange.first)) {
					velocityChanged = velocityDistanceChange.first;
				}
				if (!std::isnan(velocityDistanceChange.second)) {
					distanceTravelled = velocityDistanceChange.second;
				}
			}
		}
	}
	distanceTravelled = std::max(distanceTravelled, 0.0);
	double newSpeed = std::max(speed + velocityChanged, 0.0);
	double nextPathSpeedLimit = nextPath != nullptr ? nextPath->getSpeedLimit() : 0;
	if (nextPath != nullptr && nextPathSpeedLimit < newSpeed) { // Slow down for next path
		double decelerateDistance = (newSpeed * newSpeed - nextPathSpeedLimit * nextPathSpeedLimit) / 2.0 / deceleration;
		if (remainingDistance - distanceTravelled < 0) {
			speedLimit = nextPathSpeedLimit;
		} else if (remainingDistance - distanceTravelled <= decelerateDistance) {
			speedLimit = std::min(speedLimit, sqrt(nextPathSpeedLimit * nextPathSpeedLimit + 2 * deceleration * (remainingDistance - distanceTravelled)));
			assert(speedLimit >= 0);
			if (!vehicleDescriptionUsed) {
				vehicleDescriptionUsed = true;
				vehicleDescription = "Slowing down to " + std::to_string((long) nextPathSpeedLimit) + " for next path";
			}
		}
	}
	if (newSpeed > speedLimit) {
		newSpeed = std::max(speed - deceleration * timeElapsed, speedLimit);
		if (!vehicleDescriptionUsed) {
			vehicleDescriptionUsed = true;
			vehicleDescription = Description::NO_VEHICLE_AHEAD_MESSAGE + " at speed limit";
		}
	}
	if (remainingDistance - distanceTravelled < 0) {
		setTemporaryNextPathPosition();
	}
	if (remainingDistance <= 0 && this->temporaryNextPathPosition == nullptr) {
		newSpeed = 0;
		distanceTravelled = 0;
	}
	this->temporarySpeed = newSpeed;
	this->temporaryPosition = currentPosition->getNextPosition(distanceTravelled);

	if (objectAhead.get() != nullptr) {
		this->trafficEngine->updateObjectAheadInfo(this, objectAhead->getObjectAheadInfo());
	} else {
		this->trafficEngine->updateObjectAheadInfo(this, vehicleDescription);
	}
}

void Vehicle::move() {
	if (this->temporaryPosition != nullptr) {
		if (isAtPathEnd(this->temporaryPosition)) {
			VehiclePosition *nextPosition = getPathEndNextPosition(this->temporaryPosition);
			if (this->temporaryPosition != nextPosition) {
				delete this->temporaryPosition;
				this->temporaryPosition = nextPosition;
			}
			this->temporaryNextPathPosition = nullptr;
		}
		if (this->temporaryPosition != nullptr) {
            setPosition(this->temporaryPosition);
            this->temporaryPosition = nullptr;
		} else {
			this->temporarySpeed = 0;
		}
    }
	setSpeed(this->temporarySpeed);
}

void Vehicle::move(float timeElapsed) {
	prepareToMove(timeElapsed);
	move();
}

Direction::Cardinal Vehicle::getNextDirection(Intersection*, QVector<Direction::Cardinal> validDirections) {
    return validDirections.at(qrand() % validDirections.size());
}

int Vehicle::getNextLane(Intersection*, Direction::Cardinal, QVector<int> validLanes) {
    return validLanes.at(qrand() % validLanes.size());
}

void Vehicle::setSpeed(double speed) {
    RoadObject::setSpeed(speed);
	this->temporarySpeed = getSpeed();
	this->trafficEngine->updateVehiclesInfoTable(this);
	if (speed <= 0 && this->temporaryNextPathPosition != nullptr) {
		delete this->temporaryNextPathPosition;
		this->temporaryNextPathPosition = nullptr;
	}
}

std::string Vehicle::getRouteMessage() const {
	VehicleRoute *route = getRoute();
	return route != nullptr ? route->getDescription() : "No Route";
}

VehicleRoute *Vehicle::getRoute() const {
	return this->route.get();
}

void Vehicle::clearRoute() {
	VehicleRoute *previousRoute = this->trafficEngine->getHighlightedRoute();
	if (previousRoute != nullptr && getRoute() == previousRoute) {
		this->trafficEngine->highlightRoute(nullptr);
	}
	this->route.reset(nullptr);
	this->trafficEngine->updateVehiclesInfoTable(this);
}

bool Vehicle::setRoute(VehiclePath *destination) {
	VehicleRoute *previousHighlightRoute = this->trafficEngine->getHighlightedRoute();
	bool isHighlightedRoute = previousHighlightRoute != nullptr && getRoute() == previousHighlightRoute;
	VehicleRoute *route = VehicleRoute::newVehicleRoute(this->trafficEngine, getPosition(), destination);
	if (route != nullptr) {
		this->trafficEngine->highlightRoute(nullptr);
		this->route.reset(route);
		this->trafficEngine->updateVehiclesInfoTable(this);
		if (isHighlightedRoute) {
			this->trafficEngine->highlightRoute(this);
		}
		return true;
	}
	return false;
}

bool Vehicle::isHighlighted() const {
	return this->highlight;
}

void Vehicle::setHighlighted(bool highlight) {
	this->highlight = highlight;
}

void Vehicle::advance(int phase) {
	if (!this->trafficEngine->isPaused()) {
		double timeElapsed = this->timer.nsecsElapsed() / 1000000000.0;
		VehiclePosition* currentPosition = getPosition();
		if (currentPosition != nullptr) {
			if (phase == 0) {
				if (!isAtPathEnd(currentPosition)) {
					prepareToMove(timeElapsed);
				} else {
					VehiclePosition *nextPosition = getPathEndNextPosition(currentPosition);
					this->temporaryNextPathPosition = nullptr;
					if (nextPosition != nullptr) {
						setPosition(nextPosition);
						prepareToMove(timeElapsed);
					} else {
						this->temporarySpeed = 0;
					}
				}
			} else {
				move();
			}
		}
	}
    if (phase == 0) {
        this->timer.restart();
	}
}

void Vehicle::mousePressEvent(QGraphicsSceneMouseEvent*) {
}

void Vehicle::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
	if (boundingRect().contains(event->pos())) {
		this->trafficEngine->setVehiclesInformationTableVisible(true);
		this->trafficEngine->highlightVehicle(this);
	}
}

bool Vehicle::isAtPathEnd(VehiclePosition *currentPosition) const {
	return currentPosition->isAtEnd();
}

VehiclePosition *Vehicle::getPathEndNextPosition(VehiclePosition *currentPosition) const {
	if (this->temporaryNextPathPosition != nullptr) {
		return this->temporaryNextPathPosition;
	} else {
		VehiclePosition *nextPosition = currentPosition->getPathEndConnection(getRoute());
		while (nextPosition != nullptr && isAtPathEnd(nextPosition)) {
			VehiclePosition *nextPosition2 = nextPosition->getPathEndConnection(getRoute());
			delete nextPosition;
			nextPosition = nextPosition2;
		}
		return nextPosition;
	}
}

void Vehicle::setTemporaryNextPathPosition() {
	VehiclePosition *nextPosition = getPathEndNextPosition(getPosition());
	if (nextPosition != nullptr && this->temporaryNextPathPosition != nextPosition) {
		if (this->temporaryNextPathPosition != nullptr) {
			delete this->temporaryNextPathPosition;
		}
		this->temporaryNextPathPosition = nextPosition;
	}
}

double Vehicle::intelligentDriverModelFunction(double x1, double v1, double x2, double v2, double acceleration, double deceleration, double preferredSpeed, double, double safetyDistance) const {
	double safetyTimeHeadway = 2;
	double delta = 4.0;
	double sfunk = safetyDistance + v1 * safetyTimeHeadway + v1 * (v1 - v2) / (2.0 * sqrt(acceleration * deceleration));
	return acceleration * (1.0 - pow(v1 / preferredSpeed, delta) - pow(sfunk / (x2 - x1), 2.0));
}
