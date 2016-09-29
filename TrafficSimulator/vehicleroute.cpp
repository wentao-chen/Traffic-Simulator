#include "vehicleroute.h"

#include "intersection.h"
#include "road.h"
#include "trafficengine.h"
#include "trafficlight.h"

#include <QSet>
#include <QVector>

VehicleRoute::VehicleRoute(const QMap<Intersection *, Direction::Cardinal> &path, Intersection *firstIntersection, VehiclePath *destination) :
	description("To " + (destination != nullptr ? destination->getName() : "")),
	path(path),
	firstIntersection(firstIntersection),
	destination(destination) {
}

VehicleRoute *VehicleRoute::newVehicleRoute(const TrafficEngine *trafficEngine, VehiclePosition *currentPosition, VehiclePath *destination) {
	Intersection *firstIntersection = getNextIntersection(trafficEngine, currentPosition);
	if (firstIntersection != nullptr) {
		VehiclePath *path = currentPosition->getPath();
		if (path == firstIntersection) {
			const Direction::Cardinal &direction = currentPosition->getDestinationDirection();
			Intersection *nextIntersection = getOtherIntersection(firstIntersection, direction);
			Road *nextPathRoad = firstIntersection->getConnection(direction);
			Intersection *fixedIntersection = nextPathRoad->getFixedIntersection();
			std::unique_ptr<QMap<Intersection*, Direction::Cardinal>> path = findDestinationFloodFill(nextIntersection, destination, fixedIntersection == firstIntersection ? nextPathRoad->getFreeIntersectionDirection() : nextPathRoad->getFixedIntersectionDirection());
			if (path.get() != nullptr) {
				QMap<Intersection*, Direction::Cardinal> finalPath(*path.get());
				finalPath.insert(firstIntersection, direction);
				return new VehicleRoute(finalPath, firstIntersection, destination);
			}
		} else {
			std::unique_ptr<QMap<Intersection*, Direction::Cardinal>> path = findDestinationFloodFill(firstIntersection, destination, currentPosition->getDestinationDirection());
			if (path.get() != nullptr) {
				const std::vector<Direction::Cardinal> &possibleDirections = firstIntersection->getTrafficLight()->getPossibleDirections(IntersectionLane(currentPosition->getDestinationDirection(), currentPosition->getNextPathEnterLane()));
				if (path->contains(firstIntersection) && std::find(possibleDirections.begin(), possibleDirections.end(), path->value(firstIntersection)) != possibleDirections.end()) {
					return new VehicleRoute(QMap<Intersection*, Direction::Cardinal>(*path.get()), firstIntersection, destination);
				} else if (possibleDirections.size() > 0) {
					std::unique_ptr<QMap<Intersection*, Direction::Cardinal>> shortest;
					for (auto &d : possibleDirections) {
						Road *road = firstIntersection->getConnection(d);
						if (road != nullptr) {
							bool isFixedIntersection = road->getFixedIntersection() == firstIntersection;
							Intersection *otherIntersection = isFixedIntersection ? road->getFreeIntersection() : road->getFixedIntersection();
							const Direction::Cardinal &sourceDirection = isFixedIntersection ? road->getFreeIntersectionDirection() : road->getFixedIntersectionDirection();
							std::unique_ptr<QMap<Intersection*, Direction::Cardinal>> path = findDestinationFloodFill(otherIntersection, destination, sourceDirection);
							if (path.get() != nullptr) {
								path->insert(firstIntersection, d);
								if (shortest.get() == nullptr || path->size() < shortest->size()) {
									shortest = std::move(path);
								}
							}
						}
					}
					if (shortest.get() != nullptr) {
						return new VehicleRoute(QMap<Intersection*, Direction::Cardinal>(*shortest.get()), firstIntersection, destination);
					}
				}
			}
		}
	}
	return nullptr;
}

std::string VehicleRoute::getDescription() const {
	return this->description;
}

void VehicleRoute::setDescription(const std::string &description) {
	this->description = description;
}

bool VehicleRoute::hasIntersection(Intersection *intersection) const {
	return this->path.contains(intersection);
}

Direction::Cardinal VehicleRoute::getDirection(Intersection *intersection, Direction::Cardinal defaultDirection) const {
	return this->path.value(intersection, defaultDirection);
}

void VehicleRoute::addIntersection(Intersection *intersection, Direction::Cardinal destinationDirection) {
	this->path.insert(intersection, destinationDirection);
}

bool VehicleRoute::updateSource(VehiclePosition *currentPosition) {
	const VehiclePath *currentPath = currentPosition->getPath();
	const VehiclePath *nextPath = currentPosition->getNextPath();
	const QVector<Intersection *> &intersections = getIntersections();
	for (int i = 0; i < intersections.size(); i++) {
		Intersection *inter = intersections.at(i);
		if (inter == currentPath || inter == nextPath) {
			for (int j = 0; j < i; j++) {
				this->path.remove(intersections.at(j));
			}
			this->firstIntersection = inter;
			return true;
		}
	}
	return false;
}

VehiclePath *VehicleRoute::getDestination() const {
	return this->destination;
}

bool VehicleRoute::isValid(VehiclePosition *currentPosition) const {
	if (currentPosition != nullptr) {
		VehiclePath *currentPath = currentPosition->getPath();
		VehiclePath *nextPath = currentPosition->getNextPath();
		const QList<Intersection*> &keys = this->path.keys();
		bool valid = false;
		for (auto &i : keys) {
			if (i != nullptr && (currentPath == i || nextPath == i)) {
				valid = true;
				break;
			}
		}
		if (!valid) {
			return false;
		}
	}
	Intersection *current = this->firstIntersection;
	while (current != nullptr) {
		if (!this->path.contains(current)) {
			return false;
		} else {
			const Direction::Cardinal &direction = this->path.value(current, Direction::Cardinal::NORTH);
			Road *connection = current->getConnection(direction);
			if (current == this->destination || connection == this->destination) {
				return true;
			}
			current = getOtherIntersection(current, direction);
		}
	}
	return false;
}

bool VehicleRoute::isOnRoute(VehiclePosition *position) const {
	VehiclePath *path = position->getPath();
	VehiclePath *nextPath = position->getNextPath();
	if (path == nullptr) {
		return false;
	} else if (path == this->firstIntersection) {
		return true;
	} else if (path == this->destination) {
		return true;
	} else if (nextPath != nullptr && nextPath == this->firstIntersection) {
		return true;
	} else {
		const QList<Intersection *> &keys = this->path.keys();
		for (int i = 0; i < keys.size(); i++) {
			Intersection *k = keys.at(i);
			if (k == path || (this->path.contains(k) && k->getConnection(this->path.value(k, Direction::Cardinal::NORTH)) == path)) {
				return true;
			}
		}
	}
	return false;
}

QVector<Intersection *> VehicleRoute::getIntersections() const {
	QVector<Intersection *> path;
	Intersection *currentIntersection = this->firstIntersection;
	while (currentIntersection != nullptr && currentIntersection != this->destination && this->path.contains(currentIntersection) && !path.contains(currentIntersection)) {
		path.push_back(currentIntersection);
		const Direction::Cardinal &direction = this->path.value(currentIntersection, Direction::Cardinal::NORTH);
		currentIntersection = getOtherIntersection(currentIntersection, direction);
	}
	return path;
}

QVector<Direction::Cardinal> VehicleRoute::getDirections() const {
	QVector<Direction::Cardinal> path;
	Intersection *currentIntersection = this->firstIntersection;
	while (currentIntersection != nullptr && currentIntersection != this->destination && this->path.contains(currentIntersection)) {
		const Direction::Cardinal &direction = this->path.value(currentIntersection, Direction::Cardinal::NORTH);
		path.push_back(direction);
		currentIntersection = getOtherIntersection(currentIntersection, direction);
	}
	return path;
}

Direction::Cardinal VehicleRoute::getDirectionOfIntersection(Intersection *source, Intersection *directedIntersection, const Direction::Cardinal &defaultDirection) {
	for (int i = 0; i < Direction::DIRECTIONS_COUNT; i++) {
		const Direction::Cardinal &direction = Direction::getCardinal(i);
		if (getOtherIntersection(source, direction) == directedIntersection) {
			return direction;
		}
	}
	return defaultDirection;
}

Intersection *VehicleRoute::getNextIntersection(const TrafficEngine *trafficEngine, VehiclePosition *position) {
	VehiclePath *currentPath = position->getPath();
	VehiclePath *nextPath = position->getNextPath();
	const QVector<Intersection *> allIntersections = trafficEngine->getIntersections();
	for (int i = 0; i < allIntersections.size(); i++) {
		Intersection *intersection = allIntersections.at(i);
		if (intersection == currentPath || intersection == nextPath) {
			return intersection;
		}
	}
	return nullptr;
}

std::unique_ptr<QMap<Intersection*, Direction::Cardinal>> VehicleRoute::findDestinationFloodFill(Intersection *start, VehiclePath *finish, const Direction::Cardinal &startSourceDirection) {
	if (start == finish) {
		return std::unique_ptr<QMap<Intersection*, Direction::Cardinal>>(new QMap<Intersection*, Direction::Cardinal>());
	} else {
		QMap<Intersection*, Direction::Cardinal> visited;
		visited.insert(start, startSourceDirection);
		Intersection *lastIntersection = findDestinationFloodFillRecursive(start, finish, visited, QMap<Intersection *, Direction::Cardinal>(visited));
		if (lastIntersection != nullptr) {
			std::unique_ptr<QMap<Intersection*, Direction::Cardinal>> path(new QMap<Intersection*, Direction::Cardinal>());
			while (lastIntersection != nullptr) {
				if (!visited.contains(lastIntersection)) {
					return std::unique_ptr<QMap<Intersection*, Direction::Cardinal>>(nullptr);
				} else {
					const Direction::Cardinal &direction = visited.value(lastIntersection, Direction::Cardinal::NORTH);
					Intersection *nextIntersection = getOtherIntersection(lastIntersection, direction);
					if (nextIntersection == nullptr) {
						return std::unique_ptr<QMap<Intersection*, Direction::Cardinal>>(nullptr);
					}
					path->insert(nextIntersection, getDirectionOfIntersection(nextIntersection, lastIntersection, direction));
					lastIntersection = nextIntersection;
					if (lastIntersection == start) {
						return path;
					}
				}
			}
		}
		return std::unique_ptr<QMap<Intersection*, Direction::Cardinal>>(nullptr);
	}
}

Intersection* VehicleRoute::findDestinationFloodFillRecursive(Intersection *start, VehiclePath *finish, QMap<Intersection *, Direction::Cardinal> &visited, const QMap<Intersection *, Direction::Cardinal> &justAdded) {
	 QMap<Intersection *, Direction::Cardinal> outsideEdges;
	 for (auto &intersection : justAdded.keys()) {
		 for (int i = 0; i < Direction::DIRECTIONS_COUNT; i++) {
			 const Direction::Cardinal &direction = Direction::getCardinal(i);
			 const Direction::Cardinal &sourceDirection = justAdded.value(intersection, direction);
			 if (intersection->getTrafficLight()->hasPath(sourceDirection, direction) && (sourceDirection != direction || intersection->getConnectionsCount() == 1)) {
				 Intersection *connection = getOtherIntersection(intersection, direction);
				 if (connection != nullptr) {
					 if (connection == finish || intersection->getConnection(direction) == finish) {
						 visited.insert(connection, getDirectionOfIntersection(connection, intersection, direction));
						 return connection;
					 } else if (!visited.contains(connection)) {
						outsideEdges.insert(connection, getDirectionOfIntersection(connection, intersection, direction));
					 }
				 }
			 }
		 }
	 }
	 if (outsideEdges.size() == 0) {
		 return nullptr;
	 } else {
		 return findDestinationFloodFillRecursive(start, finish, visited.unite(outsideEdges), outsideEdges);
	 }
}

Intersection *VehicleRoute::getOtherIntersection(Intersection *intersection, const Direction::Cardinal &cardinal) {
	Road *road = intersection->getConnection(cardinal);
	if (road != nullptr) {
		Intersection *fixedIntersection = road->getFixedIntersection();
		return fixedIntersection == intersection ? road->getFreeIntersection() : fixedIntersection;
	}
	return nullptr;
}
