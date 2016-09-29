#include "road.h"

#include "intersection.h"
#include "trafficengine.h"
#include "vehicle.h"
#include "vehicleroute.h"

Road::Road(Intersection *fixedIntersection, Direction::Cardinal fixedIntersectionDirection, double speedLimit, QGraphicsItem *parent) :
		QGraphicsPathItem(parent),
		VehiclePath("Road " + std::to_string(fixedIntersection->getID() * 10 + Direction::intValue(fixedIntersectionDirection)), speedLimit),
        freeEndX(0.f),
        freeEndY(0.f),
        fixedIntersection(fixedIntersection),
        fixedIntersectionDirection(fixedIntersectionDirection),
        freeEndIntersection(nullptr),
		freeEndIntersectionDirection(Direction::Cardinal::NORTH),
		highlightOption(Road::HighlightOption::NONE) {
}

void Road::mousePressEvent(QGraphicsSceneMouseEvent *event) {
	this->fixedIntersection->getTrafficEngine()->roadClicked(this);
	QGraphicsItem::mousePressEvent(event);
}

Road::~Road() {
}

float Road::getFreeEndX() const {
    return this->freeEndX;
}

float Road::getFreeEndY() const {
    return this->freeEndY;
}

void Road::setFreeEnd(float x, float y) {
    this->freeEndX = x;
    this->freeEndY = y;
}

void Road::setFreeEnd(Intersection *freeEndIntersection, const Direction::Cardinal &freeEndIntersectionDirection) {
	prepareGeometryChange();
    if (freeEndIntersection != nullptr) {
        if (freeEndIntersection->getConnection(freeEndIntersectionDirection) == nullptr) {
			freeEndIntersection->setConnection(freeEndIntersectionDirection, this);
            this->freeEndIntersection = freeEndIntersection;
            this->freeEndIntersectionDirection = freeEndIntersectionDirection;
        }
    } else if (this->freeEndIntersection != nullptr) {
		this->freeEndIntersection->setConnection(this->freeEndIntersectionDirection, nullptr);
        this->freeEndIntersection = nullptr;
        this->freeEndIntersectionDirection = freeEndIntersectionDirection;
    }
}

void Road::clearFreeEnd() {
    setFreeEnd(nullptr, this->freeEndIntersectionDirection);
}

void Road::setFreeEndAtEnd() {
    const QPointF &end = getPoint(1);
    if (end.x() != getFreeEndX() || end.y() != getFreeEndY()) {
        setFreeEnd(end.x(), end.y());
    }
}

Intersection *Road::getFixedIntersection() const {
    return this->fixedIntersection;
}

Direction::Cardinal Road::getFixedIntersectionDirection() const {
    return this->fixedIntersectionDirection;
}

Intersection *Road::getFreeIntersection() const {
    return this->freeEndIntersection;
}

Direction::Cardinal Road::getFreeIntersectionDirection() const {
	return this->freeEndIntersectionDirection;
}

Road::HighlightOption Road::getHighlightOption() const {
	return this->highlightOption;
}

void Road::setHighlightOption(const Road::HighlightOption &option) {
	this->highlightOption = option;
	update();
}

int Road::getLanes() const {
	return getFixedIntersection()->getLanes(getFixedIntersectionDirection());
}

VehiclePosition *Road::getEndOfPathTrafficLightPosition(bool movingFixedToFree, int lane) {
	return this->getNewPosition(movingFixedToFree, lane, getBeginningPosition(lane, !movingFixedToFree), 0);
}

int Road::getDirectionDividerLane() const {
	return (getFixedIntersection()->getTrafficEngine()->isRightHandDrive() ? 1 : -1) * (getLanes() + 1) / 2;
}

std::array<int, 2> Road::getEnteringLanesRange(bool fixedEnd) const {
	int divider = getDirectionDividerLane();
	int lanes = getLanes();
	if (fixedEnd) {
		if (divider == 0) {
			return std::array<int, 2>{lanes, -1};
		} else if (std::abs(divider) >= lanes) {
			return std::array<int, 2>{0, lanes - 1};
		} else if (divider > 0) {
			return std::array<int, 2>{lanes - divider, lanes - 1};
		} else {
			return std::array<int, 2>{0, -divider - 1};
		}
	} else {
		if (divider == 0) {
			return std::array<int, 2>{0, lanes - 1};
		} else if (std::abs(divider) >= lanes) {
			return std::array<int, 2>{lanes, -1};
		} else if (divider > 0) {
			return std::array<int, 2>{0, lanes - 1 - divider};
		} else {
			return std::array<int, 2>{-divider, lanes - 1};
		}
	}
}

std::array<int, 2> Road::getExitingLanesRange(bool fixedEnd) const {
	return getEnteringLanesRange(!fixedEnd);
}

int Road::getEnteringLanesCount(bool fixedEnd) const {
	std::array<int, 2> enterLanes = getEnteringLanesRange(fixedEnd);
	return enterLanes[0] <= enterLanes[1] ? enterLanes[1] - enterLanes[0] + 1 : 0;
}

int Road::getExitingLanesCount(bool fixedEnd) const {
	return getEnteringLanesCount(!fixedEnd);
}

bool Road::isLaneMovingToFreeEnd(int lane) const {
	std::array<int, 2> enterLanes = getEnteringLanesRange(true);
	return lane >= enterLanes[0] && lane <= enterLanes[1];
}

void Road::update() {
	QGraphicsPathItem::update();
	const QVector<Vehicle*> &vehicles = getFixedIntersection()->getTrafficEngine()->getVehicles(this);
	for (int i = 0; i < vehicles.size(); i++) {
		vehicles.at(i)->updatePosition();
	}
}
