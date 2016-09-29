#include "intersection.h"

#include "constants.h"
#include "road.h"
#include "linearroad.h"
#include "trafficengine.h"
#include "trafficlight.h"
#include "vehicle.h"

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QPointF>
#include <QRectF>

#include <assert.h>

unsigned int Intersection::idAssigner = 0;

Intersection::Intersection(TrafficEngine *trafficEngine, int horizontalLanes, int verticalLanes, double speedLimit, QGraphicsItem *parent) :
		QGraphicsItem(parent),
		VehiclePath("Intersection " + std::to_string(idAssigner), speedLimit),
		ID(idAssigner++),
		highlightedState(0),
		highlightSourceDirection(Direction::Cardinal::NORTH),
		highlightDestinationDirection(Direction::Cardinal::NORTH),
		flashingColor(QColor()),
		trafficEngine(trafficEngine),
		angle(0),
		horizontalLanes(horizontalLanes),
		verticalLanes(verticalLanes),
		roadConnections({nullptr, nullptr, nullptr, nullptr}),
		temporaryLanes({-1, -1}),
		temporaryHorizontalLanes(-1),
		temporaryVerticalLanes(-1),
		temporaryRotateAngle(M_PI + 1),
		temporaryRoadExtension(0),
		hoverLocation(Intersection::HoverLocation::OUTSIDE),
		temporaryRoad(nullptr),
		temporaryHoverIntersection(nullptr),
		trafficLight(std::unique_ptr<TrafficLight>(new TrafficLight(this))) {
    setFlag(ItemIsMovable);
    setAcceptHoverEvents(true);
	setRotation(this->angle / M_PI * 180);
	this->trafficLight->getSpeedLimitTextBox()->setValue(speedLimit);
}

Intersection::~Intersection() {
	this->trafficEngine->removeVehiclesOnPath(this);
}

unsigned int Intersection::getID() const {
    return this->ID;
}

TrafficEngine *Intersection::getTrafficEngine() {
    return this->trafficEngine;
}

QRectF Intersection::boundingRect() const {
    return QRectF(-Sizes::LANE_WIDTH / 2, -Sizes::LANE_WIDTH / 2 - getRectWidth(), Sizes::LANE_WIDTH * 2 + getRectWidth(), Sizes::LANE_WIDTH * 2 + getRectHeight() + getRectWidth() * 2);
}

QPainterPath Intersection::shape() const {
    QPainterPath path;
    path.setFillRule(Qt::WindingFill);
    path.addRect(QRectF(0, 0, getRectWidth(), getRectHeight()));
    path.addEllipse(getRectWidth() - Sizes::LANE_WIDTH / 4, -Sizes::LANE_WIDTH / 4, Sizes::LANE_WIDTH / 2, Sizes::LANE_WIDTH / 2);
    path.addEllipse(-Sizes::LANE_WIDTH / 4, getRectHeight() - Sizes::LANE_WIDTH / 4, Sizes::LANE_WIDTH / 2, Sizes::LANE_WIDTH / 2);
    path.addEllipse(getRectWidth() - Sizes::LANE_WIDTH / 4, getRectHeight() - Sizes::LANE_WIDTH / 4, Sizes::LANE_WIDTH / 2, Sizes::LANE_WIDTH / 2);
    return path;
}

int Intersection::getRectWidth() const {
    return getTotalLanesWidth(Orientation::HORIZONTAL);
}

int Intersection::getRectHeight() const {
    return getTotalLanesWidth(Orientation::VERTICAL);
}

void Intersection::paint(QPainter *painter, const QStyleOptionGraphicsItem*, QWidget*) {
    QRectF rec = QRectF(0, 0, getTotalLanesWidth(Orientation::HORIZONTAL), getTotalLanesWidth(Orientation::VERTICAL));

	const int highlightedState = this->highlightedState;
	if (highlightedState > 0) {
		painter->fillRect(rec, Qt::red);
	} else if (highlightedState < 0) {
		QVector<QPointF> focusPoints;
		Road* northConnection = getConnection(Direction::Cardinal::NORTH);
		bool isNorthOneWay = northConnection == nullptr || northConnection->getDirectionDividerLane() == 0 || std::abs(northConnection->getDirectionDividerLane()) >= northConnection->getLanes();
		if (!isNorthOneWay) {
			int lanes = northConnection->getLanes();
			int divider = northConnection->getDirectionDividerLane();
			focusPoints.push_back(QPointF(((northConnection->getFixedIntersection() == this ? lanes - divider : lanes + divider) % lanes) * Sizes::LANE_WIDTH, 0));
		} else {
			focusPoints.push_back(QPointF(rec.width() / 2, 0));
		}
		Road* eastConnection = getConnection(Direction::Cardinal::EAST);
		bool isEastOneWay = eastConnection == nullptr || eastConnection->getDirectionDividerLane() == 0 || std::abs(eastConnection->getDirectionDividerLane()) >= eastConnection->getLanes();
		if (!isEastOneWay) {
			int lanes = eastConnection->getLanes();
			int divider = eastConnection->getDirectionDividerLane();
			focusPoints.push_back(QPointF(rec.width(), ((eastConnection->getFixedIntersection() == this ? lanes - divider : lanes + divider) % lanes) * Sizes::LANE_WIDTH));
		} else {
			focusPoints.push_back(QPointF(rec.width(), rec.height() / 2));
		}
		Road* southConnection = getConnection(Direction::Cardinal::SOUTH);
		bool isSouthOneWay = southConnection == nullptr || southConnection->getDirectionDividerLane() == 0 || std::abs(southConnection->getDirectionDividerLane()) >= southConnection->getLanes();
		if (!isSouthOneWay) {
			int lanes = southConnection->getLanes();
			int divider = southConnection->getDirectionDividerLane();
			focusPoints.push_back(QPointF(((southConnection->getFixedIntersection() == this ? lanes + divider : lanes - divider) % lanes) * Sizes::LANE_WIDTH, rec.height()));
		} else {
			focusPoints.push_back(QPointF(rec.width() / 2, rec.height()));
		}
		Road* westConnection = getConnection(Direction::Cardinal::WEST);
		bool isWestOneWay = westConnection == nullptr || westConnection->getDirectionDividerLane() == 0 || std::abs(westConnection->getDirectionDividerLane()) >= westConnection->getLanes();
		if (!isWestOneWay) {
			int lanes = westConnection->getLanes();
			int divider = westConnection->getDirectionDividerLane();
			focusPoints.push_back(QPointF(0, ((westConnection->getFixedIntersection() == this ? lanes + divider : lanes - divider) % lanes) * Sizes::LANE_WIDTH));
		} else {
			focusPoints.push_back(QPointF(0, rec.height() / 2));
		}
		if (isNorthOneWay && !isSouthOneWay) {
			focusPoints.replace(0, QPointF(focusPoints[2].x(), 0));
		} else if (!isNorthOneWay && isSouthOneWay) {
			focusPoints.replace(2, QPointF(focusPoints[0].x(), rec.height()));
		}
		if (isEastOneWay && !isWestOneWay) {
			focusPoints.replace(1, QPointF(rec.width(), focusPoints[3].y()));
		} else if (!isEastOneWay && isWestOneWay) {
			focusPoints.replace(3, QPointF(0, focusPoints[1].y()));
		}
		double t = ((focusPoints[3].y() - focusPoints[1].y()) * (focusPoints[1].x() - focusPoints[0].x()) - (focusPoints[3].x() - focusPoints[1].x()) * (focusPoints[1].y() - focusPoints[0].y()))
				/ ((focusPoints[3].y() - focusPoints[1].y()) * (focusPoints[2].x() - focusPoints[0].x()) - (focusPoints[3].x() - focusPoints[1].x()) * (focusPoints[2].y() - focusPoints[0].y()));
		focusPoints.push_back(focusPoints[0] + (focusPoints[2] - focusPoints[0]) * t);
		bool drawQuad1 = false;
		bool drawQuad2 = false;
		bool drawQuad3 = false;
		bool drawQuad4 = false;
		Road* sourceRoad = getConnection(this->highlightSourceDirection);
		Road* destinationRoad = getConnection(this->highlightDestinationDirection);
		int highlightSourceSide = 0;
		int highlightDestinationSide = 0;
		if (sourceRoad != nullptr && sourceRoad->getDirectionDividerLane() != 0 && std::abs(sourceRoad->getDirectionDividerLane()) < sourceRoad->getLanes()) {
			highlightSourceSide = sourceRoad->getDirectionDividerLane() > 0 ? -1 : 1;
		}
		if (destinationRoad != nullptr && destinationRoad->getDirectionDividerLane() != 0 && std::abs(destinationRoad->getDirectionDividerLane()) < destinationRoad->getLanes()) {
			highlightDestinationSide = destinationRoad->getDirectionDividerLane() > 0 ? 1 : -1;
		}
		if ((this->highlightSourceDirection == Direction::Cardinal::NORTH && highlightSourceSide <= 0) || (this->highlightDestinationDirection == Direction::Cardinal::NORTH && highlightDestinationSide <= 0)
				|| (this->highlightSourceDirection == Direction::Cardinal::WEST && highlightSourceSide >= 0) || (this->highlightDestinationDirection == Direction::Cardinal::WEST && highlightDestinationSide >= 0)) {
			drawQuad2 = true;
		}
		if ((this->highlightSourceDirection == Direction::Cardinal::NORTH && highlightSourceSide >= 0) || (this->highlightDestinationDirection == Direction::Cardinal::NORTH && highlightDestinationSide >= 0)
				|| (this->highlightSourceDirection == Direction::Cardinal::EAST && highlightSourceSide <= 0) || (this->highlightDestinationDirection == Direction::Cardinal::EAST && highlightDestinationSide <= 0)) {
			drawQuad1 = true;
		}
		if ((this->highlightSourceDirection == Direction::Cardinal::EAST && highlightSourceSide >= 0) || (this->highlightDestinationDirection == Direction::Cardinal::EAST && highlightDestinationSide >= 0)
				|| (this->highlightSourceDirection == Direction::Cardinal::SOUTH && highlightSourceSide <= 0) || (this->highlightDestinationDirection == Direction::Cardinal::SOUTH && highlightDestinationSide <= 0)) {
			drawQuad4 = true;
		}
		if ((this->highlightSourceDirection == Direction::Cardinal::SOUTH && highlightSourceSide >= 0) || (this->highlightDestinationDirection == Direction::Cardinal::SOUTH && highlightDestinationSide >= 0)
				|| (this->highlightSourceDirection == Direction::Cardinal::WEST && highlightSourceSide <= 0) || (this->highlightDestinationDirection == Direction::Cardinal::WEST && highlightDestinationSide <= 0)) {
			drawQuad3 = true;
		}
		QPainterPath path;
		if (drawQuad1) {
			path.addPolygon(QPolygonF(QVector<QPointF> {focusPoints[0], QPointF(rec.width(), 0), focusPoints[1], focusPoints[4]}));
		}
		if (drawQuad2) {
			path.addPolygon(QPolygonF(QVector<QPointF> {focusPoints[0], QPointF(0, 0), focusPoints[3], focusPoints[4]}));
		}
		if (drawQuad3) {
			path.addPolygon(QPolygonF(QVector<QPointF> {focusPoints[2], QPointF(0, rec.height()), focusPoints[3], focusPoints[4]}));
		}
		if (drawQuad4) {
			path.addPolygon(QPolygonF(QVector<QPointF> {focusPoints[2], QPointF(rec.width(), rec.height()), focusPoints[1], focusPoints[4]}));
		}
		if (drawQuad1 && drawQuad3) {
			path.addPolygon(QPolygonF(QVector<QPointF> {focusPoints[0], focusPoints[3], focusPoints[4]}));
			path.addPolygon(QPolygonF(QVector<QPointF> {focusPoints[2], focusPoints[1], focusPoints[4]}));
		}
		if (drawQuad2 && drawQuad4) {
			path.addPolygon(QPolygonF(QVector<QPointF> {focusPoints[0], focusPoints[1], focusPoints[4]}));
			path.addPolygon(QPolygonF(QVector<QPointF> {focusPoints[2], focusPoints[3], focusPoints[4]}));
		}
		painter->fillPath(path, Qt::red);
	}
	painter->drawRect(rec);

    if (this->hoverLocation != Intersection::HoverLocation::OUTSIDE) {
		painter->setPen(QPen(Qt::red));

        if (this->hoverLocation == Intersection::HoverLocation::LANES_HORIZONTAL) {
			painter->setBrush(Qt::blue);
            painter->drawEllipse(getRectWidth() - Sizes::LANE_WIDTH / 4, -Sizes::LANE_WIDTH / 4, Sizes::LANE_WIDTH / 2, Sizes::LANE_WIDTH / 2);
        } else if (this->hoverLocation == Intersection::HoverLocation::LANES_VERTICAL) {
			painter->setBrush(Qt::blue);
            painter->drawEllipse(-Sizes::LANE_WIDTH / 4, getRectHeight() - Sizes::LANE_WIDTH / 4, Sizes::LANE_WIDTH / 2, Sizes::LANE_WIDTH / 2);
        } else if (this->hoverLocation == Intersection::HoverLocation::ROTATE) {
			painter->setBrush(Qt::blue);
            painter->drawEllipse(getRectWidth() - Sizes::LANE_WIDTH / 4, getRectHeight() - Sizes::LANE_WIDTH / 4, Sizes::LANE_WIDTH / 2, Sizes::LANE_WIDTH / 2);
        } else if (this->hoverLocation == Intersection::HoverLocation::CONNECTION_NORTH) {
            if (this->temporaryRoadExtension != 1) {
				painter->fillRect(0, 0, getRectWidth(), Sizes::LANE_WIDTH / 3, Qt::blue);
            }
        } else if (this->hoverLocation == Intersection::HoverLocation::CONNECTION_EAST) {
            if (this->temporaryRoadExtension != 2) {
				painter->fillRect(getRectWidth() - Sizes::LANE_WIDTH / 3, 0, Sizes::LANE_WIDTH / 3, getRectHeight(), Qt::blue);
            }
        } else if (this->hoverLocation == Intersection::HoverLocation::CONNECTION_SOUTH) {
            if (this->temporaryRoadExtension != 3) {
				painter->fillRect(0, getRectHeight() - Sizes::LANE_WIDTH / 3, getRectWidth(), Sizes::LANE_WIDTH / 3, Qt::blue);
            }
        } else if (this->hoverLocation == Intersection::HoverLocation::CONNECTION_WEST) {
            if (this->temporaryRoadExtension != 4) {
				painter->fillRect(0, 0, Sizes::LANE_WIDTH / 3, getRectHeight(), Qt::blue);
            }
		}
    }
	painter->setBrush(Qt::NoBrush);

    // Draw traffic light paths
	const TrafficLight::TrafficLightState &trafficLightState = this->trafficLight->getCurrentState();
    for (int i = 0; i < getLanes(Direction::Cardinal::SOUTH); i++) {
        for (int j = 0; j < getLanes(Direction::Cardinal::NORTH); j++) {
            TrafficLightColor trafficLightColor1 = trafficLightState.getLightColor(Direction::Cardinal::SOUTH, i, Direction::Cardinal::NORTH, j);
            TrafficLightColor trafficLightColor2 = trafficLightState.getLightColor(Direction::Cardinal::NORTH, j, Direction::Cardinal::SOUTH, i);
            if (trafficLightColor1 == TrafficLightColor::GREEN || trafficLightColor2 == TrafficLightColor::GREEN) {
                painter->setPen(QPen(Qt::green));
            } else if (trafficLightColor1 == TrafficLightColor::YELLOW || trafficLightColor2 == TrafficLightColor::YELLOW) {
                painter->setPen(QPen(Qt::yellow));
            } else if (trafficLightColor1 == TrafficLightColor::RED || trafficLightColor2 == TrafficLightColor::RED) {
                painter->setPen(QPen(Qt::red));
            }
            if (trafficLightColor1 != TrafficLightColor::ILLEGAL || trafficLightColor2 != TrafficLightColor::ILLEGAL) {
                painter->drawLine((getLanes(Direction::Cardinal::SOUTH) - i - 0.5) * Sizes::LANE_WIDTH, getLanes(Orientation::VERTICAL) * Sizes::LANE_WIDTH, (j + 0.5) * Sizes::LANE_WIDTH, 0);
            }
        }
    }
    for (int i = 0; i < getLanes(Direction::Cardinal::EAST); i++) {
        for (int j = 0; j < getLanes(Direction::Cardinal::WEST); j++) {
            TrafficLightColor trafficLightColor1 = trafficLightState.getLightColor(Direction::Cardinal::EAST, i, Direction::Cardinal::WEST, j);
            TrafficLightColor trafficLightColor2 = trafficLightState.getLightColor(Direction::Cardinal::WEST, j, Direction::Cardinal::EAST, i);
            if (trafficLightColor1 == TrafficLightColor::GREEN || trafficLightColor2 == TrafficLightColor::GREEN) {
                painter->setPen(QPen(Qt::green));
            } else if (trafficLightColor1 == TrafficLightColor::YELLOW || trafficLightColor2 == TrafficLightColor::YELLOW) {
                painter->setPen(QPen(Qt::yellow));
            } else if (trafficLightColor1 == TrafficLightColor::RED || trafficLightColor2 == TrafficLightColor::RED) {
                painter->setPen(QPen(Qt::red));
            }
            if (trafficLightColor1 != TrafficLightColor::ILLEGAL || trafficLightColor2 != TrafficLightColor::ILLEGAL) {
                painter->drawLine(getLanes(Orientation::HORIZONTAL) * Sizes::LANE_WIDTH, (i + 0.5) * Sizes::LANE_WIDTH, 0, (getLanes(Direction::Cardinal::WEST) - j - 0.5) * Sizes::LANE_WIDTH);
            }
        }
    }
    for (int i = 0; i < getLanes(Direction::Cardinal::SOUTH); i++) {
        for (int j = 0; j < getLanes(Direction::Cardinal::WEST); j++) {
            TrafficLightColor trafficLightColor1 = trafficLightState.getLightColor(Direction::Cardinal::SOUTH, i, Direction::Cardinal::WEST, j);
            TrafficLightColor trafficLightColor2 = trafficLightState.getLightColor(Direction::Cardinal::WEST, j, Direction::Cardinal::SOUTH, i);
            if (trafficLightColor1 == TrafficLightColor::GREEN || trafficLightColor2 == TrafficLightColor::GREEN) {
                painter->setPen(QPen(Qt::green));
            } else if (trafficLightColor1 == TrafficLightColor::YELLOW || trafficLightColor2 == TrafficLightColor::YELLOW) {
                painter->setPen(QPen(Qt::yellow));
            } else if (trafficLightColor1 == TrafficLightColor::RED || trafficLightColor2 == TrafficLightColor::RED) {
                painter->setPen(QPen(Qt::red));
            }
            if (trafficLightColor1 != TrafficLightColor::ILLEGAL || trafficLightColor2 != TrafficLightColor::ILLEGAL) {
                painter->drawArc((i + 0.5 - getLanes(Direction::Cardinal::SOUTH)) * Sizes::LANE_WIDTH, (getLanes(Direction::Cardinal::WEST) - j - 0.5) * Sizes::LANE_WIDTH, (getLanes(Direction::Cardinal::SOUTH) - i - 0.5) * Sizes::LANE_WIDTH * 2, (j + 0.5) * Sizes::LANE_WIDTH * 2, 0, 1440);
            }
        }
    }
    for (int i = 0; i < getLanes(Direction::Cardinal::SOUTH); i++) {
        for (int j = 0; j < getLanes(Direction::Cardinal::EAST); j++) {
            TrafficLightColor trafficLightColor1 = trafficLightState.getLightColor(Direction::Cardinal::SOUTH, i, Direction::Cardinal::EAST, j);
            TrafficLightColor trafficLightColor2 = trafficLightState.getLightColor(Direction::Cardinal::EAST, j, Direction::Cardinal::SOUTH, i);
            if (trafficLightColor1 == TrafficLightColor::GREEN || trafficLightColor2 == TrafficLightColor::GREEN) {
                painter->setPen(QPen(Qt::green));
            } else if (trafficLightColor1 == TrafficLightColor::YELLOW || trafficLightColor2 == TrafficLightColor::YELLOW) {
                painter->setPen(QPen(Qt::yellow));
            } else if (trafficLightColor1 == TrafficLightColor::RED || trafficLightColor2 == TrafficLightColor::RED) {
                painter->setPen(QPen(Qt::red));
            }
            if (trafficLightColor1 != TrafficLightColor::ILLEGAL || trafficLightColor2 != TrafficLightColor::ILLEGAL) {
                painter->drawArc((getLanes(Direction::Cardinal::SOUTH) - i - 0.5) * Sizes::LANE_WIDTH, (j + 0.5) * Sizes::LANE_WIDTH, (i + 0.5) * Sizes::LANE_WIDTH * 2, (getLanes(Direction::Cardinal::EAST) - j - 0.5) * Sizes::LANE_WIDTH * 2, 1440, 1440);
            }
        }
    }
    for (int i = 0; i < getLanes(Direction::Cardinal::NORTH); i++) {
        for (int j = 0; j < getLanes(Direction::Cardinal::WEST); j++) {
            TrafficLightColor trafficLightColor1 = trafficLightState.getLightColor(Direction::Cardinal::NORTH, i, Direction::Cardinal::WEST, j);
            TrafficLightColor trafficLightColor2 = trafficLightState.getLightColor(Direction::Cardinal::WEST, j, Direction::Cardinal::NORTH, i);
            if (trafficLightColor1 == TrafficLightColor::GREEN || trafficLightColor2 == TrafficLightColor::GREEN) {
                painter->setPen(QPen(Qt::green));
            } else if (trafficLightColor1 == TrafficLightColor::YELLOW || trafficLightColor2 == TrafficLightColor::YELLOW) {
                painter->setPen(QPen(Qt::yellow));
            } else if (trafficLightColor1 == TrafficLightColor::RED || trafficLightColor2 == TrafficLightColor::RED) {
                painter->setPen(QPen(Qt::red));
            }
            if (trafficLightColor1 != TrafficLightColor::ILLEGAL || trafficLightColor2 != TrafficLightColor::ILLEGAL) {
                painter->drawArc((-i - 0.5) * Sizes::LANE_WIDTH, (j + 0.5 - getLanes(Direction::Cardinal::WEST)) * Sizes::LANE_WIDTH, (i + 0.5) * Sizes::LANE_WIDTH * 2, (getLanes(Direction::Cardinal::WEST) - j - 0.5) * Sizes::LANE_WIDTH * 2, 4320, 1440);
            }
        }
    }
    for (int i = 0; i < getLanes(Direction::Cardinal::NORTH); i++) {
        for (int j = 0; j < getLanes(Direction::Cardinal::EAST); j++) {
            TrafficLightColor trafficLightColor1 = trafficLightState.getLightColor(Direction::Cardinal::NORTH, i, Direction::Cardinal::EAST, j);
            TrafficLightColor trafficLightColor2 = trafficLightState.getLightColor(Direction::Cardinal::EAST, j, Direction::Cardinal::NORTH, i);
            if (trafficLightColor1 == TrafficLightColor::GREEN || trafficLightColor2 == TrafficLightColor::GREEN) {
                painter->setPen(QPen(Qt::green));
            } else if (trafficLightColor1 == TrafficLightColor::YELLOW || trafficLightColor2 == TrafficLightColor::YELLOW) {
                painter->setPen(QPen(Qt::yellow));
            } else if (trafficLightColor1 == TrafficLightColor::RED || trafficLightColor2 == TrafficLightColor::RED) {
                painter->setPen(QPen(Qt::red));
            }
            if (trafficLightColor1 != TrafficLightColor::ILLEGAL || trafficLightColor2 != TrafficLightColor::ILLEGAL) {
                painter->drawArc((i + 0.5) * Sizes::LANE_WIDTH, (-j - 0.5) * Sizes::LANE_WIDTH, (getLanes(Direction::Cardinal::NORTH) - i - 0.5) * Sizes::LANE_WIDTH * 2, (j + 0.5) * Sizes::LANE_WIDTH * 2, 2880, 1440);
            }
        }
	}
	if (this->flashingColor.isValid() && std::fmod(this->trafficLight->getTimerSecElapsed(false), FrameTime::INTERSECTION_FLASH_PERIOD) < FrameTime::INTERSECTION_FLASH_DURATION) {
		painter->fillRect(rec, this->flashingColor);
	}
}

bool Intersection::isHighlighted() const {
	return this->highlightedState != 0;
}

void Intersection::setHighlighted(bool isHighlighted) {
	this->highlightedState = isHighlighted ? 1 : 0;
	update();
}

void Intersection::setHighlighted(const Direction::Cardinal &source, const Direction::Cardinal &destination) {
	this->highlightedState = -1;
	this->highlightSourceDirection = source;
	this->highlightDestinationDirection = destination;
	update();
}

bool Intersection::isFlashing() const {
	return this->flashingColor.isValid();
}

void Intersection::setFlashing(const QColor &color) {
	this->flashingColor = color;
}

void Intersection::stopFlashing() {
	setFlashing(QColor());
}

float Intersection::getAngle() const {
    return this->angle;
}

int Intersection::getLanes(const Orientation &orientation) const {
    return orientation == Orientation::HORIZONTAL ? this->horizontalLanes : this->verticalLanes;
}

int Intersection::getLanes(const Direction::Cardinal &cardinal) const {
    return getLanes(Direction::getOrientation(cardinal));
}

void Intersection::setLanes(const Orientation &orientation, int lanes) {
    if (lanes > 0) {
        if (orientation == Orientation::HORIZONTAL) {
            this->horizontalLanes = lanes;
        } else {
            this->verticalLanes = lanes;
        }
        prepareGeometryChange();
    }
}

float Intersection::getTotalLanesWidth(const Orientation &orientation) const {
	return Sizes::LANE_WIDTH * getLanes(orientation);
}

void Intersection::setSpeedLimit(double speedLimit) {
	setSpeedLimit(speedLimit, true);
}

void Intersection::setSpeedLimit(double speedLimit, bool updateGUI) {
	VehiclePath::setSpeedLimit(speedLimit);
	if (updateGUI) {
		this->trafficEngine->updateIntersectionsInfoTable(this);
		TrafficLight* trafficLight = getTrafficLight();
		if (trafficLight != nullptr) {
			TrafficLight::SpeedLimitTextBox *textBox = trafficLight->getSpeedLimitTextBox();
			if (textBox != nullptr) {
				textBox->setValue(speedLimit);
			}
		}
	}
}

QVector<Road *> Intersection::getConnections() const {
	QVector<Road *> roads;
	for (int i = 0; i < Direction::DIRECTIONS_COUNT; i++) {
		Road *road = getConnection(Direction::getCardinal(i));
		if (road != nullptr) {
			roads.push_back(road);
		}
	}
	return roads;
}

unsigned int Intersection::getConnectionsCount() const {
	unsigned int count = 0;
	for (int i = 0; i < Direction::DIRECTIONS_COUNT; i++) {
		if (getConnection(Direction::getCardinal(i)) != nullptr) {
			count++;
		}
	}
	return count;
}

bool Intersection::hasLaneEntering(const Direction::Cardinal &cardinal) const {
	Road *road = getConnection(cardinal);
	if (road != nullptr) {
		const std::array<int, 2> &lanes = road->getExitingLanesRange(road->getFixedIntersection() == this);
		return lanes[0] <= lanes[1];
	}
	return false;
}

bool Intersection::hasLaneExiting(const Direction::Cardinal &cardinal) const {
	Road *road = getConnection(cardinal);
	if (road != nullptr) {
		const std::array<int, 2> &lanes = road->getEnteringLanesRange(road->getFixedIntersection() == this);
		return lanes[0] <= lanes[1];
	}
	return false;
}

Road *Intersection::getConnection(const Direction::Cardinal &c) const {
	return this->roadConnections[Direction::intValue(c)];
}

void Intersection::setConnection(const Direction::Cardinal &c, Road *connection) {
	this->roadConnections[Direction::intValue(c)] = connection;
	this->trafficEngine->updateIntersectionsInfoTable(this);
	this->trafficLight->getLightControlComboBox()->updateOptions();
}

float Intersection::getConnectionDirection(Direction::Cardinal c) const {
    if (c == Direction::Cardinal::NORTH) {
        return fmodf(this->angle + M_PI / 2, 2 * M_PI);
    } else if (c == Direction::Cardinal::EAST) {
        return fmodf(this->angle + M_PI, 2 * M_PI);
    } else if (c == Direction::Cardinal::SOUTH) {
        return fmodf(this->angle + M_PI * 3 / 2, 2 * M_PI);
    } else {
        return this->angle;
    }
}

QPoint Intersection::getConnectionCenter(Direction::Cardinal c) const {
    if (c == Direction::Cardinal::NORTH) {
        return QPoint(getRectWidth() / 2, 0);
    } else if (c == Direction::Cardinal::EAST) {
        return QPoint(getRectWidth(), getRectHeight() / 2);
    } else if (c == Direction::Cardinal::SOUTH) {
        return QPoint(getRectWidth() / 2, getRectHeight());
    } else {
        return QPoint(0, getRectHeight() / 2);
    }
}

QPoint Intersection::getConnectionLeft(Direction::Cardinal c) const {
    if (c == Direction::Cardinal::NORTH) {
        return QPoint(0, 0);
    } else if (c == Direction::Cardinal::EAST) {
        return QPoint(getRectWidth(), 0);
    } else if (c == Direction::Cardinal::SOUTH) {
        return QPoint(getRectWidth(), getRectHeight());
    } else {
        return QPoint(0, getRectHeight());
    }
}

QPoint Intersection::getConnectionRight(Direction::Cardinal c) const {
    if (c == Direction::Cardinal::NORTH) {
        return QPoint(getRectWidth(), 0);
    } else if (c == Direction::Cardinal::EAST) {
        return QPoint(getRectWidth(), getRectHeight());
    } else if (c == Direction::Cardinal::SOUTH) {
        return QPoint(0, getRectHeight());
    } else {
        return QPoint(0, 0);
	}
}

IntersectionPosition *Intersection::getEndOfPathPosition(const IntersectionLane &sourceLane, const IntersectionLane &destinationLane) {
	return getNewPosition(sourceLane.getDirection(), destinationLane.getDirection(), sourceLane.getLane(), destinationLane.getLane(), 1.0, 0);
}

IntersectionPosition *Intersection::getNewPosition(const Direction::Cardinal &sourceDirection, const Direction::Cardinal &destinationDirection, int lane, int destinationLane, float t, double bufferDistance) {
	assert(!std::isnan(t));
	return new IntersectionPosition(this, sourceDirection, destinationDirection, lane, destinationLane, t, bufferDistance);
}

TrafficLight* Intersection::getTrafficLight() const {
	return this->trafficLight.get();
}

double Intersection::getPathLength(const IntersectionLane &sourceLane, const IntersectionLane &destinationLane) {
	return IntersectionPosition::calculatePathLength(this, sourceLane, destinationLane);
}

bool Intersection::isEntering(const IntersectionLane &lane) const {
	Road *road = getConnection(lane.getDirection());
	if (road != nullptr) {
		int lanesCount = road->getLanes();
		bool isFixed = road->getFixedIntersection() == this;
		const std::array<int, 2> &lanes = road->getExitingLanesRange(isFixed);
		if (isFixed) {
			return lane.getLane() >= lanes[0] && lane.getLane() <= lanes[1];
		} else {
			return lane.getLane() >= lanesCount - 1 - lanes[1] && lane.getLane() <= lanesCount - 1 - lanes[0];
		}
	} else {
		return false;
	}
}

bool Intersection::isExiting(const IntersectionLane &lane) const {
	Road *road = getConnection(lane.getDirection());
	if (road != nullptr) {
		int lanesCount = road->getLanes();
		bool isFixed = road->getFixedIntersection() == this;
		const std::array<int, 2> &lanes = road->getEnteringLanesRange(isFixed);
		if (isFixed) {
			return lane.getLane() >= lanes[0] && lane.getLane() <= lanes[1];
		} else {
			return lane.getLane() >= lanesCount - 1 - lanes[1] && lane.getLane() <= lanesCount - 1 - lanes[0];
		}
	} else {
		return false;
	}
}

void Intersection::mousePressEvent(QGraphicsSceneMouseEvent *event) {
	this->trafficEngine->intersectionClicked(this);
    qreal x = event->pos().x();
    qreal y = event->pos().y();
    HoverLocation hoverLocation = checkHoverLocation(x, y, true, -1);
	if (event->button() == Qt::LeftButton) {
		if (isChangeLanes(hoverLocation)) {
			// Changing number of lanes
			Orientation o = Direction::getOrientation(hoverLocation == HoverLocation::LANES_HORIZONTAL ? 0 : 1);
			this->temporaryLanes[Direction::intValue(o)] = getLanes(o);
		} else if (hoverLocation == HoverLocation::ROTATE) {
			this->temporaryRotateAngle = atan2(y, x);
		} else if (isRoadConnection(hoverLocation)) {
			const Direction::Cardinal &c = toCardinalDirection(hoverLocation);
			int d = intValue(c);
			this->temporaryRoadExtension = d + 1;
			if (this->roadConnections[d] != nullptr && this->roadConnections[d]->getFixedIntersection() != this) {
				if (this->temporaryRoad == this->roadConnections[d]) {
					this->temporaryRoad = nullptr;
				}
				this->roadConnections[d]->getFixedIntersection()->removeRoadConnection(this->roadConnections[d]->getFixedIntersectionDirection());
				setConnection(c, nullptr);
			}
			if (this->temporaryRoad != nullptr) {
				if (this->temporaryRoad->scene() != nullptr) {
					this->temporaryRoad->scene()->removeItem(this->temporaryRoad);
				}
				delete this->temporaryRoad;
			}

			this->temporaryRoad = new LinearRoad(this, Direction::getCardinal(d), Speeds::AVERAGE_SPEED, parentItem());
			removeRoadConnection(Direction::getCardinal(d));
			setConnection(c, this->temporaryRoad);
			scene()->addItem(this->temporaryRoad);

			const QPointF &p = mapToParent(x, y);
			this->temporaryRoad->setFreeEnd(p.x(), p.y());
			this->temporaryRoad->update();
		}
	}
    update();
	QGraphicsItem::mousePressEvent(event);
}

void Intersection::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
    for (unsigned int i = 0; i < this->roadConnections.size(); i++) {
		Road *r = this->roadConnections[i];
		if (r != nullptr) {
			r->setFreeEndAtEnd();
			r->update();
        }
    }
    for (unsigned int i = 0; i < this->temporaryLanes.size(); i++) {
        this->temporaryLanes[i] = -1;
    }
    this->temporaryRotateAngle = M_PI + 1;
    if (event->button() == Qt::LeftButton) {
        // Creating the Temporary Road
        if (this->temporaryRoad != nullptr && !this->temporaryRoad->isValid()) {
            // Creating Invalid Road -- road is removed
            this->temporaryRoadExtension = 0;
            removeTemporaryRoad();
        } else {
            // Creating Valid Road
            if (this->temporaryRoad != nullptr) {
                if (this->temporaryHoverIntersection != nullptr) {
                    // Connect temporary Road to another intersection
                    const QPointF &hoverIntersectionMouse = this->temporaryHoverIntersection->mapFromParent(mapToParent(event->pos()));
                    this->temporaryHoverIntersection->hoverMoveEvent(QPointF(hoverIntersectionMouse.x(), hoverIntersectionMouse.y()), false, -1);
					this->temporaryRoad->setFreeEnd(this->temporaryHoverIntersection, toCardinalDirection(this->temporaryHoverIntersection->hoverLocation));
                } else {
                    this->temporaryRoad->setFreeEndAtEnd();
                }
				this->temporaryRoad->setFreeEndAtEnd();
				this->temporaryRoad->update();
            }
            this->temporaryRoadExtension = 0;
            this->temporaryRoad = nullptr;
        }
    } else if (event->button() == Qt::RightButton) {
        // Removing the Temporary Road
		this->temporaryRoadExtension = 0;
		removeTemporaryRoad();
    }
	this->temporaryHoverIntersection = nullptr;
    update();
    QGraphicsItem::mouseReleaseEvent(event);
	for (unsigned int i = 0; i < this->roadConnections.size(); i++) {
		Road *r = this->roadConnections[i];
		if (r != nullptr && !r->isValid()) {
			removeRoadConnection(Direction::getCardinal(i));
        }
	}
}

void Intersection::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
    int clickX = event->buttonDownPos(Qt::LeftButton).x();
    int clickY = event->buttonDownPos(Qt::LeftButton).y();
    int x = event->pos().x();
    int y = event->pos().y();
    Intersection::HoverLocation hoverLocation = checkHoverLocation(x, y, true, -1);
    for (unsigned int i = 0; i < this->temporaryLanes.size(); i++) {
        // Changing number of lanes
        if (this->temporaryLanes[i] >= 0) {
            Orientation o = Direction::getOrientation(i);
            setLanes(o, std::max(std::max(this->temporaryLanes[i] + round((i == 0 ? (x - clickX) : (y - clickY)) / Sizes::LANE_WIDTH), getLanes(o) - 1.0), 1.0));
            update();
        }
    }
    if (this->temporaryRotateAngle <= M_PI) {
        // Rotating intersection
        this->angle = fmodf(fmodf(this->angle + atan2(y, x) - this->temporaryRotateAngle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
        setRotation(this->angle / M_PI * 180.f);
        update();
    } else if (hoverLocation == Intersection::HoverLocation::INSIDE && checkHoverLocation(clickX, clickY, true, -1) == Intersection::HoverLocation::INSIDE) {
		QGraphicsItem::mouseMoveEvent(event);
		prepareConnectionsGeometryChange();
		for (auto &v : this->trafficEngine->getVehicles(this)) {
			v->updatePosition();
		}
    }

    if (this->temporaryRoad != nullptr && this->temporaryRoad->getFreeIntersection() == nullptr) {
        // Moving the temporary road
        const QPointF &p = mapToParent(x, y);
		this->temporaryRoad->setFreeEnd(p.x(), p.y());
        this->temporaryRoad->update();
    }
	for (auto &r : this->roadConnections) {
		if (r != nullptr) {
			if (r->getFreeIntersection() != nullptr) {
				r->setFreeEndAtEnd();
			}
			r->updateSize();
        }
    }
    const QPointF &transformedMouse = mapToParent(event->pos());
    Intersection *hoverIntersection = this->trafficEngine->getIntersectionBelowMouse(transformedMouse, this);
    if (this->temporaryHoverIntersection != nullptr) {
        QPointF hoverIntersectionMouse = this->temporaryHoverIntersection->mapFromParent(transformedMouse);
        this->temporaryHoverIntersection->hoverMoveEvent(QPointF(hoverIntersectionMouse.x(), hoverIntersectionMouse.y()), false, -1);
    }
    this->temporaryHoverIntersection = nullptr;
    if (hoverIntersection != nullptr && this->temporaryRoad != nullptr) {
        const QPointF &hoverIntersectionMouse = hoverIntersection->mapFromParent(transformedMouse);
        hoverIntersection->hoverMoveEvent(QPointF(hoverIntersectionMouse.x(), hoverIntersectionMouse.y()), false, this->temporaryRoad->getLanes());
        if (isRoadConnection(hoverIntersection->hoverLocation)) {
            this->temporaryHoverIntersection = hoverIntersection;
        }
	}
}

void Intersection::hoverEnterEvent(QGraphicsSceneHoverEvent *event) {
    this->hoverLocation = checkHoverLocation(event->pos().x(), event->pos().y(), true, -1);
    update();
    QGraphicsItem::hoverEnterEvent(event);
}

void Intersection::hoverLeaveEvent(QGraphicsSceneHoverEvent *event) {
    this->hoverLocation = checkHoverLocation(event->pos().x(), event->pos().y(), true, -1);
    update();
    QGraphicsItem::hoverLeaveEvent(event);
}

void Intersection::hoverMoveEvent(QGraphicsSceneHoverEvent *event) {
    hoverMoveEvent(event->pos(), true, -1);
	QGraphicsItem::hoverMoveEvent(event);
}

void Intersection::advance(int phase) {
	if (phase == 0) {
		this->trafficLight->update();
		if (isFlashing()) {
			update();
		}
	}
}

bool Intersection::isChangeLanes(const Intersection::HoverLocation &hoverLocation) const {
    return hoverLocation == Intersection::HoverLocation::LANES_HORIZONTAL || hoverLocation == Intersection::HoverLocation::LANES_VERTICAL;
}

bool Intersection::isRoadConnection(const Intersection::HoverLocation &hoverLocation) const {
	return hoverLocation == Intersection::HoverLocation::CONNECTION_NORTH || hoverLocation == Intersection::HoverLocation::CONNECTION_EAST || hoverLocation == Intersection::HoverLocation::CONNECTION_SOUTH || hoverLocation == Intersection::HoverLocation::CONNECTION_WEST;
}

Intersection::HoverLocation Intersection::toHoverLocation(const Direction::Cardinal &c) const {
	if (c == Direction::Cardinal::EAST) {
		return Intersection::HoverLocation::CONNECTION_EAST;
	} else if (c == Direction::Cardinal::SOUTH) {
		return Intersection::HoverLocation::CONNECTION_SOUTH;
	} else if (c == Direction::Cardinal::WEST) {
		return Intersection::HoverLocation::CONNECTION_WEST;
	} else {
		return Intersection::HoverLocation::CONNECTION_NORTH;
	}
}

Direction::Cardinal Intersection::toCardinalDirection(const Intersection::HoverLocation &hoverLocation) const {
    if (hoverLocation == Intersection::HoverLocation::CONNECTION_EAST) {
        return Direction::Cardinal::EAST;
    } else if (hoverLocation == Intersection::HoverLocation::CONNECTION_SOUTH) {
        return Direction::Cardinal::SOUTH;
    } else if (hoverLocation == Intersection::HoverLocation::CONNECTION_WEST) {
        return Direction::Cardinal::WEST;
    } else {
        return Direction::Cardinal::NORTH;
	}
}

QRectF Intersection::getHoverSideRectangle(const Direction::Cardinal &c) const {
	double thickness = Sizes::LANE_WIDTH / 3;
	if (c == Direction::Cardinal::EAST) {
		return QRectF(getRectWidth() - thickness, thickness, thickness, getRectHeight() - thickness);
	} else if (c == Direction::Cardinal::SOUTH) {
		return QRectF(0, getRectHeight() - thickness, getRectWidth() - thickness, thickness);
	} else if (c == Direction::Cardinal::WEST) {
		return QRectF(0, 0, thickness, getRectHeight() - thickness);
	} else {
		return QRectF(thickness, 0, getRectWidth() - thickness, thickness);
	}
}

Intersection::HoverLocation Intersection::checkHoverLocation(int mouseX, int mouseY, bool checkTransformCircles, int connectionLanes) const {
    if (checkTransformCircles && canChangeLaneCount(Orientation::HORIZONTAL) && pow(mouseX - getRectWidth(), 2) + mouseY * mouseY < Sizes::LANE_WIDTH * Sizes::LANE_WIDTH / 16) {
        return Intersection::HoverLocation::LANES_HORIZONTAL;
    } else if (checkTransformCircles && canChangeLaneCount(Orientation::VERTICAL) && mouseX * mouseX + pow(mouseY - getRectHeight(), 2) < Sizes::LANE_WIDTH * Sizes::LANE_WIDTH / 16) {
        return Intersection::HoverLocation::LANES_VERTICAL;
    } else if (checkTransformCircles && pow(mouseX - getRectWidth(), 2) + pow(mouseY - getRectHeight(), 2) < Sizes::LANE_WIDTH * Sizes::LANE_WIDTH / 16) {
        return Intersection::HoverLocation::ROTATE;
	} else {
		for (int i = 0; i < Direction::DIRECTIONS_COUNT; i++) {
			const Direction::Cardinal &c = Direction::getCardinal(i);
			if (getHoverSideRectangle(c).contains(mouseX, mouseY) && (connectionLanes < 0 || (getConnection(c) == nullptr && getLanes(c) == connectionLanes))) {
				return toHoverLocation(c);
			}
		}
		if (mouseX >= 0 && mouseY >= 0 && mouseX < getRectWidth() && mouseY < getRectHeight()) {
			return Intersection::HoverLocation::INSIDE;
		} else {
			return Intersection::HoverLocation::OUTSIDE;
		}
	}
}

void Intersection::removeTemporaryRoad() {
    if (this->temporaryRoad != nullptr) {
        for (unsigned int i = 0; i < this->roadConnections.size(); i++) {
			if (this->temporaryRoad == this->roadConnections[i]) {
				setConnection(Direction::getCardinal(i), nullptr);
                break;
            }
        }
		this->temporaryRoad->scene()->removeItem(this->temporaryRoad);
		delete this->temporaryRoad;
        this->temporaryRoad = nullptr;
    }
}

void Intersection::hoverMoveEvent(const QPointF &mousePosition, bool checkTransformCircles, int connectionLanes) {
    this->hoverLocation = checkHoverLocation(mousePosition.x(), mousePosition.y(), checkTransformCircles, connectionLanes);
    update();
}

void Intersection::removeRoadConnection(const Direction::Cardinal &c) {
	int i = Direction::intValue(c);
	Road *r = this->roadConnections[i];
	if (r != nullptr) {
		Intersection *fixedIntersection = r->getFixedIntersection();
		if (fixedIntersection != this) {
			fixedIntersection->removeRoadConnection(r->getFixedIntersectionDirection());
		} else {
			r->clearFreeEnd();
			if (this->temporaryRoad == r) {
				this->temporaryRoad = nullptr;
			}
			r->scene()->removeItem(r);
			delete r;
			setConnection(c, nullptr);
			this->trafficEngine->updateVehicleRoutes();
		}
    }
}

bool Intersection::canChangeLaneCount(const Orientation &o) const {
    for (unsigned int i = 0; i < this->roadConnections.size(); i++) {
        if (Direction::getOrientation(i) == o && this->roadConnections[i] != nullptr) {
            return false;
        }
    }
	return true;
}

void Intersection::prepareConnectionsGeometryChange() {
	for (auto r : getConnections()) {
		r->prepareGeometryChange();
	}
}

IntersectionPosition::IntersectionPosition(Intersection *intersection, const Direction::Cardinal &sourceDirection, const Direction::Cardinal &destinationDirection, int lane, int destinationLane, float t, float bufferDistance, float errorDistance) :
        intersection(intersection),
        sourceDirection(sourceDirection),
        destinationDirection(destinationDirection),
        lane(lane),
        destinationLane(destinationLane),
        position(t == -INFINITY ? 0 : t),
		errorDistance(errorDistance),
		bufferedPathLength(calculatePathLength(intersection, IntersectionLane(sourceDirection, lane), IntersectionLane(destinationDirection, destinationLane))) {
	if (this->destinationDirection == Direction::getOpposite(this->sourceDirection)) {
        const IntersectionPosition::SinCurveParameters &p = getSinCurveParameters();
        double newPosition = p.getLengthAfterPositionNewton(this->position * p.getDomainLength(), bufferDistance, bufferDistance / 100.0);
		double actualDistance = p.arcLengthAntiDerivative(newPosition) - p.arcLengthAntiDerivative(this->position * p.getDomainLength());
        this->errorDistance += bufferDistance - actualDistance;
		this->position = newPosition / p.getDomainLength();
	} else {
		double totalTurnAngle = this->destinationDirection == this->sourceDirection ? M_PI : M_PI / 2.0;
		const IntersectionPosition::EllipseParameters &p = getEllipseParameters();
		double position = this->position * totalTurnAngle;
		double newPosition = p.getLengthAfterPositionNewton(position, bufferDistance, bufferDistance / 100.0);
		double actualDistance = p.arcLengthEstimate(position, newPosition);
        this->errorDistance += bufferDistance - actualDistance;
		this->position = newPosition / totalTurnAngle;
	}

	assert(!std::isnan(this->position));
	assert(!std::isnan(this->errorDistance));
}

IntersectionPosition::~IntersectionPosition() {
}

VehiclePath *IntersectionPosition::getPath() const {
	return this->intersection;
}

VehiclePath *IntersectionPosition::getNextPath() const {
	return this->intersection->getConnection(getDestinationDirection());
}

int IntersectionPosition::getLane(const IntersectionLane &intersectionLane) {
	return intersectionLane.getLane() * Direction::DIRECTIONS_COUNT + Direction::intValue(intersectionLane.getDirection());
}

IntersectionLane IntersectionPosition::getLane(int intersectionLane) {
	return IntersectionLane(Direction::getCardinal(intersectionLane % Direction::DIRECTIONS_COUNT), intersectionLane / Direction::DIRECTIONS_COUNT);
}

int IntersectionPosition::getLane() const {
	return IntersectionPosition::getLane(IntersectionLane(this->sourceDirection, this->lane));
}

int IntersectionPosition::getChangeToLane() const {
	return IntersectionPosition::getLane(IntersectionLane(this->destinationDirection, this->destinationLane));
}

int IntersectionPosition::getNextPathEnterLane() const {
	Road *connection = this->intersection->getConnection(this->destinationDirection);
	if (connection != nullptr) {
		if (connection->getFixedIntersection() == this->intersection) {
			return this->destinationLane;
		} else {
			return connection->getLanes() - 1 - this->destinationLane;
		}
	}
	return this->destinationLane;
}

Direction::Cardinal IntersectionPosition::getSourceDirection() const {
    return this->sourceDirection;
}

Direction::Cardinal IntersectionPosition::getDestinationDirection() const {
    return this->destinationDirection;
}

QPointF IntersectionPosition::getLocation() const {
	if (this->destinationDirection == Direction::getOpposite(this->sourceDirection)) {
		const IntersectionPosition::SinCurveParameters &p = getSinCurveParameters();
		float center = Sizes::LANE_WIDTH * this->intersection->getLanes(Direction::getOrientation(this->sourceDirection));
		float x = std::min(std::max(this->position, 0.f), 1.f) * Sizes::LANE_WIDTH * this->intersection->getLanes(Direction::getOpposite(Direction::getOrientation(this->sourceDirection)));
		float value = p.evaluate(x);
		if (this->sourceDirection == Direction::Cardinal::NORTH) {
			return this->intersection->mapToParent(QPointF(center - value, x));
		} else if (this->sourceDirection == Direction::Cardinal::EAST) {
			return this->intersection->mapToParent(QPointF(Sizes::LANE_WIDTH * this->intersection->getLanes(Orientation::HORIZONTAL) - x, center - value));
		} else if (this->sourceDirection == Direction::Cardinal::SOUTH) {
			return this->intersection->mapToParent(QPointF(value, Sizes::LANE_WIDTH * this->intersection->getLanes(Orientation::VERTICAL) - x));
		} else {
			return this->intersection->mapToParent(QPointF(x, value));
		}
	} else if (this->destinationDirection == Direction::getLeft(this->sourceDirection)) {
		const QPointF &location = getEllipseParameters().evaluate(this->position * M_PI / 2.0);
        if (this->sourceDirection == Direction::Cardinal::NORTH) {
			return this->intersection->mapToParent(QPointF(this->intersection->getLanes(this->sourceDirection) * Sizes::LANE_WIDTH - location.x(), location.y()));
        } else if (this->sourceDirection == Direction::Cardinal::EAST) {
			return this->intersection->mapToParent(QPointF(this->intersection->getLanes(this->destinationDirection) * Sizes::LANE_WIDTH - location.y(), this->intersection->getLanes(this->sourceDirection) * Sizes::LANE_WIDTH - location.x()));
        } else if (this->sourceDirection == Direction::Cardinal::SOUTH) {
			return this->intersection->mapToParent(QPointF(location.x(), this->intersection->getLanes(this->destinationDirection) * Sizes::LANE_WIDTH - location.y()));
        } else {
			return this->intersection->mapToParent(QPointF(location.y(), location.x()));
        }
	} else if (this->destinationDirection == Direction::getRight(this->sourceDirection)) {
		const QPointF &location = getEllipseParameters().evaluate(this->position * M_PI / 2.0);
        if (this->sourceDirection == Direction::Cardinal::NORTH) {
			return this->intersection->mapToParent(location);
        } else if (this->sourceDirection == Direction::Cardinal::EAST) {
			return this->intersection->mapToParent(QPointF(this->intersection->getLanes(this->destinationDirection) * Sizes::LANE_WIDTH - location.y(), location.x()));
        } else if (this->sourceDirection == Direction::Cardinal::SOUTH) {
			return this->intersection->mapToParent(QPointF(this->intersection->getLanes(this->sourceDirection) * Sizes::LANE_WIDTH - location.x(), this->intersection->getLanes(this->destinationDirection) * Sizes::LANE_WIDTH - location.y()));
        } else {
			return this->intersection->mapToParent(QPointF(location.y(), this->intersection->getLanes(this->sourceDirection) * Sizes::LANE_WIDTH - location.x()));
        }
	} else {
		const QPointF &location = getEllipseParameters().evaluate(this->position * M_PI);
		double center = (this->lane + this->destinationLane + 1) / 2.0 * Sizes::LANE_WIDTH;
        if (this->sourceDirection == Direction::Cardinal::NORTH) {
			return this->intersection->mapToParent(QPointF(center - location.x(), location.y()));
        } else if (this->sourceDirection == Direction::Cardinal::EAST) {
			return this->intersection->mapToParent(QPointF(this->intersection->getLanes(Direction::getOpposite(Direction::getOrientation(this->destinationDirection))) * Sizes::LANE_WIDTH - location.y(), center - location.x()));
        } else if (this->sourceDirection == Direction::Cardinal::SOUTH) {
			return this->intersection->mapToParent(QPointF(center + location.x(), this->intersection->getLanes(Direction::getOpposite(Direction::getOrientation(this->destinationDirection))) * Sizes::LANE_WIDTH - location.y()));
        } else {
			return this->intersection->mapToParent(QPointF(location.y(), center + location.x()));
        }
	}
}

float IntersectionPosition::getDirection() const {
	if (this->sourceDirection == Direction::getOpposite(this->destinationDirection)) {
		float y = std::min(std::max(this->position, 0.f), 1.f) * Sizes::LANE_WIDTH * this->intersection->getLanes(Direction::getOpposite(Direction::getOrientation(this->sourceDirection)));
		return this->intersection->getConnectionDirection(this->sourceDirection) + atan(getSinCurveParameters().derivative(y));
	} else {
		const QPointF &tangent = getEllipseParameters().evaluateDerivative(this->position * M_PI / (this->destinationDirection == this->sourceDirection ? 1 : 2.0));
		return this->intersection->getConnectionDirection(this->sourceDirection) + (M_PI / 2 + atan2(-tangent.y(), tangent.x())) * (this->destinationDirection == Direction::getRight(this->sourceDirection) ? -1 : 1);
	}
}

VehiclePosition *IntersectionPosition::getNextPosition(double distance) const {
	assert(!std::isnan(distance));
	distance += this->errorDistance;
	if (this->destinationDirection == Direction::getOpposite(this->sourceDirection)) {
		const IntersectionPosition::SinCurveParameters &p = getSinCurveParameters();
		double newPosition = p.getLengthAfterPositionNewton(this->position * p.getDomainLength(), distance, distance / 100.0);
		double actualDistance = p.arcLengthAntiDerivative(newPosition) - p.arcLengthAntiDerivative(this->position * p.getDomainLength());
		double t = newPosition / p.getDomainLength();
		if (t < this->position || std::isnan(t)) {
			t = this->position;
		} else if (t > 1) {
			t = 1;
		}
		return new IntersectionPosition(this->intersection, this->sourceDirection, this->destinationDirection, this->lane, this->destinationLane, t, 0, std::max(distance - actualDistance, -distance));
	} else {
		double totalTurnAngle = this->destinationDirection == this->sourceDirection ? M_PI : M_PI / 2.0;
		const IntersectionPosition::EllipseParameters &p = getEllipseParameters();
		double position = this->position * totalTurnAngle;
		float newPosition = p.getLengthAfterPositionNewton(position, distance, distance / 100.0);
		double actualDistance = p.arcLengthEstimate(position, newPosition);
		double t = newPosition / totalTurnAngle;
		if (t < this->position || std::isnan(t)) {
			t = this->position;
		} else if (t > 1) {
			t = 1;
		}
		return new IntersectionPosition(this->intersection, this->sourceDirection, this->destinationDirection, this->lane, this->destinationLane, newPosition / totalTurnAngle, 0, std::max(distance - actualDistance, -distance));
	}
}

bool IntersectionPosition::isAtEnd() const {
    return this->position >= 1;
}

VehiclePosition *IntersectionPosition::getPathEndConnection(VehicleRoute *) const {
    double extraDistance = 0;
    if (this->position >= 1) {
        const IntersectionPosition::SinCurveParameters &p = getSinCurveParameters();
        extraDistance = p.arcLengthAntiDerivative(this->position * p.getDomainLength()) - p.arcLengthAntiDerivative(p.getDomainLength()) + this->errorDistance;
    }

    Road *connectedRoad = this->intersection->getConnection(this->destinationDirection);
    if (connectedRoad != nullptr) {
        bool movingFixedToFree = connectedRoad->getFixedIntersection() == this->intersection;
        if (this->sourceDirection == this->destinationDirection) {
			return connectedRoad->getNewPosition(movingFixedToFree, movingFixedToFree ? connectedRoad->getLanes() - 1 - (connectedRoad->getLanes() - 1 - this->destinationLane) : (connectedRoad->getLanes() - 1 - this->destinationLane), -INFINITY, extraDistance);
        } else {
            return connectedRoad->getNewPosition(movingFixedToFree, movingFixedToFree ? this->destinationLane : connectedRoad->getLanes() - 1 - this->destinationLane, -INFINITY, extraDistance);
        }
    }
	return nullptr;
}

bool IntersectionPosition::requestLaneChangeLeft(double, Vehicle*) {
	return false;
}

bool IntersectionPosition::requestLaneChangeRight(double, Vehicle*) {
	return false;
}

bool IntersectionPosition::requestLaneChange(int newLane, double, Vehicle*) {
    return newLane == this->lane;
}

double IntersectionPosition::getDistanceAlongPath() const {
    return this->position * getPathLength();
}

double IntersectionPosition::getPathLength() const {
	return this->bufferedPathLength;
}

TrafficLight *IntersectionPosition::getEndOfPathTrafficLight() const {
    return nullptr;
}

VehiclePosition *IntersectionPosition::getEndOfPathTrafficLightPosition() const {
    return nullptr;
}

VehiclePosition *IntersectionPosition::clone() const {
    return this->intersection->getNewPosition(this->sourceDirection, this->destinationDirection, this->lane, this->destinationLane, this->position, 0);
}

IntersectionPosition::SinCurveParameters IntersectionPosition::getSinCurveParameters() const {
	return getSinCurveParameters(this->intersection, IntersectionLane(this->sourceDirection, this->lane), this->destinationLane);
}

IntersectionPosition::EllipseParameters IntersectionPosition::getEllipseParameters() const {
	return getEllipseParameters(this->intersection, IntersectionLane(this->sourceDirection, this->lane), IntersectionLane(this->destinationDirection, this->destinationLane));
}

double IntersectionPosition::calculatePathLength(Intersection *intersection, const IntersectionLane &sourceLane, const IntersectionLane &destinationLane) {
	assert(sourceLane.getDirection() != destinationLane.getDirection() || sourceLane.getLane() != destinationLane.getLane());
	if (destinationLane.getDirection() == Direction::getOpposite(sourceLane.getDirection())) {
		const IntersectionPosition::SinCurveParameters &p = getSinCurveParameters(intersection, sourceLane, destinationLane.getLane());
		return p.getLength() / 2.0;
	} else {
		const IntersectionPosition::EllipseParameters &p = getEllipseParameters(intersection, sourceLane, destinationLane);
		return p.arcLengthEstimate(0, destinationLane.getDirection() == sourceLane.getDirection() ? M_PI : M_PI / 2.0);
	}
}

VehiclePosition::SinCurveParameters IntersectionPosition::getSinCurveParameters(Intersection *intersection, const IntersectionLane &sourceLane, int destinationLane) {
	float length = intersection->getLanes(Direction::getOpposite(Direction::getOrientation(sourceLane.getDirection()))) * Sizes::LANE_WIDTH;
	return IntersectionPosition::SinCurveParameters(0, (intersection->getLanes(sourceLane.getDirection()) - sourceLane.getLane() - 0.5) * Sizes::LANE_WIDTH, length, (destinationLane + 0.5) * Sizes::LANE_WIDTH, length);
}

VehiclePosition::EllipseParameters IntersectionPosition::getEllipseParameters(Intersection *intersection, const IntersectionLane &sourceLane, const IntersectionLane &destinationLane) {
	if (destinationLane.getDirection() == Direction::getOpposite(sourceLane.getDirection())) {
		return IntersectionPosition::EllipseParameters(NAN, NAN);
	} else if (destinationLane.getDirection() == Direction::getLeft(sourceLane.getDirection())) {
		float aRadius = (intersection->getLanes(sourceLane.getDirection()) - sourceLane.getLane() - 0.5) * Sizes::LANE_WIDTH;
		float bRadius = (destinationLane.getLane() + 0.5) * Sizes::LANE_WIDTH;
		return IntersectionPosition::EllipseParameters(aRadius, bRadius);
	} else if (destinationLane.getDirection() == Direction::getRight(sourceLane.getDirection())) {
		float aRadius = (sourceLane.getLane() + 0.5) * Sizes::LANE_WIDTH;
		float bRadius = (intersection->getLanes(destinationLane.getDirection()) - destinationLane.getLane() - 0.5) * Sizes::LANE_WIDTH;
		return IntersectionPosition::EllipseParameters(aRadius, bRadius);
	} else {
		// double aRadius = (this->lane + this->destinationLane - this->intersection->getLanes(this->destinationDirection) + 1) * Sizes::LANE_WIDTH / 2;
		double aRadius = std::abs(sourceLane.getLane() - destinationLane.getLane()) * Sizes::LANE_WIDTH / 2.0;
		double bRadius = std::min((intersection->getLanes(Direction::getOpposite(Direction::getOrientation(sourceLane.getDirection()))) - 0.5) * Sizes::LANE_WIDTH, fabs(aRadius));
		return IntersectionPosition::EllipseParameters(aRadius, bRadius);
	}
}

IntersectionLane::IntersectionLane(Direction::Cardinal direction, int lane) : direction(direction), lane(lane) {
}

Direction::Cardinal IntersectionLane::getDirection() const {
    return this->direction;
}

int IntersectionLane::getLane() const {
    return this->lane;
}

void IntersectionLane::setDirection(Direction::Cardinal direction) {
    this->direction = direction;
}

void IntersectionLane::setLane(int lane) {
	this->lane = lane;
}

bool IntersectionLane::operator==(const IntersectionLane &lane) const {
	return this->lane == lane.lane && this->direction == lane.direction;
}

QDebug operator<<(QDebug debug, const IntersectionLane &lane) {
	debug << "IntersectionLane[D: " << lane.getDirection() << ", L:" << (std::to_string(lane.getLane()) + "]").c_str();
	return debug;
}
