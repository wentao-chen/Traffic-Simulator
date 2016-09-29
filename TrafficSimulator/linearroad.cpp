#include "linearroad.h"

#include "constants.h"
#include "intersection.h"
#include "trafficengine.h"
#include "trafficlight.h"
#include "vehicle.h"
#include "vehicleroute.h"

#include <QGraphicsScene>
#include <QPainter>

#include <vector>

#include <QDebug>

LinearRoad::LinearRoad(Intersection *fixedIntersection, Direction::Cardinal fixedIntersectionDirection, double speedLimit, QGraphicsItem *parent) :
		Road(fixedIntersection, fixedIntersectionDirection, speedLimit, parent), boundingRectangle(QRectF(-10, -10, 20, 20)) {
	const QPointF &p = getFixedIntersection()->mapToParent(getFixedIntersection()->getConnectionCenter(getFixedIntersectionDirection()));
    setPos(p.x(), p.y());
    QPainterPath path;
    path.addRect(1, -20, 40, 19);
	setPath(path);
	updateSize();
}

LinearRoad::~LinearRoad() {
	getFixedIntersection()->getTrafficEngine()->removeVehiclesOnPath(this);
}

QPointF LinearRoad::getPoint(float t) const {
	if (t == 0) {
		return getStart();
	} else if (t == 1) {
		return getEnd();
	} else {
		const QPointF &s = getStart();
		const QPointF &e = getEnd();
		return QPointF(s.x() + t * (e.x() - s.x()), s.y() + t * (e.y() - s.y()));
	}
}

QPointF LinearRoad::getTangent(float) const {
    const QPointF &s = getStart();
    const QPointF &e = getEnd();
    return QPointF(e.x() - s.x(), e.y() - s.y());
}

bool LinearRoad::isValid() const {
    const QPointF &start = getStart();
    const QPointF &end = getEnd();
	return !std::isnan(start.x()) && !std::isnan(start.y()) && !std::isnan(end.x()) && !std::isnan(end.y());
}

void LinearRoad::updateSize() {
	const QPointF &start = getStart();
	const QPointF &end = getEnd();
	float bufferDistance = 2 * Sizes::LANE_WIDTH * getFixedIntersectionLanes();

	float leftX = std::min(start.x(), end.x()) - bufferDistance;
	float rightX = std::max(start.x(), end.x()) + bufferDistance;
	float upY = std::min(start.y(), end.y()) - bufferDistance;
	float downY = std::max(start.y(), end.y()) + bufferDistance;
	if (std::isnan(leftX) && std::isnan(rightX)) {
		leftX = 0;
		rightX = 1;
	} else if (std::isnan(leftX)) {
		leftX = rightX - 1;
	} else if (std::isnan(rightX)) {
		rightX = leftX + 1;
	}
	if (std::isnan(upY) && std::isnan(downY)) {
		upY = 0;
		downY = 1;
	} else if (std::isnan(upY)) {
		upY = downY - 1;
	} else if (std::isnan(downY)) {
		downY = upY + 1;
	}
	const QPointF &topLeft = mapFromParent(leftX, upY);
	prepareGeometryChange();
	this->boundingRectangle = QRectF(topLeft.x(), topLeft.y(), rightX - leftX, downY - upY);
	update();
}

QRectF LinearRoad::boundingRect() const {
	return this->boundingRectangle;
}

void LinearRoad::paint(QPainter *painter, const QStyleOptionGraphicsItem*, QWidget*) {
	if (!isValid()) {
		return;
	}
    if (!scene()->collidingItems(this).isEmpty()) {
        painter->setPen(Qt::red);
    } else {
        painter->setPen(Qt::green);
    }
    const QPointF &p1 = mapFromParent(getStart());
    const QPointF &p2 = mapFromParent(getEnd());
    const QPointF &t = getTangent(0);
	int divider = getDirectionDividerLane();
	int lanes = getLanes();
    float intersectionSize = getFixedIntersectionSize();
    float tangentLength = sqrt(t.x() * t.x() + t.y() * t.y());
    if (p1 == p2 || tangentLength == 0) {
        return;
    }
	const Road::HighlightOption &highlightOption = getHighlightOption();
	QPainterPath highlightPath;

    QPen pen(QColor(0xFF, 0xA5, 0x00));
    // Drawing Arcs
    const QPointF &d = getFixedIntersection()->mapFromParent(getFreeEndX(), getFreeEndY());
    const QPointF &e = getFixedIntersection()->getConnectionLeft(getFixedIntersectionDirection());
    const QPointF &f = getFixedIntersection()->getConnectionRight(getFixedIntersectionDirection());
    float fixedEndRadius = getFixedIntersection()->getTotalLanesWidth(Direction::getOrientation(getFixedIntersectionDirection())) / 2;
    float leftPivotAngle = fmodf(fmodf(getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) - getFixedIntersection()->getAngle() - getParallelLineAngle(e, d, -fixedEndRadius, 0) + M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    float rightPivotAngle = fmodf(fmodf(getParallelLineAngle(f, d, fixedEndRadius, 0) - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) + getFixedIntersection()->getAngle() + M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
	float yDis = cos(atan2(d.y() - e.y(), d.x() - e.x()) - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) + getFixedIntersection()->getAngle()) * sqrt(pow(d.x() - e.x(), 2) + pow(d.y() - e.y(), 2));
	if (leftPivotAngle < M_PI || rightPivotAngle < M_PI || (yDis <= 0 && (std::isnan(leftPivotAngle) || std::isnan(rightPivotAngle)))) {
		if (std::isnan(leftPivotAngle)) {
			leftPivotAngle = getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) - getFixedIntersection()->getAngle() - std::atan2(d.y() - e.y(), d.x() - e.x()) - M_PI / 2.0;
		}
		if (std::isnan(rightPivotAngle)) {
			rightPivotAngle = getFixedIntersection()->getAngle() - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) + std::atan2(d.y() - f.y(), d.x() - f.x()) - M_PI / 2.0;
		}
		if (leftPivotAngle < M_PI) {
            const QPointF &p = mapFromParent(getFixedIntersection()->mapToParent(getFixedIntersection()->getConnectionLeft(getFixedIntersectionDirection())));
			float startAngle = M_PI / 2 - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection());
			for (int i = 1; i <= lanes; i++) {
				pen.setStyle(i == lanes || i == (lanes - divider) % lanes ? Qt::SolidLine : Qt::DashLine);
				painter->setPen(pen);
				float radius = i * Sizes::LANE_WIDTH;
				painter->drawArc(p.x() - radius, p.y() - radius, radius * 2, radius * 2, startAngle / M_PI * 2880, fmodf(fmodf(leftPivotAngle, 2 * M_PI) + 2 * M_PI, 2 * M_PI) / M_PI * 2880);
			}
			addHighlightPath(highlightPath, p, startAngle, fmodf(fmodf(leftPivotAngle, 2 * M_PI) + 2 * M_PI, 2 * M_PI), highlightOption, true);
		} else if (rightPivotAngle < M_PI) {
            const QPointF &p = mapFromParent(getFixedIntersection()->mapToParent(getFixedIntersection()->getConnectionRight(getFixedIntersectionDirection())));
			float startAngle = -M_PI / 2 - rightPivotAngle - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection());
			for (int i = 1; i <= lanes; i++) {
				pen.setStyle(i == lanes || i == (lanes + divider) % lanes ? Qt::SolidLine : Qt::DashLine);
				painter->setPen(pen);
				float radius = i * Sizes::LANE_WIDTH;
				painter->drawArc(p.x() - radius, p.y() - radius, radius * 2, radius * 2, startAngle / M_PI * 2880, fmodf(fmodf(rightPivotAngle, 2 * M_PI) + 2 * M_PI, 2 * M_PI) / M_PI * 2880);
			}
			addHighlightPath(highlightPath, p, startAngle, fmodf(fmodf(rightPivotAngle, 2 * M_PI) + 2 * M_PI, 2 * M_PI), highlightOption, false);
		}
    } else {
		if (leftPivotAngle < rightPivotAngle || leftPivotAngle > rightPivotAngle || leftPivotAngle == rightPivotAngle) {
			const QPointF &p = mapFromParent(getFixedIntersection()->mapToParent(leftPivotAngle <= rightPivotAngle ? getFixedIntersection()->getConnectionLeft(getFixedIntersectionDirection()) : getFixedIntersection()->getConnectionRight(getFixedIntersectionDirection())));
			for (int i = 1; i <= lanes; i++) {
				pen.setStyle(i == lanes || (leftPivotAngle <= rightPivotAngle ? lanes - divider : lanes + divider) % lanes == i ? Qt::SolidLine : Qt::DashLine);
				painter->setPen(pen);
				float radius = i * Sizes::LANE_WIDTH;
				painter->drawArc(p.x() - radius, p.y() - radius, radius * 2, radius * 2, 1440 - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) / M_PI * 2880, 2880);
			}
			addHighlightPath(highlightPath, p, M_PI / 2.0 - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()), M_PI, highlightOption, leftPivotAngle <= rightPivotAngle);
		} else {
			if (yDis > 0) {
				if (leftPivotAngle >= 0) {
                    const QPointF &p = mapFromParent(getFixedIntersection()->mapToParent(getFixedIntersection()->getConnectionRight(getFixedIntersectionDirection())));
					for (int i = 1; i <= lanes; i++) {
						pen.setStyle(i == lanes || (lanes + divider) % lanes == i ? Qt::SolidLine : Qt::DashLine);
						painter->setPen(pen);
						float radius = i * Sizes::LANE_WIDTH;
                        painter->drawArc(p.x() - radius, p.y() - radius, radius * 2, radius * 2, 1440 - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) / M_PI * 2880, 2880);
					}
					addHighlightPath(highlightPath, p, M_PI / 2.0 - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()), M_PI, highlightOption, false);
				} else if (rightPivotAngle >= 0) {
                    const QPointF &p = mapFromParent(getFixedIntersection()->mapToParent(getFixedIntersection()->getConnectionLeft(getFixedIntersectionDirection())));
					for (int i = 1; i <= lanes; i++) {
						pen.setStyle(i == lanes || (lanes - divider) % lanes == i ? Qt::SolidLine : Qt::DashLine);
						painter->setPen(pen);
						float radius = i * Sizes::LANE_WIDTH;
                        painter->drawArc(p.x() - radius, p.y() - radius, radius * 2, radius * 2, 1440 - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) / M_PI * 2880, 2880);
					}
					addHighlightPath(highlightPath, p, M_PI / 2.0 - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()), M_PI, highlightOption, true);
				}
			}
        }
    }
    // Drawing Free Intersection Arcs
    if (getFreeIntersection() != nullptr) {
        const Orientation &fixedOrientation = Direction::getOrientation(getFixedIntersectionDirection());
        const Orientation &freeOrientation = Direction::getOrientation(getFreeIntersectionDirection());
        float fixedEndRadius = getFixedIntersection()->getTotalLanesWidth(fixedOrientation) / 2;

        const QPointF &coordL = getFreeIntersection()->mapToItem(getFixedIntersection(), getFreeIntersection()->getConnectionLeft(getFreeIntersectionDirection()));
        const QPointF &coordR = getFreeIntersection()->mapToItem(getFixedIntersection(), getFreeIntersection()->getConnectionRight(getFreeIntersectionDirection()));
        const QPointF &origL = getFixedIntersection()->getConnectionLeft(getFixedIntersectionDirection());
        const QPointF &origR = getFixedIntersection()->getConnectionRight(getFixedIntersectionDirection());

        float pivotAngleL = fmod(fmod(atan2(coordL.y() - origR.y(), coordL.x() - origR.x()) + getFixedIntersection()->getAngle() - getFreeIntersection()->getConnectionDirection(getFreeIntersectionDirection()), 2 * M_PI) + 2 * M_PI, 2 * M_PI);
        bool freeLeftPivot = pivotAngleL >= M_PI;
        const QPointF &freeEndPivot = freeLeftPivot ? coordL : coordR;

        float freeEndRadius = getFreeIntersection()->getTotalLanesWidth(freeOrientation) / (freeLeftPivot ? 2 : -2);

        float roadAngleRad = getParallelLineAngle(origR, freeEndPivot, fixedEndRadius, freeEndRadius);


        float pivotAngleR = fmod(fmod(atan2(coordR.y() - origL.y(), coordR.x() - origL.x()) + getFixedIntersection()->getAngle() - getFreeIntersection()->getConnectionDirection(getFreeIntersectionDirection()), 2 * M_PI) + 2 * M_PI, 2 * M_PI);
        bool freeRightPivot = pivotAngleR < M_PI;
        const QPointF &freeEndPivot2 = freeRightPivot ? coordR : coordL;

        float freeEndRadius2 = getFreeIntersection()->getTotalLanesWidth(freeOrientation) / (freeRightPivot ? 2 : -2);

        float roadAngleRad2 = getParallelLineAngle(origL, freeEndPivot2, -fixedEndRadius, -freeEndRadius2);

        if (fmodf(fmodf(roadAngleRad - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) + getFixedIntersection()->getAngle(), 2 * M_PI) + 2 * M_PI, 2 * M_PI) >= M_PI) {
            float pivotAngleRad = fmodf(fmodf((freeLeftPivot ? 1 : -1) * (getFreeIntersection()->getConnectionDirection(getFreeIntersectionDirection()) - getFixedIntersection()->getAngle() - roadAngleRad), 2 * M_PI) + 2 * M_PI, 2 * M_PI);
            if (pivotAngleRad < M_PI) {
				const QPointF &pivot = mapFromParent(getFixedIntersection()->mapToParent(freeEndPivot));
				for (int i = 1; i <= lanes; i++) {
					pen.setStyle(i == lanes || (freeLeftPivot ? lanes + divider : lanes - divider) % lanes == i ? Qt::SolidLine : Qt::DashLine);
					painter->setPen(pen);
                    float radius = i * Sizes::LANE_WIDTH;
					painter->drawArc(pivot.x() - radius, pivot.y() - radius, 2 * radius, 2 * radius, (freeLeftPivot ? 1440 : 4320) - getFreeIntersection()->getConnectionDirection(getFreeIntersectionDirection()) / M_PI * 2880, (freeLeftPivot ? 1 : -1) * pivotAngleRad / M_PI * 2880);
                }
				addHighlightPath(highlightPath, pivot, (freeLeftPivot ? M_PI : 3 * M_PI) / 2.0 - getFreeIntersection()->getConnectionDirection(getFreeIntersectionDirection()), (freeLeftPivot ? 1 : -1) * pivotAngleRad, highlightOption, !freeLeftPivot);
			}
        }
        if (fmodf(fmodf(roadAngleRad2 - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) + getFixedIntersection()->getAngle(), 2 * M_PI) + 2 * M_PI, 2 * M_PI) <= M_PI) {
            float pivotAngleRad = fmodf(fmodf((freeRightPivot ? -1 : 1) * (getFreeIntersection()->getConnectionDirection(getFreeIntersectionDirection()) - getFixedIntersection()->getAngle() - roadAngleRad2), 2 * M_PI) + 2 * M_PI, 2 * M_PI);
            if (pivotAngleRad < M_PI) {
				const QPointF &pivot = mapFromParent(getFixedIntersection()->mapToParent(freeEndPivot2));
				for (int i = 1; i <= lanes; i++) {
					pen.setStyle(i == lanes || (freeRightPivot ? lanes - divider : lanes + divider) % lanes == i ? Qt::SolidLine : Qt::DashLine);
					painter->setPen(pen);
                    float radius = i * Sizes::LANE_WIDTH;
					painter->drawArc(pivot.x() - radius, pivot.y() - radius, 2 * radius, 2 * radius, (freeRightPivot ? 4320 : 1440) - getFreeIntersection()->getConnectionDirection(getFreeIntersectionDirection()) / M_PI * 2880, (freeRightPivot ? -1 : 1) * pivotAngleRad / M_PI * 2880);
                }
				addHighlightPath(highlightPath, pivot, (freeRightPivot ? 3 * M_PI : M_PI) / 2.0 - getFreeIntersection()->getConnectionDirection(getFreeIntersectionDirection()), (freeRightPivot ? -1 : 1) * pivotAngleRad, highlightOption, freeRightPivot);
			}
        }
    }

	QPointF tCopy(t.y() / tangentLength, -t.x() / tangentLength);

	// Drawing highlight
	if (highlightOption != Road::HighlightOption::NONE) {
		int divider = getDirectionDividerLane();

		const QPointF &distanceSide1 = tCopy * -intersectionSize / 2.0;
		const QPointF &center = tCopy * (-intersectionSize / 2 + ((lanes + divider) % lanes * Sizes::LANE_WIDTH));
		const QPointF &distanceSide2 = tCopy * intersectionSize / 2.0;
		QVector<QPointF> highlightPoints;
		if (highlightOption == Road::HighlightOption::ALL || (divider == 0 && highlightOption == Road::HighlightOption::FREE_TO_FIXED) || (std::abs(divider) >= getLanes() && highlightOption == Road::HighlightOption::FIXED_TO_FREE)) {
			highlightPoints.push_back(p1 + distanceSide1);
			highlightPoints.push_back(p2 + distanceSide1);
			highlightPoints.push_back(p2 + distanceSide2);
			highlightPoints.push_back(p1 + distanceSide2);
		} else {
			highlightPoints.push_back(p1 + center);
			highlightPoints.push_back(p2 + center);
			if ((divider > 0) == (highlightOption == Road::HighlightOption::FIXED_TO_FREE)) {
				highlightPoints.push_back(p2 + distanceSide1);
				highlightPoints.push_back(p1 + distanceSide1);
			} else {
				highlightPoints.push_back(p2 + distanceSide2);
				highlightPoints.push_back(p1 + distanceSide2);
			}
		}
		if (highlightPoints.size() >= 4) {
			highlightPath.addPolygon(QPolygonF(highlightPoints));
			painter->fillPath(highlightPath, Qt::red);
		}
	}

	// Drawing Lanes
	for (int i = 0; i <= lanes; i++) {
		pen.setStyle(i == 0 || i == lanes || i == (divider + lanes) % lanes ? Qt::SolidLine : Qt::DashLine);
        painter->setPen(pen);
		const QPointF &distance = tCopy * (-intersectionSize / 2 + (i * Sizes::LANE_WIDTH));
        painter->drawLine(p1 + distance, p2 + distance);
    }
    pen.setColor(Qt::black);
    pen.setStyle(Qt::SolidLine);
    painter->drawLine(p1 - tCopy * (intersectionSize / 2), p1 + tCopy * (intersectionSize / 2));
	if (getFreeIntersection() == nullptr) {
        painter->drawLine(p2 - tCopy * (intersectionSize / 2), p2 + tCopy * (intersectionSize / 2));
    }
}

VehiclePosition* LinearRoad::getNewPosition(bool movingFixedToFree, int lane, float t, float bufferDistance) {
    return new LinearRoadPosition(this, movingFixedToFree, lane, t, bufferDistance, lane, NAN, NAN);
}

double LinearRoad::getLength() const {
    const QPointF &start = getStart();
    const QPointF &end = getEnd();
	return sqrt(pow(start.x() - end.x(), 2) + pow(start.y() - end.y(), 2));
}

double LinearRoad::getPathLength(int lane) const {
	return (getBeginningPosition(lane, false) - getBeginningPosition(lane, true)) * getLength();
}

float LinearRoad::getBeginningPosition(int lane, bool fixedEnd) const {
	const QPointF &start = getPoint(0);
	const QPointF &end = getPoint(1);
	float totalDistance = sqrt(pow(start.x() - end.x(), 2) + pow(start.y() - end.y(), 2));
	const QPointF &tangent = getTangent(0);
	float startAngle = atan2(tangent.y(), tangent.x()) - M_PI;
	Intersection *fixedIntersection = getFixedIntersection();
	Intersection *freeIntersection = getFreeIntersection();
	if (fixedEnd && fixedIntersection != nullptr) {
		float turnAngle = fmodf(fmodf(startAngle - fixedIntersection->getConnectionDirection(getFixedIntersectionDirection()), 2 * M_PI) + 2 * M_PI, 2 * M_PI);
		if (turnAngle < M_PI) {
			float turnRadius = (getLanes() - lane - 0.5) * Sizes::LANE_WIDTH;
			return -turnAngle * turnRadius / totalDistance;
		} else {
			double turnRadius = (lane + 0.5) * Sizes::LANE_WIDTH;
			return (turnAngle - 2 * M_PI) * turnRadius / totalDistance;
		}
	} else if (!fixedEnd) {
		if (freeIntersection != nullptr) {
			const QPointF &tangent = getTangent(1);
			float endAngle = atan2(tangent.y(), tangent.x());
			double turnAngle = fmodf(fmodf(freeIntersection->getConnectionDirection(getFreeIntersectionDirection()) - endAngle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
			if (turnAngle < M_PI) {
				return 1 + turnAngle * Sizes::LANE_WIDTH * (getLanes() - lane - 0.5) / totalDistance;
			} else {
				return 1 + (2 * M_PI - turnAngle) * Sizes::LANE_WIDTH * (lane + 0.5) / totalDistance;
			}
		} else {
			return 1;
		}
	}
	return 0;
}

QPointF LinearRoad::getStart() const {
    if (getFreeIntersection() != nullptr) {
        const std::array<QPointF, 2> &points = getConnectedFreeEndPoints();
        if (!std::isnan(points[0].x())) {
            return points[0];
        }
    } else {
        const QPointF &d = getFixedIntersection()->mapFromParent(getFreeEndX(), getFreeEndY());
        const QPointF &e = getFixedIntersection()->getConnectionLeft(getFixedIntersectionDirection());
        const QPointF &f = getFixedIntersection()->getConnectionRight(getFixedIntersectionDirection());
        float fixedEndRadius = getFixedIntersection()->getTotalLanesWidth(Direction::getOrientation(getFixedIntersectionDirection())) / 2;
        float leftPivotAngle = fmodf(fmodf(getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) - getFixedIntersection()->getAngle() - getParallelLineAngle(e, d, -fixedEndRadius, 0) + M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
        float rightPivotAngle = fmodf(fmodf(getParallelLineAngle(f, d, fixedEndRadius, 0) - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) + getFixedIntersection()->getAngle() + M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
        if (leftPivotAngle < M_PI || rightPivotAngle < M_PI) {
            float directionAngle = getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) - getFixedIntersection()->getAngle();
            if (leftPivotAngle < M_PI) {
                QPointF p = getFixedIntersection()->getConnectionLeft(getFixedIntersectionDirection());
                float pivotAngle = leftPivotAngle - directionAngle;
                return getFixedIntersection()->mapToParent(p.x() - sin(pivotAngle) * fixedEndRadius, p.y() - cos(pivotAngle) * abs(fixedEndRadius));
            } else if (rightPivotAngle < M_PI) {
                QPointF p = getFixedIntersection()->getConnectionRight(getFixedIntersectionDirection());
                float pivotAngle = rightPivotAngle + directionAngle;
                return getFixedIntersection()->mapToParent(p.x() - sin(pivotAngle) * fixedEndRadius, p.y() + cos(pivotAngle) * abs(fixedEndRadius));
            }
        } else {
            float directionAngle = getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) - getFixedIntersection()->getAngle();
            if (leftPivotAngle < rightPivotAngle || leftPivotAngle > rightPivotAngle || leftPivotAngle == rightPivotAngle) {
                fixedEndRadius *= leftPivotAngle <= rightPivotAngle ? 1 : -1;
                const QPointF &p = leftPivotAngle <= rightPivotAngle ? getFixedIntersection()->getConnectionLeft(getFixedIntersectionDirection()) : getFixedIntersection()->getConnectionRight(getFixedIntersectionDirection());
                return getFixedIntersection()->mapToParent(p.x() - sin(directionAngle) * fixedEndRadius, p.y() + cos(directionAngle) * fixedEndRadius);
            } else {
                float yDis = cos(atan2(d.y() - e.y(), d.x() - e.x()) - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) + getFixedIntersection()->getAngle()) * sqrt(pow(d.x() - e.x(), 2) + pow(d.y() - e.y(), 2));
                if (yDis > 0) {
                    if (leftPivotAngle >= 0) {
                        const QPointF &p = getFixedIntersection()->getConnectionRight(getFixedIntersectionDirection());
                        return getFixedIntersection()->mapToParent(p.x() + sin(directionAngle) * fixedEndRadius, p.y() - cos(directionAngle) * fixedEndRadius);
                    } else if (rightPivotAngle >= 0) {
                        const QPointF &p = getFixedIntersection()->getConnectionLeft(getFixedIntersectionDirection());
                        return getFixedIntersection()->mapToParent(p.x() - sin(directionAngle) * fixedEndRadius, p.y() + cos(directionAngle) * fixedEndRadius);
					}
				} else {
					if (std::isnan(leftPivotAngle)) {
						double angle = atan2(d.y() - e.y(), d.x() - e.x());
						return getFixedIntersection()->mapToParent(e.x() + fixedEndRadius * cos(angle), e.y() + fixedEndRadius * sin(angle));
					} else {
						double angle = atan2(d.y() - f.y(), d.x() - f.x());
						return getFixedIntersection()->mapToParent(f.x() + fixedEndRadius * cos(angle), f.y() + fixedEndRadius * sin(angle));
					}
				}
			}
		}
    }
    return QPointF(NAN, NAN);
}

QPointF LinearRoad::getEnd() const {
    if (getFreeIntersection() != nullptr) {
        const std::array<QPointF, 2> &points = getConnectedFreeEndPoints();
        if (!std::isnan(points[1].x())) {
            return points[1];
        }
    } else {
        const QPointF &d = getFixedIntersection()->mapFromParent(getFreeEndX(), getFreeEndY());
        const QPointF &e = getFixedIntersection()->getConnectionLeft(getFixedIntersectionDirection());
        const QPointF &f = getFixedIntersection()->getConnectionRight(getFixedIntersectionDirection());
        float fixedEndRadius = getFixedIntersection()->getTotalLanesWidth(Direction::getOrientation(getFixedIntersectionDirection())) / 2;
        float leftPivotAngle = fmodf(fmodf(getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) - getFixedIntersection()->getAngle() - getParallelLineAngle(e, d, -fixedEndRadius, 0) + M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
        float rightPivotAngle = fmodf(fmodf(getParallelLineAngle(f, d, fixedEndRadius, 0) - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) + getFixedIntersection()->getAngle() + M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
        if (leftPivotAngle < M_PI || rightPivotAngle < M_PI) {
            return QPointF(getFreeEndX(), getFreeEndY());
        } else {
            float yDis = cos(atan2(d.y() - e.y(), d.x() - e.x()) - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) + getFixedIntersection()->getAngle()) * sqrt(pow(d.x() - e.x(), 2) + pow(d.y() - e.y(), 2));
            float directionAngle = getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) - getFixedIntersection()->getAngle();
            if (leftPivotAngle < rightPivotAngle || leftPivotAngle > rightPivotAngle || leftPivotAngle == rightPivotAngle) {
                fixedEndRadius *= leftPivotAngle <= rightPivotAngle ? 1 : -1;
                const QPointF &p = leftPivotAngle <= rightPivotAngle ? getFixedIntersection()->getConnectionLeft(getFixedIntersectionDirection()) : getFixedIntersection()->getConnectionRight(getFixedIntersectionDirection());
                const QPointF start(p.x() - sin(directionAngle) * fixedEndRadius, p.y() + cos(directionAngle) * fixedEndRadius);
                return getFixedIntersection()->mapToParent(start.x() + yDis * cos(directionAngle), start.y() + yDis * sin(directionAngle));
            } else if (yDis > 0) {
                if (leftPivotAngle >= 0) {
                    const QPointF &p = getFixedIntersection()->getConnectionRight(getFixedIntersectionDirection());
                    const QPointF start(p.x() + sin(directionAngle) * fixedEndRadius, p.y() - cos(directionAngle) * fixedEndRadius);
                    return getFixedIntersection()->mapToParent(start.x() + yDis * cos(directionAngle), start.y() + yDis * sin(directionAngle));
                } else if (rightPivotAngle >= 0) {
                    const QPointF &p = getFixedIntersection()->getConnectionLeft(getFixedIntersectionDirection());
                    const QPointF start(p.x() - sin(directionAngle) * fixedEndRadius, p.y() + cos(directionAngle) * fixedEndRadius);
                    return getFixedIntersection()->mapToParent(start.x() + yDis * cos(directionAngle), start.y() + yDis * sin(directionAngle));
                }
			} else {
				if (std::isnan(leftPivotAngle)) {
					double angle = atan2(d.y() - e.y(), d.x() - e.x());
					const QPointF &p = getFixedIntersection()->mapToParent(e.x() + fixedEndRadius * cos(angle), e.y() + fixedEndRadius * sin(angle));
					angle += getFixedIntersection()->getAngle() - M_PI / 2;
					return p + 0.0001 * QPointF(cos(angle), sin(angle));
				} else {
					double angle = atan2(d.y() - f.y(), d.x() - f.x());
					const QPointF &p = getFixedIntersection()->mapToParent(f.x() + fixedEndRadius * cos(angle), f.y() + fixedEndRadius * sin(angle));
					angle += getFixedIntersection()->getAngle() + M_PI / 2;
					return p + 0.0001 * QPointF(cos(angle), sin(angle));
				}
			}
        }
    }
    return QPointF(NAN, NAN);
}

float LinearRoad::getFixedIntersectionSize() const {
    return getFixedIntersection()->getTotalLanesWidth(Direction::getOrientation(getFixedIntersectionDirection()));
}

int LinearRoad::getFixedIntersectionLanes() const {
    return getFixedIntersection()->getLanes(Direction::getOrientation(getFixedIntersectionDirection()));
}

float LinearRoad::getParallelLineAngle(const QPointF &p1, const QPointF &p2, float d1, float d2) const {
    float kx = p2.x() - p1.x();
    float ky = p2.y() - p1.y();
    float c = d1 - d2;
    float sqr = sqrt(kx * kx + ky * ky - c * c);
    return M_PI / 2 + atan2(c * ky - kx * sqr, c * kx + ky * sqr);
}

std::array<QPointF, 2> LinearRoad::getConnectedFreeEndPoints() const {
    if (getFreeIntersection() != nullptr) {
        const QPointF &coordL = getFreeIntersection()->mapToItem(getFixedIntersection(), getFreeIntersection()->getConnectionLeft(getFreeIntersectionDirection()));
        const QPointF &coordR = getFreeIntersection()->mapToItem(getFixedIntersection(), getFreeIntersection()->getConnectionRight(getFreeIntersectionDirection()));
        const QPointF &origL = getFixedIntersection()->getConnectionLeft(getFixedIntersectionDirection());
        const QPointF &origR = getFixedIntersection()->getConnectionRight(getFixedIntersectionDirection());

        float pivotAngleL = fmod(fmod(atan2(coordL.y() - origR.y(), coordL.x() - origR.x()) + getFixedIntersection()->getAngle() - getFreeIntersection()->getConnectionDirection(getFreeIntersectionDirection()), 2 * M_PI) + 2 * M_PI, 2 * M_PI);
        bool freeLeftPivot = pivotAngleL >= M_PI;
        const QPointF &freeEndPivot = freeLeftPivot ? coordL : coordR;

        const Orientation &fixedOrientation = Direction::getOrientation(getFixedIntersectionDirection());
        float fixedEndRadius = getFixedIntersection()->getTotalLanesWidth(fixedOrientation) / 2;
        const Orientation &freeOrientation = Direction::getOrientation(getFreeIntersectionDirection());
        float freeEndRadius = getFreeIntersection()->getTotalLanesWidth(freeOrientation) / (freeLeftPivot ? 2 : -2);

        float roadAngleRad = getParallelLineAngle(origR, freeEndPivot, fixedEndRadius, freeEndRadius);


        float pivotAngleR = fmod(fmod(atan2(coordR.y() - origL.y(), coordR.x() - origL.x()) + getFixedIntersection()->getAngle() - getFreeIntersection()->getConnectionDirection(getFreeIntersectionDirection()), 2 * M_PI) + 2 * M_PI, 2 * M_PI);
        bool freeRightPivot = pivotAngleR < M_PI;
        const QPointF &freeEndPivot2 = freeRightPivot ? coordR : coordL;

        float freeEndRadius2 = getFreeIntersection()->getTotalLanesWidth(freeOrientation) / (freeRightPivot ? 2 : -2);

        float roadAngleRad2 = getParallelLineAngle(origL, freeEndPivot2, -fixedEndRadius, -freeEndRadius2);


        float abc = M_PI / 2 - getFreeIntersection()->getConnectionDirection(getFreeIntersectionDirection()) + getFreeIntersection()->getAngle();
        if (fmodf(fmodf(roadAngleRad - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) + getFixedIntersection()->getAngle(), 2 * M_PI) + 2 * M_PI, 2 * M_PI) >= M_PI &&
                fmodf(fmodf((freeLeftPivot ? 1 : -1) * (M_PI / 2 - roadAngleRad + getFreeIntersection()->getAngle() - getFixedIntersection()->getAngle() - abc), 2 * M_PI) + 2 * M_PI, 2 * M_PI) < M_PI)  {
            QPointF roadStart(origR.x() + fixedEndRadius * cos(roadAngleRad - M_PI / 2), origR.y() + fixedEndRadius * sin(roadAngleRad - M_PI / 2));
            QPointF roadEnd(freeEndPivot.x() - freeEndRadius * cos(roadAngleRad + M_PI / 2), freeEndPivot.y() - freeEndRadius * sin(roadAngleRad + M_PI / 2));
            std::array<QPointF, 2> points = {getFixedIntersection()->mapToParent(roadStart), getFixedIntersection()->mapToParent(roadEnd)};
            return points;
        }
        if (fmodf(fmodf(roadAngleRad2 - getFixedIntersection()->getConnectionDirection(getFixedIntersectionDirection()) + getFixedIntersection()->getAngle(), 2 * M_PI) + 2 * M_PI, 2 * M_PI) <= M_PI &&
                fmodf(fmodf((freeRightPivot ? -1 : 1) * (M_PI / 2 - roadAngleRad2 + getFreeIntersection()->getAngle() - getFixedIntersection()->getAngle() - abc), 2 * M_PI) + 2 * M_PI, 2 * M_PI) < M_PI) {
            QPointF roadStart(origL.x() - fixedEndRadius * cos(roadAngleRad2 - M_PI / 2), origL.y() - fixedEndRadius * sin(roadAngleRad2 - M_PI / 2));
            QPointF roadEnd(freeEndPivot2.x() + freeEndRadius2 * cos(roadAngleRad2 + M_PI / 2), freeEndPivot2.y() + freeEndRadius2 * sin(roadAngleRad2 + M_PI / 2));
            std::array<QPointF, 2> points = {getFixedIntersection()->mapToParent(roadStart), getFixedIntersection()->mapToParent(roadEnd)};
            return points;
        }
    }
    std::array<QPointF, 2> points = {QPointF(NAN, NAN), QPointF(NAN, NAN)};
	return points;
}

void LinearRoad::addHighlightPath(QPainterPath &path, const QPointF &center, double startAngle, double angleLength, const Road::HighlightOption &highlightOption, bool arcLeft) const {
	startAngle *= 180.0 / M_PI;
	angleLength *= 180.0 / M_PI;
	int lanesCount = getLanes();
	float outerRadius = lanesCount * Sizes::LANE_WIDTH;
	float innerRadius = 0;
	if (highlightOption == Road::HighlightOption::NONE) {
		outerRadius = 0;
	} else if (highlightOption != Road::HighlightOption::ALL) {
		int divider = getDirectionDividerLane();
		if (divider == 0 || std::abs(divider) >= lanesCount) {
			if ((highlightOption == Road::HighlightOption::FIXED_TO_FREE) == (divider == 0)) {
				outerRadius = 0;
			} else {
				outerRadius = lanesCount * Sizes::LANE_WIDTH;
				innerRadius = 0;
			}
		} else if ((highlightOption == Road::HighlightOption::FIXED_TO_FREE) == ((divider > 0) == arcLeft)) {
			outerRadius = lanesCount * Sizes::LANE_WIDTH;
			innerRadius = (lanesCount - std::abs(divider)) * Sizes::LANE_WIDTH;
			innerRadius = (arcLeft ? lanesCount - divider :  lanesCount + divider) % lanesCount * Sizes::LANE_WIDTH;
		} else {
			outerRadius = std::abs(divider) * Sizes::LANE_WIDTH;
			outerRadius = (arcLeft ? lanesCount - divider :  lanesCount + divider) % lanesCount * Sizes::LANE_WIDTH;
			innerRadius = 0;
		}
	}
	const QRectF outerRect(center.x() - outerRadius, center.y() - outerRadius, outerRadius * 2, outerRadius * 2);
	const QRectF innerRect(center.x() - innerRadius, center.y() - innerRadius, innerRadius * 2, innerRadius * 2);
	path.arcMoveTo(outerRect, startAngle);
	path.arcTo(outerRect, startAngle, angleLength);
	if (outerRadius > innerRadius) {
		if (innerRadius == 0) {
			path.lineTo(center);
		} else {
			QPainterPath tempPath1;
			tempPath1.arcMoveTo(innerRect, startAngle + angleLength);
			path.lineTo(tempPath1.currentPosition());

			path.arcTo(innerRect, startAngle + angleLength, -angleLength);
		}
		path.closeSubpath();
	}
}

LinearRoadPosition::LinearRoadPosition(LinearRoad *linearRoad, bool movingFixedToFree, int lane, float t, float bufferDistance, int changeToLane, double changeLaneStartPosition, double changeLaneFinishPosition) :
        VehiclePosition(),
        linearRoad(linearRoad),
        movingFixedToFree(movingFixedToFree),
        lane(lane),
        position(t),
        changeToLane(changeToLane),
        changeLaneStartPosition(changeLaneStartPosition),
		changeLaneFinishPosition(changeLaneFinishPosition) {
	if (this->movingFixedToFree != this->linearRoad->isLaneMovingToFreeEnd(this->lane)) {
		this->movingFixedToFree = !this->movingFixedToFree;
	}
    if (t == -INFINITY) {
		this->position = getBeginningPosition(this->movingFixedToFree);
	}
    const QPointF &start = this->linearRoad->getPoint(0);
    const QPointF &end = this->linearRoad->getPoint(1);
    double totalDistance = sqrt(pow(start.x() - end.x(), 2) + pow(start.y() - end.y(), 2));
    if (this->movingFixedToFree) {
        this->position += bufferDistance / totalDistance;
    } else {
        this->position -= bufferDistance / totalDistance;
    }
    if (this->changeToLane != this->lane && !std::isnan(this->changeLaneStartPosition) && !std::isnan(this->changeLaneFinishPosition) && this->changeLaneStartPosition != this->changeLaneFinishPosition && (this->movingFixedToFree == (this->changeLaneFinishPosition > this->changeLaneStartPosition))) {
        if (this->movingFixedToFree == (this->position >= this->changeLaneFinishPosition)) {
            this->lane = this->changeToLane;
            this->changeLaneStartPosition = NAN;
            this->changeLaneFinishPosition = NAN;
        } else if (this->changeToLane < 0) {
            this->changeToLane = 0;
        } else if (this->changeToLane >= this->linearRoad->getLanes()) {
            this->changeToLane = this->linearRoad->getLanes() - 1;
        }
    } else {
        this->changeToLane = this->lane;
        this->changeLaneStartPosition = NAN;
        this->changeLaneFinishPosition = NAN;
    }
}

LinearRoadPosition::~LinearRoadPosition() {
}

VehiclePath *LinearRoadPosition::getPath() const {
	return this->linearRoad;
}

VehiclePath *LinearRoadPosition::getNextPath() const {
	return this->movingFixedToFree ? this->linearRoad->getFreeIntersection() : this->linearRoad->getFixedIntersection();
}

int LinearRoadPosition::getLane() const {
    return this->lane;
}

int LinearRoadPosition::getChangeToLane() const {
    return this->changeToLane;
}

int LinearRoadPosition::getNextPathEnterLane() const {
	return this->movingFixedToFree ? this->linearRoad->getLanes() - 1 - this->lane : this->lane;
}

Direction::Cardinal LinearRoadPosition::getSourceDirection() const {
    return this->movingFixedToFree ? this->linearRoad->getFixedIntersectionDirection() : this->linearRoad->getFreeIntersectionDirection();
}

Direction::Cardinal LinearRoadPosition::getDestinationDirection() const {
    return this->movingFixedToFree ? this->linearRoad->getFreeIntersectionDirection() : this->linearRoad->getFixedIntersectionDirection();
}

QPointF LinearRoadPosition::getLocation() const {
    const QPointF &start = this->linearRoad->getPoint(0);
    const QPointF &end = this->linearRoad->getPoint(1);
    double totalDistance = sqrt(pow(start.x() - end.x(), 2) + pow(start.y() - end.y(), 2));
    double currentPosition = this->position;
    if (currentPosition < 0) {
        Intersection *fixedIntersection = this->linearRoad->getFixedIntersection();
        if (fixedIntersection != nullptr) {
            const QPointF &tangent = this->linearRoad->getTangent(0);
            double startAngle = atan2(tangent.y(), tangent.x()) - M_PI;
            double turnAngle = fmodf(fmodf(startAngle - fixedIntersection->getConnectionDirection(this->linearRoad->getFixedIntersectionDirection()), 2 * M_PI) + 2 * M_PI, 2 * M_PI);
            if (turnAngle < M_PI) {
                double turnRadius = (this->linearRoad->getLanes() - this->lane - 0.5) * Sizes::LANE_WIDTH;
                double turnEndAngle = startAngle + M_PI / 2 - std::min(-currentPosition * totalDistance / turnRadius, turnAngle);
                return fixedIntersection->mapToParent(fixedIntersection->getConnectionRight(this->linearRoad->getFixedIntersectionDirection())) + QPointF(turnRadius * cos(turnEndAngle), turnRadius * sin(turnEndAngle));
            } else {
                double turnRadius = (this->lane + 0.5) * Sizes::LANE_WIDTH;
                double turnEndAngle = startAngle + M_PI / 2 + std::min(-currentPosition * totalDistance / turnRadius, 2 * M_PI - turnAngle);
                return fixedIntersection->mapToParent(fixedIntersection->getConnectionLeft(this->linearRoad->getFixedIntersectionDirection())) - QPointF(turnRadius * cos(turnEndAngle), turnRadius * sin(turnEndAngle));
            }
        } else {
            currentPosition = 0;
        }
    } else if (currentPosition > 1) {
        Intersection *freeIntersection = this->linearRoad->getFreeIntersection();
        if (freeIntersection != nullptr) {
            const QPointF &tangent = this->linearRoad->getTangent(1);
            double endDirection = atan2(tangent.y(), tangent.x()) - M_PI / 2;
            double turnAngle = fmodf(fmodf(freeIntersection->getConnectionDirection(this->linearRoad->getFreeIntersectionDirection()) - M_PI / 2 - endDirection, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
            if (turnAngle < M_PI) {
                double turnRadius = (this->linearRoad->getLanes() - this->lane - 0.5) * Sizes::LANE_WIDTH;
                double turnEndAngle = endDirection + std::min(((currentPosition - 1) * totalDistance) / turnRadius, turnAngle);
                return freeIntersection->mapToParent(freeIntersection->getConnectionLeft(this->linearRoad->getFreeIntersectionDirection())) + QPointF(turnRadius * cos(turnEndAngle), turnRadius * sin(turnEndAngle));
            } else {
                double turnRadius = (this->lane + 0.5) * Sizes::LANE_WIDTH;
                double turnEndAngle = endDirection - std::min(((currentPosition - 1) * totalDistance) / turnRadius, 2 * M_PI - turnAngle);
                return freeIntersection->mapToParent(freeIntersection->getConnectionRight(this->linearRoad->getFreeIntersectionDirection())) - QPointF(turnRadius * cos(turnEndAngle), turnRadius * sin(turnEndAngle));
            }
        } else {
            currentPosition = 1;
        }
    } else if (isChangingLanes()) {
		VehiclePosition::SinCurveParameters p(this->changeLaneStartPosition, this->lane * Sizes::LANE_WIDTH, this->changeLaneFinishPosition, this->changeToLane * Sizes::LANE_WIDTH, fabs(this->changeLaneFinishPosition - this->changeLaneStartPosition));
        double lane = p.evaluate(this->position) / Sizes::LANE_WIDTH;

        const QPointF &tangent = this->linearRoad->getTangent(this->position);
        double a = atan2(tangent.y(), tangent.x()) + (this->movingFixedToFree ? 0 : M_PI) + M_PI / 2;

        double distance = Sizes::LANE_WIDTH * (lane - (this->linearRoad->getLanes() - 1) / 2.f) * (this->movingFixedToFree ? 1 : -1);
        return this->linearRoad->mapToParent(this->linearRoad->mapFromParent(this->linearRoad->getPoint(currentPosition)) + QPointF(distance * cos(a), distance * sin(a)));
    }
    double a = getDirection() + M_PI / 2;
    double distance = Sizes::LANE_WIDTH * (this->lane - (this->linearRoad->getLanes() - 1) / 2.f) * (this->movingFixedToFree ? 1 : -1);
    return this->linearRoad->mapToParent(this->linearRoad->mapFromParent(this->linearRoad->getPoint(currentPosition)) + QPointF(distance * cos(a), distance * sin(a)));
}

float LinearRoadPosition::getDirection() const {
    const QPointF &start = this->linearRoad->getPoint(0);
    const QPointF &end = this->linearRoad->getPoint(1);
    float totalDistance = sqrt(pow(start.x() - end.x(), 2) + pow(start.y() - end.y(), 2));
    float currentPosition = this->position;
    if (currentPosition < 0) {
        const QPointF &tangent = this->linearRoad->getTangent(0);
        double startAngle = atan2(tangent.y(), tangent.x()) - M_PI;
        Intersection *fixedIntersection = this->linearRoad->getFixedIntersection();
        if (fixedIntersection != nullptr) {
            double turnAngle = fmodf(fmodf(startAngle - fixedIntersection->getConnectionDirection(this->linearRoad->getFixedIntersectionDirection()), 2 * M_PI) + 2 * M_PI, 2 * M_PI);
            if (turnAngle < M_PI) {
                double turnRadius = (this->linearRoad->getLanes() - this->lane - 0.5) * Sizes::LANE_WIDTH;
                return startAngle + (this->movingFixedToFree ? M_PI : 0) - std::min(-currentPosition * totalDistance / turnRadius, turnAngle);
            } else {
                double turnRadius = (this->lane + 0.5) * Sizes::LANE_WIDTH;
                return startAngle + (this->movingFixedToFree ? M_PI : 0) + std::min(-currentPosition * totalDistance / turnRadius, 2 * M_PI - turnAngle);
            }
        }
    } else if (currentPosition > 1) {
        const QPointF &tangent = this->linearRoad->getTangent(1);
        float endAngle = atan2(tangent.y(), tangent.x());
        Intersection *freeIntersection = this->linearRoad->getFreeIntersection();
        if (freeIntersection != nullptr) {
            double turnAngle = fmodf(fmodf(freeIntersection->getConnectionDirection(this->linearRoad->getFreeIntersectionDirection()) - endAngle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
            if (turnAngle < M_PI) {
                return endAngle + (this->movingFixedToFree ? 0 : M_PI) + std::min(((currentPosition - 1) * totalDistance) / (this->linearRoad->getLanes() - this->lane - 0.5) / Sizes::LANE_WIDTH, turnAngle);
            } else {
                return endAngle + (this->movingFixedToFree ? 0 : M_PI) - std::min(((currentPosition - 1) * totalDistance) / (this->lane + 0.5) / Sizes::LANE_WIDTH, 2 * M_PI - turnAngle);
            }
        }
    } else if (isChangingLanes()) {
		VehiclePosition::SinCurveParameters p(this->changeLaneStartPosition, this->lane * Sizes::LANE_WIDTH, this->changeLaneFinishPosition, this->changeToLane * Sizes::LANE_WIDTH, fabs(this->changeLaneFinishPosition - this->changeLaneStartPosition));
        const QPointF &tangent = this->linearRoad->getTangent(this->position);
        return atan(p.derivative(this->position) / totalDistance) + atan2(tangent.y(), tangent.x()) + (this->movingFixedToFree ? 0 : M_PI);
    }
    const QPointF &tangent = this->linearRoad->getTangent(1);
    return atan2(tangent.y(), tangent.x()) + (this->movingFixedToFree ? 0 : M_PI);
}

VehiclePosition* LinearRoadPosition::getNextPosition(double distance) const {
    const QPointF &start = this->linearRoad->getPoint(0);
    const QPointF &end = this->linearRoad->getPoint(1);
    float totalDistance = sqrt(pow(start.x() - end.x(), 2) + pow(start.y() - end.y(), 2));
    if (isChangingLanes()) {
        if (this->movingFixedToFree) {
			VehiclePosition::SinCurveParameters p(this->changeLaneStartPosition * totalDistance, this->lane * Sizes::LANE_WIDTH, this->changeLaneFinishPosition * totalDistance, this->changeToLane * Sizes::LANE_WIDTH, fabs(this->changeLaneFinishPosition - this->changeLaneStartPosition) * totalDistance);
            double newPosition = p.getLengthAfterPositionNewton(this->position * totalDistance, distance, distance / 100.0) / totalDistance;
            double actualDistance = p.arcLengthAntiDerivative(newPosition * totalDistance) - p.arcLengthAntiDerivative(this->position * totalDistance);
            int newLane = this->lane;
            if (newPosition >= this->changeLaneFinishPosition) {
                newPosition = this->changeLaneFinishPosition;
                newLane = this->changeToLane;
            }
            return new LinearRoadPosition(this->linearRoad, this->movingFixedToFree, newLane, newPosition, distance - actualDistance, this->changeToLane, this->changeLaneStartPosition, this->changeLaneFinishPosition);
        } else {
			VehiclePosition::SinCurveParameters p(this->changeLaneFinishPosition * totalDistance, this->changeToLane * Sizes::LANE_WIDTH, this->changeLaneStartPosition * totalDistance, this->lane * Sizes::LANE_WIDTH, fabs(this->changeLaneFinishPosition - this->changeLaneStartPosition) * totalDistance);
            double newPosition = p.getLengthAfterPositionNewton(this->position * totalDistance, -distance, distance / 100.0) / totalDistance;
            double actualDistance = p.arcLengthAntiDerivative(this->position * totalDistance) - p.arcLengthAntiDerivative(newPosition * totalDistance);
            int newLane = this->lane;
            if (newPosition <= this->changeLaneFinishPosition) {
                newPosition = this->changeLaneFinishPosition;
                newLane = this->changeToLane;
            }
            return new LinearRoadPosition(this->linearRoad, this->movingFixedToFree, newLane, newPosition, distance - actualDistance, this->changeToLane, this->changeLaneStartPosition, this->changeLaneFinishPosition);
        }
    }
    float t = distance / totalDistance;
    if (std::isnan(t)) {
        t = 0;
    }
    return new LinearRoadPosition(this->linearRoad, this->movingFixedToFree, this->lane, this->position + (this->movingFixedToFree ? t : -t), 0, this->changeToLane, this->changeLaneStartPosition, this->changeLaneFinishPosition);
}

bool LinearRoadPosition::isAtEnd() const {
    if (!this->movingFixedToFree && this->position <= 0) {
        return this->position <= getBeginningPosition(true);
    } else if (this->movingFixedToFree && this->position >= 1) {
        Intersection *freeIntersection = this->linearRoad->getFreeIntersection();
        if (freeIntersection != nullptr) {
            return this->position >= getBeginningPosition(false);
        } else {
            return true;
        }
    }
    return false;
}

VehiclePosition *LinearRoadPosition::getPathEndConnection(VehicleRoute *route) const {
    const QPointF &start = this->linearRoad->getPoint(0);
    const QPointF &end = this->linearRoad->getPoint(1);
    float totalDistance = sqrt(pow(start.x() - end.x(), 2) + pow(start.y() - end.y(), 2));
    double extraPosition = 0;
	if (!this->movingFixedToFree) {
        extraPosition = getBeginningPosition(true) - this->position;
    } else {
        extraPosition = this->position - getBeginningPosition(false);
    }
	double extraDistance = extraPosition > 0 ? extraPosition * totalDistance : 0;

	int sourceLaneCount = this->linearRoad->getLanes();
	int intersectionLane = this->movingFixedToFree ? sourceLaneCount - 1 - this->lane : this->lane;
    Intersection* intersection = this->movingFixedToFree ? this->linearRoad->getFreeIntersection() : this->linearRoad->getFixedIntersection();
    Direction::Cardinal cardinal = this->movingFixedToFree ? this->linearRoad->getFreeIntersectionDirection() : this->linearRoad->getFixedIntersectionDirection();

	if (intersection != nullptr) {
		const TrafficLight::TrafficLightState &trafficLightState = intersection->getTrafficLight()->getCurrentState();

		if (route != nullptr && route->hasIntersection(intersection)) {
			std::vector<IntersectionLane> validDestinations;
			const Direction::Cardinal &c = route->getDirection(intersection, Direction::Cardinal::NORTH);
			int lanes = intersection->getLanes(c);
			for (int i = 0; i < lanes; i++) {
				if (trafficLightState.getLightColor(cardinal, intersectionLane, c, i) != TrafficLightColor::ILLEGAL) {
					validDestinations.push_back(IntersectionLane(c, i));
				}
			}
			if (validDestinations.size() > 0) {
				IntersectionLane selection = validDestinations.at(qrand() % validDestinations.size());
				return intersection->getNewPosition(cardinal, selection.getDirection(), intersectionLane, selection.getLane(), -INFINITY, extraDistance);
			}
		} else {
			std::vector<IntersectionLane> validDestinations;
			for (int i = 0; i < Direction::DIRECTIONS_COUNT; i++) {
				const Direction::Cardinal &c = Direction::getCardinal(i);
				int lanes = intersection->getLanes(c);
				for (int i = 0; i < lanes; i++) {
					if (trafficLightState.getLightColor(cardinal, intersectionLane, c, i) != TrafficLightColor::ILLEGAL) {
						validDestinations.push_back(IntersectionLane(c, i));
					}
				}
			}
			if (validDestinations.size() > 0) {
				IntersectionLane selection = validDestinations.at(qrand() % validDestinations.size());
				return intersection->getNewPosition(cardinal, selection.getDirection(), intersectionLane, selection.getLane(), -INFINITY, extraDistance);
			}
		}
	}
	return nullptr;
}

bool LinearRoadPosition::requestLaneChangeLeft(double distance, Vehicle *vehicle) {
	return requestLaneChange(this->movingFixedToFree ? this->lane - 1 : this->lane + 1, distance, vehicle);
}

bool LinearRoadPosition::requestLaneChangeRight(double distance, Vehicle *vehicle) {
	return requestLaneChange(this->movingFixedToFree ? this->lane + 1 : this->lane - 1, distance, vehicle);
}

bool LinearRoadPosition::requestLaneChange(int newLane, double distance, Vehicle *vehicle) {
	if (distance <= 0 || std::isnan(distance) || this->lane != this->changeToLane || !std::isnan(this->changeLaneStartPosition) || !std::isnan(this->changeLaneFinishPosition)) {
		return false;
	} else if (this->position <= 0 || this->position >= 1) {
		return false;
	} else if (newLane == this->lane) {
		return true;
	} else {
		double vehicleHalfLength = vehicle->getVehicleLength() / 2.0;
		double pathLength = getPathLength();
		double distanceAlongPath = getDistanceAlongPath();
		double minimumDistanceRequiredForChange1 = distanceAlongPath + vehicleHalfLength + distance + Distances::LANE_CHANGE_BUFFER_DISTANCE;
		double minimumDistanceRequiredForChange2 = distanceAlongPath - vehicleHalfLength - distance - Distances::LANE_CHANGE_BUFFER_DISTANCE;
		if (this->movingFixedToFree && minimumDistanceRequiredForChange1 >= pathLength) {
			return false;
		} else if (!this->movingFixedToFree && minimumDistanceRequiredForChange2 <= 0) {
			return false;
		} else {
			const std::array<int, 2> &availableLanes = this->linearRoad->getEnteringLanesRange(this->movingFixedToFree);
			if (newLane >= availableLanes.at(0) && newLane <= availableLanes.at(1)) {
				for (auto &v : this->linearRoad->getFixedIntersection()->getTrafficEngine()->getVehicles()) {
					if (vehicle != v) {
						VehiclePosition *position = v->getPosition();
						if (position->getPath() == this->linearRoad) {
							int lane = position->getLane();
							int changeToLane = position->getChangeToLane();
							if (lane == newLane || changeToLane == this->lane || changeToLane == newLane) {
								double positionDistance = position->getDistanceAlongPath();
								double vLengthHalf = v->getVehicleLength() / 2.0;
								if (this->movingFixedToFree && positionDistance + vLengthHalf >= distanceAlongPath - vehicleHalfLength - Distances::LANE_CHANGE_BUFFER_DISTANCE && positionDistance - vLengthHalf <= minimumDistanceRequiredForChange1) {
									return false;
								} else if (!this->movingFixedToFree && positionDistance - vLengthHalf <= distanceAlongPath + vehicleHalfLength + Distances::LANE_CHANGE_BUFFER_DISTANCE && positionDistance + vLengthHalf >= minimumDistanceRequiredForChange2) {
									return false;
								}
							}
						}
					}
				}
				const QPointF &start = this->linearRoad->getPoint(0);
				const QPointF &end = this->linearRoad->getPoint(1);
				float totalDistance = sqrt(pow(start.x() - end.x(), 2) + pow(start.y() - end.y(), 2));
				double distanceRemaining = this->movingFixedToFree ? totalDistance * (1 - this->position) : totalDistance * this->position;
				if (distanceRemaining >= distance) {
					this->changeLaneStartPosition = this->position;
					this->changeToLane = newLane;
					this->changeLaneFinishPosition = this->position + distance / totalDistance * (this->movingFixedToFree ? 1 : -1);
					return true;
				}
			}
		}
	}
	return false;
}

double LinearRoadPosition::getDistanceAlongPath() const {
	return (this->movingFixedToFree ? this->position - getBeginningPosition(true) : getBeginningPosition(false) - this->position) * this->linearRoad->getLength();
}

double LinearRoadPosition::getPathLength() const {
    return (getBeginningPosition(false) - getBeginningPosition(true)) * this->linearRoad->getLength();
}

TrafficLight *LinearRoadPosition::getEndOfPathTrafficLight() const {
    Intersection* intersection = this->movingFixedToFree ? this->linearRoad->getFreeIntersection() : this->linearRoad->getFixedIntersection();
    return intersection != nullptr ? intersection->getTrafficLight() : nullptr;
}

VehiclePosition *LinearRoadPosition::getEndOfPathTrafficLightPosition() const {
    bool movingFixedToFree = this->movingFixedToFree;
    return this->linearRoad->getNewPosition(movingFixedToFree, this->lane, getBeginningPosition(!movingFixedToFree), 0);
}

VehiclePosition *LinearRoadPosition::clone() const {
	return this->linearRoad->getNewPosition(this->movingFixedToFree, this->lane, this->position, 0);
}

float LinearRoadPosition::getBeginningPosition(bool fixedEnd) const {
	return this->linearRoad->getBeginningPosition(this->lane, fixedEnd);
}

bool LinearRoadPosition::isChangingLanes() const {
    return !std::isnan(this->changeLaneStartPosition) && !std::isnan(this->changeLaneFinishPosition) && this->changeLaneStartPosition != this->changeLaneFinishPosition && (this->movingFixedToFree == (this->changeLaneFinishPosition > this->changeLaneStartPosition));
}
