#ifndef LINEARROAD_H
#define LINEARROAD_H

#include "direction.h"
#include "road.h"
#include "vehicleposition.h"

class Vehicle;

class LinearRoad : public Road {
	friend class LinearRoadPosition;

	QRectF boundingRectangle;
public:
    LinearRoad(Intersection *fixedIntersection, Direction::Cardinal fixedIntersectionDirection, double speedLimit, QGraphicsItem *parent = 0);
    virtual ~LinearRoad();

    virtual QPointF getPoint(float t) const override;
    virtual QPointF getTangent(float t) const override;

    virtual bool isValid() const override;

	virtual void updateSize() override;
    virtual QRectF boundingRect() const override;

    virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

	virtual VehiclePosition* getNewPosition(bool movingFixedToFree, int lane, float t, float bufferDistance) override;

	virtual double getLength() const override;
	virtual double getPathLength(int lane) const override;

protected:
	virtual float getBeginningPosition(int lane, bool fixedEnd) const override;

private:
    QPointF getStart() const;
    QPointF getEnd() const;

    float getFixedIntersectionSize() const;
    int getFixedIntersectionLanes() const;

    /**
     * @brief getParallelLineAngle Returns the angle passing through a point at a distance of d1 from p1 and a point at a distance of d2 from p2 that is perpendicular to the angle from each fixed point
     * @param p1 the first fixed point
     * @param p2 the second fixed point
     * @param d1 the distance from p1
     * @param d2 the distance from p2
     * @return the angle in radians
     */
    float getParallelLineAngle(const QPointF &p1, const QPointF &p2, float d1, float d2) const;

    /**
     * @brief getConnectedFreeEndPoints Returns an array with the start and end points of a straight road connecting the fixed and free intersections
     * The end points are in the parent component's coordinates. If there is no free intersection or the created road is invalid then all values of the points returned are NAN
     * @return an array with the start end point at index 0 and end end point at index 1
     */
    std::array<QPointF, 2> getConnectedFreeEndPoints() const;

	virtual void addHighlightPath(QPainterPath &path, const QPointF &center, double startAngle, double angleLength, const Road::HighlightOption &highlightOption, bool arcLeft) const;
};

class LinearRoadPosition : public VehiclePosition {
	LinearRoad *linearRoad;
	bool movingFixedToFree;
	int lane;
	float position;
	int changeToLane;
	double changeLaneStartPosition;
	double changeLaneFinishPosition;

public:
    LinearRoadPosition(LinearRoad *linearRoad, bool movingFixedToFree, int lane, float t, float bufferDistance, int changeToLane, double changeLaneStartPosition, double changeLaneFinishPosition);
    virtual ~LinearRoadPosition();

    virtual VehiclePath* getPath() const override;
	virtual VehiclePath* getNextPath() const override;

    virtual int getLane() const override;
    virtual int getChangeToLane() const override;
	virtual int getNextPathEnterLane() const override;
    virtual Direction::Cardinal getSourceDirection() const override;
    virtual Direction::Cardinal getDestinationDirection() const override;

    virtual QPointF getLocation() const override;
    virtual float getDirection() const override;

	virtual VehiclePosition* getNextPosition(double distance) const override;

    virtual bool isAtEnd() const override;

	virtual VehiclePosition* getPathEndConnection(VehicleRoute *route = nullptr) const override;

	virtual bool requestLaneChangeLeft(double distance, Vehicle *vehicle) override;
	virtual bool requestLaneChangeRight(double distance, Vehicle *vehicle) override;
	virtual bool requestLaneChange(int newLane, double distance, Vehicle *vehicle) override;

    virtual double getDistanceAlongPath() const override;

    virtual double getPathLength() const override;

    virtual TrafficLight* getEndOfPathTrafficLight() const override;

    virtual VehiclePosition* getEndOfPathTrafficLightPosition() const override;

    virtual VehiclePosition* clone() const override;

private:
    float getBeginningPosition(bool fixedEnd) const;
    bool isChangingLanes() const;
};

#endif // LINEARROAD_H
