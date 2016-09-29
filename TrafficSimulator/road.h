#ifndef ROAD_H
#define ROAD_H

#include "direction.h"
#include "vehiclepath.h"

#include <QGraphicsPathItem>

class Intersection;
class Vehicle;
class VehiclePosition;

class QPainter;
class QStyleOptionGraphicsItem;
class QWidget;

class Road : public QGraphicsPathItem, public VehiclePath {
public:
	enum class HighlightOption {
		FREE_TO_FIXED = -1,
		NONE = 0,
		FIXED_TO_FREE = 1,
		ALL = 2
	};

private:
	float freeEndX;
	float freeEndY;
	Intersection* fixedIntersection;
	Direction::Cardinal fixedIntersectionDirection;
	Intersection* freeEndIntersection;
	Direction::Cardinal freeEndIntersectionDirection;
	HighlightOption highlightOption;

public:
	virtual ~Road();
	using QGraphicsItem::prepareGeometryChange;

    /**
     * @brief getPoint Gets the location of the middle path running along with the road in the parent item's coordinates
     * @param t the time along the road in the interval [0, 1] where represents the start of the road and 1 represents the end
     * @return the location of the path at a certain time in the parent item's coordinates
     */
    virtual QPointF getPoint(float t) const = 0;
    /**
     * @brief getTangent Gets a point representing the tangent vector in the parent item's coordinates
     * @param t the time along the road in the interval [0, 1] where represents the start of the road and 1 represents the end
     * @return a point representing the tangent vector in the parent item's coordinates
     */
    virtual QPointF getTangent(float t) const = 0;

    /**
     * @brief isValid Tests if this is a valid road
     * @return true if the road is valid; otherwise, false
     */
    virtual bool isValid() const = 0;

    virtual float getFreeEndX() const;
    virtual float getFreeEndY() const;
    virtual void setFreeEnd(float x, float y);
    virtual void setFreeEnd(Intersection* freeEndIntersection, const Direction::Cardinal &freeEndIntersectionDirection);
    virtual void clearFreeEnd();

    /**
     * @brief setFreeEndAtEnd Updates the location of the free end to be the same location as getPoint(1)
     * The location of the free end and getPoint(1) may not be the same when the free end is no longer 'free' and is attached to another intersection.
     */
    virtual void setFreeEndAtEnd();

    virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) = 0;

    virtual Intersection *getFixedIntersection() const;
    virtual Direction::Cardinal getFixedIntersectionDirection() const;
    virtual Intersection *getFreeIntersection() const;
    virtual Direction::Cardinal getFreeIntersectionDirection() const;

	virtual HighlightOption getHighlightOption() const;
	virtual void setHighlightOption(const HighlightOption &option);

    /**
     * @brief getLanes Gets the number of lanes of the road equal to the number of lanes of the connected intersection
     * @return the number of lanes
     */
    virtual int getLanes() const;

	VehiclePosition* getEndOfPathTrafficLightPosition(bool movingFixedToFree, int lane);
	virtual VehiclePosition* getNewPosition(bool movingFixedToFree, int lane, float t, float bufferDistance) = 0;

    /**
     * @brief getDirectionDividerLane Gets the lane that divides the opposing directions of traffic of the road
     * If the value is greater than 0, the traffic is right-handed and the value represents the number of lanes with traffic directed from the fixed intersection to the free intersection
	 * If the value is less than 0, the traffic is left-handed and the absolute of the value represents the number of lanes with traffic directed from the fixed intersection to the free intersection
	 * If the value is equal to 0, the traffic is one-way from the free intersection to the fixed intersection.
	 * It is assumed that all traffic travelling in the same direction is in adjacent lanes.
     * @return a value representing the directions of the traffic where its absolute value is the number of lanes in the direction of the fixed intersection to the free intersection
     */
	virtual int getDirectionDividerLane() const;

	virtual std::array<int, 2> getEnteringLanesRange(bool fixedEnd) const;
	virtual std::array<int, 2> getExitingLanesRange(bool fixedEnd) const;

	int getEnteringLanesCount(bool fixedEnd) const;
	int getExitingLanesCount(bool fixedEnd) const;
	bool isLaneMovingToFreeEnd(int lane) const;

    virtual double getLength() const = 0;
	virtual double getPathLength(int lane) const = 0;

	virtual void updateSize() = 0;
	virtual void update();

protected:
	Road(Intersection *fixedIntersection, Direction::Cardinal fixedIntersectionDirection, double speedLimit, QGraphicsItem *parent = nullptr);

	virtual float getBeginningPosition(int lane, bool fixedEnd) const = 0;

	virtual void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
};

#endif // ROAD_H
