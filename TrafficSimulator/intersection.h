#ifndef INTERSECTION_H
#define INTERSECTION_H

#include "constants.h"
#include "direction.h"
#include "vehiclepath.h"
#include "vehicleposition.h"

#include <QGraphicsItem>

#include <memory>

class IntersectionPosition;
class Road;
class TrafficEngine;
class TrafficLight;

class QGraphicsSceneMouseEvent;
class QPainter;
class QPainterPath;

class Intersection : public QGraphicsItem, public VehiclePath {
	enum class HoverLocation {
		LANES_HORIZONTAL = 0,
		LANES_VERTICAL,
		ROTATE,
		CONNECTION_NORTH,
		CONNECTION_EAST,
		CONNECTION_SOUTH,
		CONNECTION_WEST,
		INSIDE,
		OUTSIDE
	};

	static unsigned int idAssigner;
	const unsigned int ID;
	int highlightedState; // -1 = highlight directions, 0 = no highlight, 1 = highlight all
	Direction::Cardinal highlightSourceDirection;
	Direction::Cardinal highlightDestinationDirection;
	QColor flashingColor;
	TrafficEngine* trafficEngine;
	float angle;
	int horizontalLanes;
	int verticalLanes;
	std::array<Road*, Direction::DIRECTIONS_COUNT> roadConnections;

	std::array<int, Direction::ORIENTATIONS_COUNT> temporaryLanes;
	int temporaryHorizontalLanes;
	int temporaryVerticalLanes;
	float temporaryRotateAngle;
	int temporaryRoadExtension;
	HoverLocation hoverLocation;
	Road *temporaryRoad;
	Intersection *temporaryHoverIntersection;
	std::unique_ptr<TrafficLight> trafficLight;

public:
	Intersection(TrafficEngine* trafficEngine, int horizontalLanes = 2, int verticalLanes = 2, double speedLimit = Speeds::INTERSECTION_SPEED, QGraphicsItem *parent = 0);
    virtual ~Intersection();
    unsigned int getID() const;
    TrafficEngine* getTrafficEngine();

    QRectF boundingRect() const override;
    QPainterPath shape() const override;
    int getRectWidth() const;
    int getRectHeight() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
	bool isHighlighted() const;
	void setHighlighted(bool isHighlighted);
	void setHighlighted(const Direction::Cardinal &source, const Direction::Cardinal &destination);
	bool isFlashing() const;
	void setFlashing(const QColor &color);
	void stopFlashing();

    /**
     * @brief getAngle Gets the angle of rotation in degrees.
     * @return  the angle of rotation in degrees
     */
    float getAngle() const;

    int getLanes(const Orientation &orientation) const;
    int getLanes(const Direction::Cardinal &cardinal) const;
    void setLanes(const Orientation &orientation, int lanes);
    float getTotalLanesWidth(const Orientation &orientation) const;

	virtual void setSpeedLimit(double speedLimit) override;
	virtual void setSpeedLimit(double speedLimit, bool updateGUI);

	QVector<Road*> getConnections() const;
	unsigned int getConnectionsCount() const;
	bool hasLaneEntering(const Direction::Cardinal &cardinal) const;
	bool hasLaneExiting(const Direction::Cardinal &cardinal) const;
    Road *getConnection(const Direction::Cardinal &cardinal) const;
	void setConnection(const Direction::Cardinal &cardinal, Road *connection);

    /**
     * @brief getConnectionDirection Gets the perpendicular angle in radians pointing outwards of the intersection relative to the parent item's coordinates.
     * @param cardinal the side of the intersection
     * @return angle in radians in [0, 2 * PI)
     */
    float getConnectionDirection(Direction::Cardinal cardinal) const;

    /**
     * @brief getConnectionCenter Gets the location in local coordinates of the middle of the side of the intersection.
     * @param cardinal the side of the intersection
     * @return the location of the center in local coordinates
     */
    QPoint getConnectionCenter(Direction::Cardinal cardinal) const;
    QPoint getConnectionLeft(Direction::Cardinal cardinal) const;
    QPoint getConnectionRight(Direction::Cardinal cardinal) const;

	IntersectionPosition* getEndOfPathPosition(const IntersectionLane &sourceLane, const IntersectionLane &destinationLane);
	virtual IntersectionPosition* getNewPosition(const Direction::Cardinal &sourceDirection, const Direction::Cardinal &destinationDirection, int lane, int destinationLane, float t, double bufferDistance);

    virtual TrafficLight *getTrafficLight() const;

	double getPathLength(const IntersectionLane &sourceLane, const IntersectionLane &destinationLane);

	bool isEntering(const IntersectionLane &lane) const;
	bool isExiting(const IntersectionLane &lane) const;

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
    void hoverEnterEvent(QGraphicsSceneHoverEvent *event) override;
    void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;
    void hoverMoveEvent(QGraphicsSceneHoverEvent *event) override;

    void advance(int phase) override;

private:
	bool isChangeLanes(const HoverLocation &hoverLocation) const;
	bool isRoadConnection(const HoverLocation &hoverLocation) const;
	HoverLocation toHoverLocation(const Direction::Cardinal &c) const;
	Direction::Cardinal toCardinalDirection(const HoverLocation &hoverLocation) const;

	QRectF getHoverSideRectangle(const Direction::Cardinal &c) const;
    HoverLocation checkHoverLocation(int mouseX, int mouseY, bool checkTransformCircles, int connectionLanes) const;
    void removeTemporaryRoad();

    void hoverMoveEvent(const QPointF &mousePosition, bool checkTransformCircles, int connectionLanes);

    void removeRoadConnection(const Direction::Cardinal &c);

    bool canChangeLaneCount(const Orientation &o) const;

	void prepareConnectionsGeometryChange();
};

class IntersectionPosition : public VehiclePosition {
	friend class Intersection;

	Intersection *intersection;
	Direction::Cardinal sourceDirection;
	Direction::Cardinal destinationDirection;
	int lane;
	int destinationLane;
	float position;
	float errorDistance;
	double bufferedPathLength;

public:
	IntersectionPosition(Intersection *intersection, const Direction::Cardinal &sourceDirection, const Direction::Cardinal &destinationDirection, int lane, int destinationLane, float t, float bufferDistance, float errorDistance=0);
    virtual ~IntersectionPosition();

	virtual VehiclePath* getPath() const override;
	virtual VehiclePath* getNextPath() const override;

    static int getLane(const IntersectionLane &intersectionLane);
	static IntersectionLane getLane(int intersectionLane);
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

    SinCurveParameters getSinCurveParameters() const;

	EllipseParameters getEllipseParameters() const;

private:
	static double calculatePathLength(Intersection *intersection, const IntersectionLane &sourceLane, const IntersectionLane &destinationLane);
	static SinCurveParameters getSinCurveParameters(Intersection *intersection, const IntersectionLane &sourceLane, int destinationLane);
	static EllipseParameters getEllipseParameters(Intersection *intersection, const IntersectionLane &sourceLane, const IntersectionLane &destinationLane);
};

class IntersectionLane {
public:
    IntersectionLane(Direction::Cardinal direction, int lane);
    Direction::Cardinal getDirection() const;
    int getLane() const;
    void setDirection(Direction::Cardinal direction);
    void setLane(int lane);

	bool operator==(const IntersectionLane &lane) const;
private:
    Direction::Cardinal direction;
    int lane;
};
QDebug operator<<(QDebug debug, const IntersectionLane &lane);

#endif // INTERSECTION_H
