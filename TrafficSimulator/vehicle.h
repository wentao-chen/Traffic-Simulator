#ifndef VEHICLE_H
#define VEHICLE_H

#include "direction.h"
#include "roadobject.h"
#include "trafficengine.h"

#include <QElapsedTimer>
#include <QGraphicsItem>

#include <memory>

class Intersection;
class TrafficEngine;
class VehiclePath;
class VehiclePosition;
class VehicleRoute;

class QTableWidget;
class QPainter;
class QStyleOptionGraphicsItem;
class QWidget;

class Vehicle : public QGraphicsItem, public RoadObject {
	static unsigned int idAssigner;
	static int highlightBorderSize;

	const unsigned int id;
	std::string name;
	TrafficEngine::Models model;
	TrafficEngine *trafficEngine;
	VehiclePosition *temporaryPosition;
	VehiclePosition *temporaryNextPathPosition; // Stores the next position for when vehicle front crossed line but vehicle position did not
	double temporarySpeed;
	bool highlight;
	QElapsedTimer timer;
	std::unique_ptr<VehicleRoute> route;

public:
	Vehicle(TrafficEngine *trafficEngine, const std::string &name, const TrafficEngine::Models &model, double vehicleWidth, double vehicleLength, double acceleration, double comfortableDeceleration, double safetyDistance, QGraphicsItem *parent = 0);
    virtual ~Vehicle();

	unsigned int getID() const;
	std::string getName() const;
	TrafficEngine::Models getModel() const;
	void setModel(TrafficEngine::Models m);

	QPainterPath shape() const override;
	QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

    virtual void setPosition(VehiclePosition *currentPosition) override;
    virtual bool isOnPath(VehiclePath* path);

    virtual void updatePosition();

    virtual void prepareToMove(float timeElapsed);
    virtual void move();
	virtual void move(float timeElapsed);

    /**
     * @brief getNextDirection Gets the destination direction towards which the vehicle should turn in the intersection
     * If an invalid direction is returned where a valid direction is possible, a random valid direction is chosen.
     * @param intersection the intersection in which the turn is made
     * @param validDirections the set of all valid directions which cannot be empty
     * @return the destination direction
     */
    virtual Direction::Cardinal getNextDirection(Intersection* intersection, QVector<Direction::Cardinal> validDirections);

    /**
     * @brief getNextLane Gets the destination lane towards which the vehicle should turn in the intersection
     * If an invalid lane is returned where a valid direction is possible, a random valid lane is chosen.
     * @param intersection the intersection in which the turn is made
     * @param destinationDirection the direction that the vehicle is turning towards
     * @param validLanes the set of all valid lanes which cannot be empty
     * @return the destination lane
     */
    virtual int getNextLane(Intersection* intersection, Direction::Cardinal destinationDirection, QVector<int> validLanes);

	virtual void setSpeed(double speed) override;

	virtual std::string getRouteMessage() const;
    virtual VehicleRoute* getRoute() const;
	virtual void clearRoute();
	virtual bool setRoute(VehiclePath *destination);

	bool isHighlighted() const;
	void setHighlighted(bool highlight);

protected:
    void advance(int phase) override;

	void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;

private:
	bool isAtPathEnd(VehiclePosition *currentPosition) const;
	VehiclePosition* getPathEndNextPosition(VehiclePosition *currentPosition) const;
	void setTemporaryNextPathPosition();

	double intelligentDriverModelFunction(double x1, double v1, double x2, double v2, double acceleration, double deceleration, double preferredSpeed, double timeElapsed, double safetyDistance) const;
};

#endif // VEHICLE_H
