#ifndef TRAFFICENGINE_H
#define TRAFFICENGINE_H

#include <QObject>
#include <QVector>

#include <functional>
#include <memory>

#include <QElapsedTimer>

class Intersection;
class MainWindow;
class Road;
class RoadObject;
class RouteSelectorDialog;
class Vehicle;
class VehiclePath;
class VehicleRoute;

class QDialog;
class QGraphicsItem;
class QPointF;
class QTableWidget;

class TrafficEngine : public QObject {
	Q_OBJECT

	MainWindow *mainWindow;
	qint64 totalPausedTime;
	QElapsedTimer pauseTimer;
	VehicleRoute* highlightedRoute;

	bool rightHandDrive;

	std::unique_ptr<QDialog> intersectionsInfoDialog;
	QTableWidget* intersectionsInfoTable;
	std::unique_ptr<QDialog> vehiclesInfoDialog;
	QTableWidget* vehiclesInfoTable;
	std::unique_ptr<RouteSelectorDialog> routeSelectorDialog;
	Vehicle *routeSelectorVehicle;
	bool enableRouteVehicle;

	QVector<Intersection*> intersections;
	QVector<Vehicle*> vehicles;

public:
	class Models;

	TrafficEngine(MainWindow *mainWindow);
    virtual ~TrafficEngine();

	bool isPaused() const;
	void setPauseState(bool isPaused);
	qint64 getPausedTime() const;

	bool isRightHandDrive() const;
	void setRightHandDrive(bool rightHandDrive);

	void addIntersection(Intersection *intersection);

	/**
	 * @brief addVehicle Adds the vehicle to this traffic engine
	 * The vehicle will be managed by this traffic engine
	 * @param vehicle the vehicle to be added
	 */
	void addVehicle(Vehicle* vehicle);

    const QVector<Intersection*> getIntersections() const;
	const QVector<Road*> getRoads() const;
	unsigned int getRoadCount() const;

    const QVector<Vehicle*> getVehicles() const;
	const QVector<Vehicle*> getVehicles(VehiclePath *vehiclePath) const;
	void setVehicleModel(const TrafficEngine::Models &m) const;

    int getVehicleCount() const;

    /**
     * @brief getIntersectionBelowMouse Returns the top intersection that contains the coordinates of the mouse
     * @param mouse the position of the mouse relative to the parent
     * @param skipIntersection an intersection that will not be considered when searching for candidate intersections or nullptr if no intersections should be skipped
     * @return the top intersection containing the coordinates of the mouse
     */
    Intersection* getIntersectionBelowMouse(const QPointF &mouse, Intersection *skipIntersection) const;

	virtual void updateVehicles();
	virtual void removeVehicle(Vehicle *vehicle);
	virtual void removeVehiclesOnPath(VehiclePath* path);

	void toggleIntersectionsInformationTableVisible();
	void setIntersectionsInformationTableVisible(bool visible);
	void updateIntersectionsInfoTable(Intersection* intersection = nullptr);
	void toggleVehiclesInformationTableVisible();
	void setVehiclesInformationTableVisible(bool visible);
	void updateVehiclesInfoTable(Vehicle* vehicle = nullptr);
	void updateObjectAheadInfo(Vehicle* vehicle, const std::string &info);
	VehicleRoute *getHighlightedRoute() const;
	void highlightVehicle(Vehicle *vehicle);
	void highlightRoute(Vehicle *vehicle);
	void updateVehicleRoutes() const;

	void intersectionClicked(Intersection *intersection);
	void roadClicked(Road *road);

	void toggleFlashingIntersections();
	void setFlashingIntersections(bool isFlashing);
	void toggleHighlightRouteSelectorVehicleRoute();
	void setEnableRouteVehicle(bool enable);
	void clearRouteVehicle();

private:
	virtual void removeVehicle(int vehicleIndex);
	void setVehicleDestination(VehiclePath *destination);
	void vehiclePathClicked(VehiclePath *path);

private slots:
	void intersectionTableClicked(int row, int col);
	void vehiclesTableClicked(int row, int col);
	void setVehicleDestination(Vehicle *vehicle);
};

class TrafficEngine::Models {
public:
	static const TrafficEngine::Models INTELLIGENT_DRIVER_MODEL;
	static const TrafficEngine::Models CHANDLER;
	static const TrafficEngine::Models GAZIS;
	static const TrafficEngine::Models EDIE;
	static const TrafficEngine::Models GM_MODEL;
	static const TrafficEngine::Models SIMPLE;
	static const std::array<TrafficEngine::Models, 6> VALUES;
private:
	static unsigned int idAssigner;

	unsigned int id;
	std::string name;
	std::function<std::pair<double, double>(Vehicle *, double, double, double, double, RoadObject*, double, double, double, double, double)> modelFunction;

	Models(const std::string &name, const std::function<std::pair<double, double>(Vehicle*, double, double, double, double, RoadObject*, double, double, double, double, double)> &modelFunction);

	static double intelligentDriverModelFunction(double x1, double v1, double x2, double v2, double acceleration, double deceleration, double preferredSpeed, double, double safetyDistance);

	static std::pair<double, double> basicAccelerationModel(double accelerationModelValue, Vehicle *currentVehicle, double currentPosition, double speed, double acceleration, double deceleration, RoadObject *objectAhead, double objectAheadPosition, double speedLimit, double timeElapsed, double safetyDistance, double remainingDistance);
public:
	std::string getName() const;
	std::pair<double, double> apply(Vehicle*, double, double, double, double, RoadObject*, double, double, double, double, double) const;
	bool operator==(const TrafficEngine::Models &m) const;
};

#endif // TRAFFICENGINE_H
