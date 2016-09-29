#include "trafficengine.h"

#include "constants.h"
#include "direction.h"
#include "intersection.h"
#include "mainwindow.h"
#include "road.h"
#include "roadobject.h"
#include "routeselectordialog.h"
#include "trafficlight.h"
#include "vehicle.h"
#include "vehicleroute.h"
#include "vehiclepath.h"

#include <QDialog>
#include <QHBoxLayout>
#include <QTableWidget>
#include <QStringList>
#include <QApplication>
#include <QDesktopWidget>
#include <QHeaderView>
#include <QVector>
#include <QString>
#include <QGraphicsScene>
#include <QPushButton>

#include <QDebug>

TrafficEngine::TrafficEngine(MainWindow *mainWindow) :
		QObject(),
		mainWindow(mainWindow),
		totalPausedTime(0),
		pauseTimer(QElapsedTimer()),
		highlightedRoute(nullptr),
		rightHandDrive(true),
		intersectionsInfoDialog(std::unique_ptr<QDialog>(new QDialog(mainWindow))),
		intersectionsInfoTable(new QTableWidget(this->intersectionsInfoDialog.get())),
		vehiclesInfoDialog(std::unique_ptr<QDialog>(new QDialog(mainWindow))),
		vehiclesInfoTable(new QTableWidget(this->vehiclesInfoDialog.get())),
		routeSelectorDialog(new RouteSelectorDialog(this, this->vehiclesInfoDialog.get())),
		routeSelectorVehicle(nullptr),
		enableRouteVehicle(false) {
	QHBoxLayout *intersectionsDialogLayout = new QHBoxLayout(this->intersectionsInfoDialog.get());
	this->intersectionsInfoDialog->setMinimumSize(550, 200);
	this->intersectionsInfoDialog->move(100, 600);
	this->intersectionsInfoDialog->setLayout(intersectionsDialogLayout);
	intersectionsDialogLayout->addWidget(this->intersectionsInfoTable);

	this->intersectionsInfoTable->setRowCount(0);
	this->intersectionsInfoTable->setColumnCount(4);
	this->intersectionsInfoTable->setHorizontalHeaderLabels(QStringList() << "#" << "Connections" << "Max Speed" << "State");
	this->intersectionsInfoTable->verticalHeader()->setVisible(false);
	this->intersectionsInfoTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
	this->intersectionsInfoTable->setSelectionBehavior(QAbstractItemView::SelectRows);
	this->intersectionsInfoTable->setSelectionMode(QAbstractItemView::SingleSelection);
	this->intersectionsInfoTable->setShowGrid(false);
	this->intersectionsInfoTable->setGeometry(QApplication::desktop()->screenGeometry());
	this->intersectionsInfoTable->connect(this->intersectionsInfoTable, SIGNAL(cellClicked(int,int)), this, SLOT(intersectionTableClicked(int, int)));

	QHBoxLayout *vehiclesDialogLayout = new QHBoxLayout(this->vehiclesInfoDialog.get());
	this->vehiclesInfoDialog->setMinimumSize(550, 200);
	this->vehiclesInfoDialog->move(100, 200);
	this->vehiclesInfoDialog->setLayout(vehiclesDialogLayout);
	this->vehiclesInfoDialog->connect(this->vehiclesInfoDialog.get(), &QDialog::rejected, [this]{
		this->routeSelectorDialog->hide();
	});
	vehiclesDialogLayout->addWidget(this->vehiclesInfoTable);

	this->vehiclesInfoTable->setRowCount(0);
	this->vehiclesInfoTable->setColumnCount(5);
	this->vehiclesInfoTable->setHorizontalHeaderLabels(QStringList() << "#" << "Name" << "Speed" << "Vehicle Ahead" << "Destination");
	this->vehiclesInfoTable->verticalHeader()->setVisible(false);
	this->vehiclesInfoTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
	this->vehiclesInfoTable->setSelectionBehavior(QAbstractItemView::SelectRows);
	this->vehiclesInfoTable->setSelectionMode(QAbstractItemView::SingleSelection);
	this->vehiclesInfoTable->setShowGrid(false);
	this->vehiclesInfoTable->setGeometry(QApplication::desktop()->screenGeometry());
	this->vehiclesInfoTable->connect(this->vehiclesInfoTable, SIGNAL(cellClicked(int,int)), this, SLOT(vehiclesTableClicked(int, int)));
}

TrafficEngine::~TrafficEngine() {
	for (int i = 0; i < this->vehicles.size(); i++) {
        delete this->vehicles.at(i);
	}
}

bool TrafficEngine::isPaused() const {
	return this->pauseTimer.isValid();
}

void TrafficEngine::setPauseState(bool isPaused) {
	if (isPaused) {
		this->pauseTimer.restart();
	} else {
		this->totalPausedTime += this->pauseTimer.nsecsElapsed();
		this->pauseTimer.invalidate();
	}
}

qint64 TrafficEngine::getPausedTime() const {
	return this->totalPausedTime + (this->pauseTimer.isValid() ? this->pauseTimer.nsecsElapsed() : 0);
}

bool TrafficEngine::isRightHandDrive() const {
	return this->rightHandDrive;
}

void TrafficEngine::setRightHandDrive(bool rightHandDrive) {
	if (this->rightHandDrive != rightHandDrive) {
		this->rightHandDrive = rightHandDrive;
		for (auto &r : this->intersections) {
			r->update();
		}
		for (auto &r : getRoads()) {
			r->update();
		}
	}
}

void TrafficEngine::addIntersection(Intersection *intersection) {
	this->intersections.push_back(intersection);

	this->intersectionsInfoTable->setRowCount(this->intersections.size());
	this->intersectionsInfoTable->setItem(this->intersections.size() - 1, 0, new QTableWidgetItem(QString::number(intersection->getID())));
	this->intersectionsInfoTable->setItem(this->intersections.size() - 1, 1, new QTableWidgetItem(QString::number(intersection->getConnectionsCount())));
	this->intersectionsInfoTable->setCellWidget(this->intersections.size() - 1, 2, intersection->getTrafficLight()->getSpeedLimitTextBox());
	this->intersectionsInfoTable->setCellWidget(this->intersections.size() - 1, 3, intersection->getTrafficLight()->getLightControlComboBox());
}

void TrafficEngine::addVehicle(Vehicle *vehicle) {
	this->vehicles.push_back(vehicle);
	unsigned int roadsCount = getRoadCount();
	if (roadsCount > 0) {
		int currentRoad = 0;
		int roadSelection = qrand() % roadsCount;
		bool selectedRoad = false;
		for (auto &intersection : this->intersections) {
			for (int i = 0; i < Direction::DIRECTIONS_COUNT; i++) {
				Road* road = intersection->getConnection(Direction::getCardinal(i));
				if (road != nullptr) {
					int totalLanes = road->getLanes();
					if (totalLanes > 0 && currentRoad >= roadSelection) {
						int lane = qrand() % totalLanes;
						int divider = road->getDirectionDividerLane();
						vehicle->setPosition(road->getNewPosition(divider > 0 ? lane >= totalLanes - divider : lane < -divider, lane, 0.5, 0));
						selectedRoad = true;
						break;
					}
					currentRoad++;
				}
			}
			if (selectedRoad) {
				break;
			}
		}
	}
	this->vehiclesInfoTable->setRowCount(this->vehicles.size());
	this->vehiclesInfoTable->setItem(this->vehicles.size() - 1, 0, new QTableWidgetItem(QString::number(vehicle->getID())));
	this->vehiclesInfoTable->setItem(this->vehicles.size() - 1, 1, new QTableWidgetItem(QString::fromStdString(vehicle->getName())));
	this->vehiclesInfoTable->setItem(this->vehicles.size() - 1, 2, new QTableWidgetItem(QString::number(vehicle->getSpeed())));
	this->vehiclesInfoTable->setItem(this->vehicles.size() - 1, 3, new QTableWidgetItem("No Vehicle Ahead"));
	QPushButton *button = new QPushButton(QString::fromStdString(vehicle->getRouteMessage()), this->vehiclesInfoTable);
	this->vehiclesInfoTable->setCellWidget(this->vehicles.size() - 1, 4, button);
	connect(button, &QPushButton::clicked, [this, vehicle]{
		setVehicleDestination(vehicle);
	});
}

const QVector<Intersection *> TrafficEngine::getIntersections() const {
	return this->intersections;
}

const QVector<Road *> TrafficEngine::getRoads() const {
	QVector<Road *> roads;
	for (auto &intersection : this->intersections) {
		roads.append(intersection->getConnections());
	}
	return roads;
}

unsigned int TrafficEngine::getRoadCount() const {
	unsigned int count = 0;
	for (int i = 0; i < this->intersections.size(); i++) {
		count += this->intersections.at(i)->getConnectionsCount();
	}
	return count;
}

const QVector<Vehicle *> TrafficEngine::getVehicles() const {
	return this->vehicles;
}

const QVector<Vehicle *> TrafficEngine::getVehicles(VehiclePath *vehiclePath) const {
	if (vehiclePath != nullptr) {
		const QVector<Vehicle *> &allVehicles = getVehicles();
		QVector<Vehicle *> vehiclesOnPath;
		for (int i = 0; i < allVehicles.size(); i++) {
			Vehicle *v = allVehicles.at(i);
			if (v->getPosition()->getPath() == vehiclePath) {
				vehiclesOnPath.push_back(v);
			}
		}
		return QVector<Vehicle *>(vehiclesOnPath);
	} else {
		return getVehicles();
	}
}

void TrafficEngine::setVehicleModel(const TrafficEngine::Models &m) const {
	for (auto &v : this->vehicles) {
		v->setModel(m);
	}
}

int TrafficEngine::getVehicleCount() const {
    return this->vehicles.size();
}

Intersection *TrafficEngine::getIntersectionBelowMouse(const QPointF &mouse, Intersection *skipIntersection) const {
    for (int i = this->intersections.size() - 1; i >= 0; i--) {
		Intersection *intersection = this->intersections.at(i);
		if ((skipIntersection == nullptr || skipIntersection != intersection) && intersection->contains(intersection->mapFromParent(mouse))) {
			return intersection;
        }
    }
    return nullptr;
}

void TrafficEngine::updateVehicles() {
	for (int i = 0; i < this->vehicles.size(); i++) {
        this->vehicles.at(i)->updatePosition();
	}
}

void TrafficEngine::removeVehicle(Vehicle *vehicle) {
	removeVehicle(this->vehicles.indexOf(vehicle));
}

void TrafficEngine::highlightRoute(Vehicle *vehicle) {
	VehicleRoute *route = vehicle != nullptr ? vehicle->getRoute() : nullptr;
	for (auto &i : this->intersections) {
		i->setHighlighted(false);
	}
	for (auto &r : getRoads()) {
		r->setHighlightOption(Road::HighlightOption::NONE);
	}
	if (this->highlightedRoute == route) {
		this->highlightedRoute = nullptr;
	} else {
		if (route != nullptr) {
			const QVector<Intersection*> &path = route->getIntersections();
			if (path.size() > 0) {
				Intersection *firstIntersection = path.at(0);
				if (firstIntersection != nullptr && vehicle->getPosition()->getNextPath() == firstIntersection) {
					VehiclePath *currentPath = vehicle->getPosition()->getPath();
					for (int i = 0; i < Direction::DIRECTIONS_COUNT; i++) {
						const Direction::Cardinal &d = Direction::getCardinal(i);
						Road *connection = firstIntersection->getConnection(d);
						if (connection != nullptr && connection == currentPath) {
							connection->setHighlightOption(connection->getFixedIntersection() == firstIntersection ? Road::HighlightOption::FREE_TO_FIXED : Road::HighlightOption::FIXED_TO_FREE);
						}
					}
				}
			}
			for (int i = 0; i < path.size(); i++) {
				Intersection *intersection = path.at(i);
				if (route->hasIntersection(intersection)) {
					const Direction::Cardinal &direction = route->getDirection(intersection, Direction::Cardinal::NORTH);
					if (i == 0) {
						if (vehicle->getPosition()->getPath() == intersection) {
							intersection->setHighlighted(vehicle->getPosition()->getSourceDirection(), direction);
						} else {
							intersection->setHighlighted(vehicle->getPosition()->getDestinationDirection(), direction);
						}
					} else {
						intersection->setHighlighted(VehicleRoute::getDirectionOfIntersection(intersection, path.at(i - 1), direction), direction);
					}
					Road *road = intersection->getConnection(direction);
					if (road != nullptr) {
						road->setHighlightOption(road->getFixedIntersection() == intersection ? Road::HighlightOption::FIXED_TO_FREE : Road::HighlightOption::FREE_TO_FIXED);
					}
				}
			}
		}
		this->highlightedRoute = route;
	}
}

void TrafficEngine::updateVehicleRoutes() const {
	const QVector<Vehicle*> vehicles = getVehicles();
	for (auto &v : vehicles) {
		if (v != nullptr) {
			VehicleRoute *route = v->getRoute();
			if (route != nullptr) {
				if (!route->isValid(v->getPosition())) {
					v->setRoute(route->getDestination());
				}
			}
		}
	}
}

void TrafficEngine::removeVehicle(int vehicleIndex) {
	if (vehicleIndex >= 0 && vehicleIndex < this->vehicles.size()) {
		Vehicle *v = this->vehicles.at(vehicleIndex);
		if (v != nullptr) {
			if (v == this->routeSelectorVehicle) { // Remove potential dangling pointer
				this->routeSelectorVehicle = nullptr;
			}
			v->scene()->removeItem(v);
			this->vehicles.remove(vehicleIndex);
			delete v;
			this->vehiclesInfoTable->removeRow(vehicleIndex);
		}
	}
}

void TrafficEngine::intersectionTableClicked(int row, int) {
	if (this->highlightedRoute != nullptr) {
		highlightRoute(nullptr);
	}
	for (int i = 0; i < this->intersections.size(); i++) {
		Intersection *intersection = this->intersections.at(i);
		intersection->setHighlighted(i == row && !intersection->isHighlighted());
	}
}

void TrafficEngine::vehiclesTableClicked(int row, int col) {
	if (row >= 0) {
		Vehicle *v = this->vehicles.at(row);
		if (v != nullptr) {
			highlightRoute(v);
		}
		if (col < 4) {
			for (auto &v2 : this->vehicles) {
				v2->setHighlighted(v == v2 && !v2->isHighlighted());
			}
			this->vehiclesInfoTable->scrollToItem(this->vehiclesInfoTable->item(row, col));
			this->vehiclesInfoTable->selectRow(row);
		}
	}
}

void TrafficEngine::setVehicleDestination(Vehicle *vehicle) {
	this->routeSelectorVehicle = vehicle;
	setEnableRouteVehicle(false);
	if (vehicle != nullptr) {
		this->routeSelectorDialog->setWindowTitle(QString::fromStdString("Route Selector for " + vehicle->getName()));
		this->routeSelectorDialog->show();
		this->routeSelectorDialog->raise();
		this->routeSelectorDialog->activateWindow();
		this->routeSelectorDialog->setDestinationText(vehicle->getRouteMessage());
	}
}

void TrafficEngine::setVehicleDestination(VehiclePath *destination) {
	Vehicle *vehicle = this->routeSelectorVehicle;
	if (vehicle != nullptr && this->enableRouteVehicle) {
		vehicle->setRoute(destination);
		this->routeSelectorDialog->resetNewDestinationButton();
		if (this->highlightedRoute == nullptr) {
			highlightRoute(vehicle);
		}
	}
}

void TrafficEngine::intersectionClicked(Intersection *intersection) {
	return vehiclePathClicked(intersection);
}

void TrafficEngine::roadClicked(Road *road) {
	return vehiclePathClicked(road);
}

void TrafficEngine::toggleFlashingIntersections() {
	for (auto &intersection : getIntersections()) {
		if (intersection->isFlashing()) {
			intersection->stopFlashing();
		} else {
			intersection->setFlashing(QColor(Qt::yellow));
		}
	}
}

void TrafficEngine::setFlashingIntersections(bool isFlashing) {
	if (isFlashing) {
		for (auto &intersection : getIntersections()) {
			intersection->setFlashing(QColor(Qt::yellow));
		}
	} else {
		for (auto &intersection : getIntersections()) {
			intersection->stopFlashing();
		}
	}
}

void TrafficEngine::toggleHighlightRouteSelectorVehicleRoute() {
	highlightRoute(this->routeSelectorVehicle);
}

void TrafficEngine::setEnableRouteVehicle(bool enable) {
	this->enableRouteVehicle = enable;
}

void TrafficEngine::clearRouteVehicle() {
	this->routeSelectorVehicle = nullptr;
}

void TrafficEngine::vehiclePathClicked(VehiclePath *path) {
	setVehicleDestination(path);
}

void TrafficEngine::removeVehiclesOnPath(VehiclePath *path) {
	for (int i = 0; i < this->vehicles.size(); i++) {
		Vehicle *v = this->vehicles.at(i);
		if (v->isOnPath(path)) {
			removeVehicle(i);
            i--;
        }
	}
}

void TrafficEngine::toggleIntersectionsInformationTableVisible() {
	setIntersectionsInformationTableVisible(!this->intersectionsInfoDialog.get()->isVisible());
}

void TrafficEngine::setIntersectionsInformationTableVisible(bool visible) {
	if (visible) {
		this->intersectionsInfoDialog->show();
	} else {
		this->intersectionsInfoDialog->hide();
	}
}

void TrafficEngine::updateIntersectionsInfoTable(Intersection *intersection) {
	if (intersection != nullptr) {
		int i = this->intersections.indexOf(intersection);
		if (i >= 0) {
			this->intersectionsInfoTable->item(i, 0)->setText(QString::number(intersection->getID()));
			this->intersectionsInfoTable->item(i, 1)->setText(QString::number(intersection->getConnectionsCount()));
		}
	} else {
		for (int i = 0; i < this->vehicles.size(); i++) {
			Intersection *inter = this->intersections.at(i);
			this->intersectionsInfoTable->item(i, 0)->setText(QString::number(inter->getID()));
			this->intersectionsInfoTable->item(i, 1)->setText(QString::number(inter->getConnectionsCount()));
		}
	}
}

void TrafficEngine::toggleVehiclesInformationTableVisible() {
	setVehiclesInformationTableVisible(!this->vehiclesInfoDialog.get()->isVisible());
}

void TrafficEngine::setVehiclesInformationTableVisible(bool visible) {
	if (visible) {
		this->vehiclesInfoDialog->show();
	} else {
		this->vehiclesInfoDialog->hide();
	}
}

void TrafficEngine::updateVehiclesInfoTable(Vehicle *vehicle) {
	if (vehicle != nullptr) {
		int i = this->vehicles.indexOf(vehicle);
		if (i >= 0) {
			this->vehiclesInfoTable->item(i, 0)->setText(QString::number(vehicle->getID()));
			this->vehiclesInfoTable->item(i, 1)->setText(QString::fromStdString(vehicle->getName()));
			this->vehiclesInfoTable->item(i, 2)->setText(QString::number(vehicle->getSpeed()));
			QPushButton *button = dynamic_cast<QPushButton *>(this->vehiclesInfoTable->cellWidget(i, 4));
			if (button != nullptr) {
				button->setText(QString::fromStdString(vehicle->getRouteMessage()));
			}
		}
		if (this->routeSelectorVehicle == vehicle) {
			this->routeSelectorDialog->setDestinationText(this->routeSelectorVehicle->getRouteMessage());
		}
	} else {
		for (int i = 0; i < this->vehicles.size(); i++) {
			Vehicle *v = this->vehicles.at(i);
			this->vehiclesInfoTable->item(i, 0)->setText(QString::number(v->getID()));
			this->vehiclesInfoTable->item(i, 1)->setText(QString::fromStdString(v->getName()));
			this->vehiclesInfoTable->item(i, 2)->setText(QString::number(v->getSpeed()));
			QPushButton *button = dynamic_cast<QPushButton *>(this->vehiclesInfoTable->cellWidget(i, 4));
			if (button != nullptr) {
				button->setText(QString::fromStdString(v->getRouteMessage()));
			}
		}
		if (this->routeSelectorVehicle != nullptr) {
			this->routeSelectorDialog->setDestinationText(this->routeSelectorVehicle->getRouteMessage());
		}
	}
}

void TrafficEngine::updateObjectAheadInfo(Vehicle *vehicle, const std::string &info) {
	int i = this->vehicles.indexOf(vehicle);
	if (i >= 0) {
		this->vehiclesInfoTable->item(i, 3)->setText(QString::fromStdString(info));
	}
}

VehicleRoute *TrafficEngine::getHighlightedRoute() const {
	return this->highlightedRoute;
}

void TrafficEngine::highlightVehicle(Vehicle *vehicle) {
	int i = this->vehicles.indexOf(vehicle);
	if (i >= 0) {
		vehicle->setHighlighted(false);
		vehiclesTableClicked(i, 0);
	}
}

const TrafficEngine::Models TrafficEngine::Models::INTELLIGENT_DRIVER_MODEL("Intelligent Driver Model", [](Vehicle *currentVehicle, double currentPosition, double speed, double acceleration, double deceleration, RoadObject *objectAhead, double objectAheadPosition, double speedLimit, double timeElapsed, double safetyDistance, double remainingDistance) {
		const double currentFrontPosition = currentPosition + currentVehicle->getVehicleLength() / 2.0;
		const double vehicleAheadBackPosition = objectAheadPosition - objectAhead->getVehicleLength() / 2.0;
		if (currentFrontPosition < vehicleAheadBackPosition - safetyDistance) {
			const double objectAheadSpeed = objectAhead->getSpeed();
			double temp = intelligentDriverModelFunction(currentFrontPosition, speed, vehicleAheadBackPosition, objectAheadSpeed, acceleration, deceleration, speedLimit, timeElapsed, safetyDistance);
			const double k1 = timeElapsed * speed;
			const double w1 = timeElapsed * temp;

			temp = intelligentDriverModelFunction(currentFrontPosition, speed + w1 / 2.0, vehicleAheadBackPosition, objectAheadSpeed, acceleration, deceleration, speedLimit, timeElapsed, safetyDistance);
			const double k2 = timeElapsed * (speed + w1);
			const double w2 = timeElapsed * temp;

			temp = intelligentDriverModelFunction(currentFrontPosition, speed + w2 / 2.0, vehicleAheadBackPosition, objectAheadSpeed, acceleration, deceleration, speedLimit, timeElapsed, safetyDistance);
			const double k3 = timeElapsed * (speed + w2);
			const double w3 = timeElapsed * temp;

			temp = intelligentDriverModelFunction(currentFrontPosition, speed + w3, vehicleAheadBackPosition, objectAheadSpeed, acceleration, deceleration, speedLimit, timeElapsed, safetyDistance);
			const double k4 = timeElapsed * (speed + w3 * 2);
			const double w4 = timeElapsed * temp;

			return std::pair<double, double>(w1 / 6.0 + w2 / 3.0 + w3 / 3.0 + w4 / 6.0, // velocity changed
											 k1 / 6.0 + k2 / 3.0 + k3 / 3.0 + k4 / 6.0); // distance travelled
		} else {
			return std::pair<double, double>(-deceleration * timeElapsed, // velocity changed
											 std::min(speed * timeElapsed - deceleration * timeElapsed * timeElapsed / 2.0, remainingDistance)); // distance travelled
		}
});
const TrafficEngine::Models TrafficEngine::Models::CHANDLER("Chandler et al. (1958)", [](Vehicle *currentVehicle, double currentPosition, double speed, double acceleration, double deceleration, RoadObject *objectAhead, double objectAheadPosition, double speedLimit, double timeElapsed, double safetyDistance, double remainingDistance) {
		const double alpha = 0.37;
		return basicAccelerationModel(
				alpha * (objectAhead->getSpeed() - speed),
				currentVehicle, currentPosition, speed, acceleration, deceleration, objectAhead, objectAheadPosition, speedLimit, timeElapsed, safetyDistance, remainingDistance);
});
const TrafficEngine::Models TrafficEngine::Models::GAZIS("Gazis et al. (1959)", [](Vehicle *currentVehicle, double currentPosition, double speed, double acceleration, double deceleration, RoadObject *objectAhead, double objectAheadPosition, double speedLimit, double timeElapsed, double safetyDistance, double remainingDistance) {
		const double currentFrontPosition = currentPosition + currentVehicle->getVehicleLength() / 2.0;
		const double vehicleAheadBackPosition = objectAheadPosition - objectAhead->getVehicleLength() / 2.0;
		const double alpha = 12.2489;
		return basicAccelerationModel(
				alpha / (vehicleAheadBackPosition - currentFrontPosition) * (objectAhead->getSpeed() - speed),
				currentVehicle, currentPosition, speed, acceleration, deceleration, objectAhead, objectAheadPosition, speedLimit, timeElapsed, safetyDistance, remainingDistance);
});
const TrafficEngine::Models TrafficEngine::Models::EDIE("Edie (1961)", [](Vehicle *currentVehicle, double currentPosition, double speed, double acceleration, double deceleration, RoadObject *objectAhead, double objectAheadPosition, double speedLimit, double timeElapsed, double safetyDistance, double remainingDistance) {
		const double currentFrontPosition = currentPosition + currentVehicle->getVehicleLength() / 2.0;
		const double vehicleAheadBackPosition = objectAheadPosition - objectAhead->getVehicleLength() / 2.0;
		const double alpha = 12.2489;
		return basicAccelerationModel(
				alpha * speed / std::pow(vehicleAheadBackPosition - currentFrontPosition, 2.0) * (objectAhead->getSpeed() - speed),
				currentVehicle, currentPosition, speed, acceleration, deceleration, objectAhead, objectAheadPosition, speedLimit, timeElapsed, safetyDistance, remainingDistance);
});
const TrafficEngine::Models TrafficEngine::Models::GM_MODEL("General Motors Model", [](Vehicle *currentVehicle, double currentPosition, double speed, double acceleration, double deceleration, RoadObject *objectAhead, double objectAheadPosition, double speedLimit, double timeElapsed, double safetyDistance, double remainingDistance) {
		const double currentFrontPosition = currentPosition + currentVehicle->getVehicleLength() / 2.0;
		const double vehicleAheadBackPosition = objectAheadPosition - objectAhead->getVehicleLength() / 2.0;
		const double alpha = 0.000133;
		const double beta = 0.8;
		const double gamma = 2.8;
		return basicAccelerationModel(
				alpha * std::pow(speed, beta) / std::pow(vehicleAheadBackPosition - currentFrontPosition, gamma) * (objectAhead->getSpeed() - speed),
				currentVehicle, currentPosition, speed, acceleration, deceleration, objectAhead, objectAheadPosition, speedLimit, timeElapsed, safetyDistance, remainingDistance);
});
const TrafficEngine::Models TrafficEngine::Models::SIMPLE("Simple", [](Vehicle *currentVehicle, double currentPosition, double speed, double acceleration, double deceleration, RoadObject *objectAhead, double objectAheadPosition, double, double timeElapsed, double safetyDistance, double remainingDistance) {
		const double stopDistance = pow(speed + acceleration * timeElapsed, 2.0) / 2.0 / deceleration + safetyDistance + objectAhead->getVehicleLength() / 2.0;
		const double distanceTravelled = speed * timeElapsed + acceleration * timeElapsed * timeElapsed / 2.0;
		if (currentPosition + distanceTravelled + stopDistance + currentVehicle->getVehicleLength() / 2.0 >= objectAheadPosition) {
			return std::pair<double, double>(-deceleration * timeElapsed, // velocity changed
											 std::min(speed * timeElapsed - deceleration * timeElapsed * timeElapsed / 2.0, remainingDistance)); // distance travelled
		} else {
			return std::pair<double, double>(NAN, NAN);
		}
});
const std::array<TrafficEngine::Models, 6> TrafficEngine::Models::VALUES = std::array<TrafficEngine::Models, 6> {
	TrafficEngine::Models::INTELLIGENT_DRIVER_MODEL,
			TrafficEngine::Models::CHANDLER,
			TrafficEngine::Models::GAZIS,
			TrafficEngine::Models::EDIE,
			TrafficEngine::Models::GM_MODEL,
			TrafficEngine::Models::SIMPLE
};

unsigned int TrafficEngine::Models::idAssigner = 0;

TrafficEngine::Models::Models(const std::string &name, const std::function<std::pair<double, double>(Vehicle *, double, double, double, double, RoadObject *, double, double, double, double, double)> &modelFunction) :
	id(idAssigner++), name(name), modelFunction(modelFunction) {
}

double TrafficEngine::Models::intelligentDriverModelFunction(double x1, double v1, double x2, double v2, double acceleration, double deceleration, double preferredSpeed, double, double safetyDistance) {
	double safetyTimeHeadway = 2;
	double delta = 4.0;
	double sfunk = safetyDistance + v1 * safetyTimeHeadway + v1 * (v1 - v2) / (2.0 * sqrt(acceleration * deceleration));
	return acceleration * (1.0 - pow(v1 / preferredSpeed, delta) - pow(sfunk / (x2 - x1), 2.0));
}

std::pair<double, double> TrafficEngine::Models::basicAccelerationModel(double accelerationModelValue, Vehicle *currentVehicle, double currentPosition, double speed, double acceleration, double deceleration, RoadObject *objectAhead, double objectAheadPosition, double, double timeElapsed, double safetyDistance, double remainingDistance) {
	const double currentFrontPosition = currentPosition + currentVehicle->getVehicleLength() / 2.0;
	const double vehicleAheadBackPosition = objectAheadPosition - objectAhead->getVehicleLength() / 2.0;
	if (!objectAhead->isStationary() && currentFrontPosition < vehicleAheadBackPosition - safetyDistance) {
		if (accelerationModelValue > 0 && accelerationModelValue > acceleration) {
			accelerationModelValue = acceleration;
		} else if (accelerationModelValue < 0 && accelerationModelValue < -deceleration) {
			accelerationModelValue = -deceleration;
		}
		return std::pair<double, double>(accelerationModelValue * timeElapsed, // velocity changed
										 speed * timeElapsed + accelerationModelValue * timeElapsed * timeElapsed / 2.0); // distance travelled
	} else {
		return std::pair<double, double>(-deceleration * timeElapsed, // velocity changed
										 std::min(speed * timeElapsed - deceleration * timeElapsed * timeElapsed / 2.0, remainingDistance)); // distance travelled
	}
}

std::string TrafficEngine::Models::getName() const {
	return this->name;
}

std::pair<double, double> TrafficEngine::Models::apply(Vehicle *currentVehicle, double currentPosition, double speed, double acceleration, double deceleration, RoadObject *objectAhead, double objectAheadPosition, double speedLimit, double timeElapsed, double safetyDistance, double remainingDistance) const {
	return modelFunction(currentVehicle, currentPosition, speed, acceleration, deceleration, objectAhead, objectAheadPosition, speedLimit, timeElapsed, safetyDistance, remainingDistance);
}

bool TrafficEngine::Models::operator==(const TrafficEngine::Models &m) const {
	return this->id == m.id;
}
