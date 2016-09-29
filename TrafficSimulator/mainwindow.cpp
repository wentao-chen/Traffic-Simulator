#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "constants.h"
#include "helpdialog.h"
#include "intersection.h"
#include "linearroad.h"
#include "settingsdialog.h"
#include "trafficengine.h"
#include "vehicle.h"

#include <QDialog>
#include <QTimer>
#include <QTime>

#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow()),
		scene(new QGraphicsScene(this)),
		settingsDialog(std::unique_ptr<SettingsDialog>(new SettingsDialog(this))),
		helpDialog(std::unique_ptr<HelpDialog>(new HelpDialog(this))),
		trafficEngine(std::unique_ptr<TrafficEngine>(new TrafficEngine(this))),
        timer(std::unique_ptr<QTimer>(new QTimer(this))) {
    qsrand(QTime::currentTime().msec());

	ui->setupUi(this);
    ui->graphicsView->setScene(this->scene);
    ui->graphicsView->setRenderHint(QPainter::Antialiasing);

	this->scene->setSceneRect(-200, -200, 400, 400);
	setMinimumSize(this->scene->width() + 200, this->scene->height() + 120);

	connect(ui->actionAddIntersection, SIGNAL(triggered()), this, SLOT(addIntersection()));
	connect(ui->actionAddVehicle, SIGNAL(triggered()), this, SLOT(addVehicle()));
	connect(ui->actionActionAddIntersectionGrid, SIGNAL(triggered()), this, SLOT(addIntersectionGrid()));
	ui->statusBar->showMessage("Running");

    connect(this->timer.get(), SIGNAL(timeout()), this->scene, SLOT(advance()));
	this->timer->start(FrameTime::FRAME_DELAY * 1000);
}

MainWindow::~MainWindow() {
	for (auto &intersection : getTrafficEngine()->getIntersections()) {
		for (auto &road : intersection->getConnections()) {
			if (road->scene() == this->scene) {
				this->scene->removeItem(road);
				delete road;
			}
		}
		this->scene->removeItem(intersection);
		delete intersection;
	}
	delete ui;
}

TrafficEngine *MainWindow::getTrafficEngine() {
	return this->trafficEngine.get();
}

int MainWindow::getTimerInterval() const {
	return this->timer->interval();
}

void MainWindow::setTimerInterval(int msDelay) {
	this->timer->setInterval(msDelay);
}

void MainWindow::addIntersection() {
	Intersection *intersection = new Intersection(getTrafficEngine());
	addIntersection(fmod(qrand(), this->scene->width() - intersection->getRectWidth()) + this->scene->sceneRect().left(), fmod(qrand(), this->scene->height() - intersection->getRectHeight()) + this->scene->sceneRect().top(), intersection);
}

void MainWindow::addIntersection(qreal x, qreal y, Intersection *intersection) {
	TrafficEngine *trafficEngine = getTrafficEngine();
	if (intersection == nullptr) {
		intersection = new Intersection(trafficEngine);
	}
	trafficEngine->addIntersection(intersection);
	intersection->setZValue(1);
	this->scene->addItem(intersection);
	intersection->setPos(intersection->mapToParent(x, y));
}

void MainWindow::addVehicle() {
    TrafficEngine *trafficEngine = getTrafficEngine();
	if (trafficEngine->getRoadCount() > 0) {
		const TrafficEngine::Models &trafficModel = this->settingsDialog->getCurrentTrafficModel();
		Vehicle *vehicle = new Vehicle(trafficEngine, "Car", trafficModel, Sizes::LANE_WIDTH / 2.0, Sizes::LANE_WIDTH, Speeds::AVERAGE_ACCELERATION, Speeds::COMFORTABLE_DECELERATION, Speeds::AVERAGE_SAFETY_DISTANCE);
		if (qrand() % 2 == 0) {
			delete vehicle;
			vehicle = new Vehicle(trafficEngine, "Truck", trafficModel, Sizes::LANE_WIDTH / 2.0, Sizes::LANE_WIDTH * 1.5, Speeds::AVERAGE_ACCELERATION / 2.0, Speeds::COMFORTABLE_DECELERATION / 2.0, Speeds::AVERAGE_SAFETY_DISTANCE * 2.0);
		}
		trafficEngine->addVehicle(vehicle);
		vehicle->setZValue(2);
		this->scene->addItem(vehicle);
	}
}

void MainWindow::addIntersectionGrid() {
	ui->actionActionAddIntersectionGrid->setEnabled(false);
	QVector<QVector<Intersection*>> newIntersections;
	QVector<QVector<bool>> hasIntersection { {true, true, true},
											 {true, true, true},
											 {true, true, true}
										   };
	QVector<double> intersectionVerticalLanes {4, 5, 4};
	QVector<double> intersectionVerticalLanesGaps {Sizes::LANE_WIDTH * 3, Sizes::LANE_WIDTH * 6};
	QVector<double> intersectionHorizontalLanes {4, 5, 4};
	QVector<double> intersectionHorizontalLanesGaps {Sizes::LANE_WIDTH * 6, Sizes::LANE_WIDTH * 2};
	double totalWidth = intersectionVerticalLanes.at(0) * Sizes::LANE_WIDTH;
	double totalHeight = intersectionHorizontalLanes.at(0) * Sizes::LANE_WIDTH;
	for (int x = 1; x < intersectionVerticalLanes.size(); x++) {
		totalWidth += intersectionVerticalLanesGaps.at(x - 1) + intersectionVerticalLanes.at(x) * Sizes::LANE_WIDTH;
	}
	for (int y = 1; y < intersectionHorizontalLanes.size(); y++) {
		totalHeight += intersectionHorizontalLanesGaps.at(y - 1) + intersectionHorizontalLanes.at(y) * Sizes::LANE_WIDTH;
	}
	double leftX = -totalWidth / 2.0;
	double topY = -totalHeight / 2.0;
	for (int y = 0; y < hasIntersection.size(); y++) {
		QVector<Intersection*> newIntersectionsRow;
		double beforeY = topY;
		for (int i = 0; i < y; i++) {
			beforeY += intersectionHorizontalLanes.at(i) * Sizes::LANE_WIDTH + intersectionHorizontalLanesGaps.at(i);
		}
		const QVector<bool> &hasIntersectionRow = hasIntersection.at(y);
		for (int x = 0; x < hasIntersectionRow.size(); x++) {
			if (hasIntersectionRow.at(x)) {
				Intersection *intersection = new Intersection(getTrafficEngine(), intersectionVerticalLanes.at(x), intersectionHorizontalLanes.at(y));
				double beforeX = leftX;
				for (int i = 0; i < x; i++) {
					beforeX += intersectionVerticalLanes.at(i) * Sizes::LANE_WIDTH + intersectionVerticalLanesGaps.at(i);
				}
				addIntersection(beforeX, beforeY, intersection);
				int previousIntersectionY = -1;
				for (int i = y - 1; i >= 0; i--) {
					if (hasIntersection.at(i).at(x)) {
						previousIntersectionY = i;
						break;
					}
				}
				if (previousIntersectionY >= 0) {
					Road *road = new LinearRoad(intersection, Direction::Cardinal::NORTH, Speeds::AVERAGE_SPEED, intersection->parentItem());
					intersection->setConnection(Direction::Cardinal::NORTH, road);
					intersection->scene()->addItem(road);
					road->setFreeEnd(newIntersections.at(previousIntersectionY).at(x), Direction::Cardinal::SOUTH);
					road->setFreeEndAtEnd();
					road->update();
				}
				int previousIntersectionX = -1;
				for (int i = x - 1; i >= 0; i--) {
					if (hasIntersectionRow.at(i)) {
						previousIntersectionX = i;
						break;
					}
				}
				if (previousIntersectionX >= 0) {
					Road *road = new LinearRoad(intersection, Direction::Cardinal::WEST, Speeds::AVERAGE_SPEED, intersection->parentItem());
					intersection->setConnection(Direction::Cardinal::WEST, road);
					intersection->scene()->addItem(road);
					road->setFreeEnd(newIntersectionsRow.at(previousIntersectionX), Direction::Cardinal::EAST);
					road->setFreeEndAtEnd();
					road->update();
				}
				newIntersectionsRow.push_back(intersection);
			} else {
				newIntersectionsRow.push_back(nullptr);
			}
		}
		newIntersections.push_back(newIntersectionsRow);
	}
}

void MainWindow::on_pushButton_2_clicked() {
	this->trafficEngine->toggleVehiclesInformationTableVisible();
}

void MainWindow::on_pushButton_clicked() {
	this->trafficEngine->toggleIntersectionsInformationTableVisible();
}

void MainWindow::on_pushButton_3_clicked() {
	if (this->trafficEngine->isPaused()) {
		this->trafficEngine->setPauseState(false);
		ui->statusBar->showMessage("Running");
		ui->pushButton_3->setText(QString::fromStdString("Pause"));
	} else {
		this->trafficEngine->setPauseState(true);
		ui->statusBar->showMessage("Paused");
		ui->pushButton_3->setText(QString::fromStdString("Resume"));
	}
}

void MainWindow::on_settingsButton_clicked() {
	this->settingsDialog->setVisible(!this->settingsDialog->isVisible());
}

void MainWindow::on_helpPushButton_clicked() {
	this->helpDialog->setVisible(!this->helpDialog->isVisible());
}
