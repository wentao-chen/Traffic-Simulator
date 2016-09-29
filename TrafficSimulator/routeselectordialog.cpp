#include "routeselectordialog.h"
#include "ui_routeselectordialog.h"

#include "trafficengine.h"

#include <QDebug>

RouteSelectorDialog::RouteSelectorDialog(TrafficEngine *trafficEngine, QWidget *parent) :
		QDialog(parent),
		ui(new Ui::RouteSelectorDialog),
		newDestinationButtonText(QString()),
		newDestinationButtonClicked(false),
		trafficEngine(trafficEngine) {
	ui->setupUi(this);
	newDestinationButtonText = ui->newDestinationButton->text();
}

RouteSelectorDialog::~RouteSelectorDialog() {
	delete ui;
}

void RouteSelectorDialog::resetNewDestinationButton() {
	this->newDestinationButtonClicked = false;
	ui->label->setText(QString());
	ui->newDestinationButton->setText(this->newDestinationButtonText);
	this->trafficEngine->setEnableRouteVehicle(false);
	this->trafficEngine->setFlashingIntersections(false);
}

void RouteSelectorDialog::setDestinationText(const std::string &text) {
	ui->showDestinationButton->setText(QString::fromStdString(text));
}

void RouteSelectorDialog::hideEvent(QHideEvent*) {
	resetNewDestinationButton();
	this->trafficEngine->clearRouteVehicle();
}

void RouteSelectorDialog::on_newDestinationButton_clicked() {
	if (!this->newDestinationButtonClicked) {
		ui->label->setText(QString::fromStdString("Select a destination on the map."));
		ui->newDestinationButton->setText(QString::fromStdString("Cancel"));
		this->trafficEngine->setEnableRouteVehicle(true);
		this->trafficEngine->setFlashingIntersections(true);
	} else {
		ui->label->setText(QString());
		ui->newDestinationButton->setText(this->newDestinationButtonText);
		this->trafficEngine->setEnableRouteVehicle(false);
		this->trafficEngine->setFlashingIntersections(false);
	}
	this->newDestinationButtonClicked = !this->newDestinationButtonClicked;
}

void RouteSelectorDialog::on_showDestinationButton_clicked() {
	this->trafficEngine->toggleHighlightRouteSelectorVehicleRoute();
}
