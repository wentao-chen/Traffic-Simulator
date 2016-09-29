#include "settingsdialog.h"
#include "ui_settingsdialog.h"

#include "constants.h"
#include "mainwindow.h"
#include "trafficengine.h"

#include <QDebug>

SettingsDialog::SettingsDialog(MainWindow *mainWindow) :
		QDialog(mainWindow),
		ui(new Ui::SettingsDialog),
		mainWindow(mainWindow),
		temporaryTrafficDirection(mainWindow->getTrafficEngine()->isRightHandDrive()),
		temporaryTrafficModel(TrafficEngine::Models::VALUES.at(0)) {
	ui->setupUi(this);
	for (auto &model : TrafficEngine::Models::VALUES) {
		ui->trafficModelComboBox->addItem(QString::fromStdString(model.getName()));
	}
	ui->fpsSpinBox->setValue(1.0 / FrameTime::FRAME_DELAY);
	updateTrafficDirectionLabel();
}

SettingsDialog::~SettingsDialog() {
	delete ui;
}

void SettingsDialog::showEvent(QShowEvent *) {
	this->temporaryTrafficDirection = this->mainWindow->getTrafficEngine()->isRightHandDrive();
	updateTrafficDirectionLabel();
	ui->fpsSpinBox->setValue(1000.0 / this->mainWindow->getTimerInterval());
}

void SettingsDialog::on_trafficDirectionButton_clicked() {
	this->temporaryTrafficDirection = !this->temporaryTrafficDirection;
	updateTrafficDirectionLabel();
}

void SettingsDialog::updateTrafficDirectionLabel() {
	if (this->temporaryTrafficDirection) {
		ui->trafficDirectionLabel->setText(QString::fromStdString(Description::RIGHT_HAND_TRAFFIC));
	} else {
		ui->trafficDirectionLabel->setText(QString::fromStdString(Description::LEFT_HAND_TRAFFIC));
	}
}

void SettingsDialog::on_buttonBox_accepted() {
	this->mainWindow->getTrafficEngine()->setRightHandDrive(this->temporaryTrafficDirection);
	this->mainWindow->getTrafficEngine()->setVehicleModel(this->temporaryTrafficModel);
	this->mainWindow->setTimerInterval((int) std::ceil(1000.0 / ui->fpsSpinBox->value()));
}

TrafficEngine::Models SettingsDialog::getCurrentTrafficModel() const {
	int index = ui->trafficModelComboBox->currentIndex();
	if (index < 0 && (unsigned int) index >= TrafficEngine::Models::VALUES.size()) {
		index = 0;
	}
	return TrafficEngine::Models::VALUES.at(index);
}

void SettingsDialog::on_trafficModelComboBox_currentIndexChanged(int index) {
	if (index >= 0 && (unsigned int) index < TrafficEngine::Models::VALUES.size()) {
		this->temporaryTrafficModel = TrafficEngine::Models::VALUES.at(index);
	}
}
