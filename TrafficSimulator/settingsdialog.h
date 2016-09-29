#ifndef SETTINGSDIALOG_H
#define SETTINGSDIALOG_H

#include "trafficengine.h"

#include <QDialog>

class MainWindow;

namespace Ui {
	class SettingsDialog;
}

class SettingsDialog : public QDialog {
	Q_OBJECT

public:
	explicit SettingsDialog(MainWindow *parent = 0);
	~SettingsDialog();

	TrafficEngine::Models getCurrentTrafficModel() const;
protected:
	virtual void showEvent(QShowEvent *showEvent) override;

private slots:
	void on_trafficDirectionButton_clicked();

	void on_buttonBox_accepted();

	void on_trafficModelComboBox_currentIndexChanged(int index);

private:
	Ui::SettingsDialog *ui;
	MainWindow *mainWindow;

	bool temporaryTrafficDirection;
	TrafficEngine::Models temporaryTrafficModel;

	void updateTrafficDirectionLabel();
};

#endif // SETTINGSDIALOG_H
