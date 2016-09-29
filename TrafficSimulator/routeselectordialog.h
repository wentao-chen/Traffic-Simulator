#ifndef ROUTESELECTORDIALOG_H
#define ROUTESELECTORDIALOG_H

#include <QDialog>

class TrafficEngine;
class Vehicle;

namespace Ui {
	class RouteSelectorDialog;
}

class RouteSelectorDialog : public QDialog {
	Q_OBJECT

public:
	explicit RouteSelectorDialog(TrafficEngine *trafficEngine, QWidget *parent = 0);
	~RouteSelectorDialog();

	void resetNewDestinationButton();
	void setDestinationText(const std::string &text);

protected:
	virtual void hideEvent(QHideEvent *event);

private slots:
	void on_newDestinationButton_clicked();

	void on_showDestinationButton_clicked();

private:
	Ui::RouteSelectorDialog *ui;
	QString newDestinationButtonText;
	bool newDestinationButtonClicked;
	TrafficEngine *trafficEngine;
};

#endif // ROUTESELECTORDIALOG_H
