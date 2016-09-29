#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QObject>

#include <memory>

class HelpDialog;
class Intersection;
class SettingsDialog;
class TrafficEngine;

class QDialog;
class QGraphicsScene;
class QTimer;
class QWidget;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

private:
	Ui::MainWindow *ui;
	QGraphicsScene *scene;
	std::unique_ptr<SettingsDialog> settingsDialog;
	std::unique_ptr<HelpDialog> helpDialog;
	std::unique_ptr<TrafficEngine> trafficEngine;
	std::unique_ptr<QTimer> timer;

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

	TrafficEngine* getTrafficEngine();

	int getTimerInterval() const;
	void setTimerInterval(int msDelay);

private slots:
    /**
     * @brief addIntersection Creates an intersection and places in randomly in the scene rect
     */
    void addIntersection();
	void addIntersection(qreal x, qreal y, Intersection *intersection = nullptr);
    void addVehicle();
	void addIntersectionGrid();
	void on_pushButton_2_clicked();
	void on_pushButton_clicked();
	void on_pushButton_3_clicked();
	void on_settingsButton_clicked();
	void on_helpPushButton_clicked();
};

#endif // MAINWINDOW_H
