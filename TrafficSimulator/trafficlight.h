#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

#include "direction.h"

#include <QComboBox>
#include <QElapsedTimer>
#include <QDoubleSpinBox>
#include <QVector>

#include <memory>

class Intersection;
class IntersectionLane;
enum class TrafficLightColor;
class VehiclePath;

template<typename TrafficLightColor>
class QVector;
template<typename IntersectionLane>
class QSharedPointer;

enum class TrafficLightColor {
    GREEN = 0,
    YELLOW,
    RED,
    ILLEGAL
};

QDebug operator<<(QDebug debug, const TrafficLightColor &trafficLightColor);

class TrafficLight {
public:
	class LightControlComboBox;
	class SpeedLimitTextBox;
	class TrafficLightState;

private:
	class TrafficLightStateOptions;

	static const std::array<TrafficLightStateOptions, 8> VALUES_RHD;
	static const std::array<TrafficLightStateOptions, 8> VALUES_LHD;

	Intersection* intersection;
	QElapsedTimer timer;
	double timeOffset; // [0, 1) where 0 is no offset and 1 is offset by 1 total duration cycle
	unsigned int previousStateOptionID;
	int forceTimeState;
	std::unique_ptr<LightControlComboBox> lightControlComboBox;
	std::unique_ptr<SpeedLimitTextBox> speedLimitTextBox;

public:
    TrafficLight(Intersection* intersection);

	TrafficLight::TrafficLightState getCurrentState() const;
	TrafficLight::TrafficLightState getPriorState(double timePriorDifference) const;

	double getTimerSecElapsed(bool includePause) const;
    Intersection* getIntersection() const;

	LightControlComboBox *getLightControlComboBox() const;
	SpeedLimitTextBox *getSpeedLimitTextBox() const;

	bool isPathLegal(Direction::Cardinal sourceDirection, Direction::Cardinal destinationDirection) const;
	bool isPathLegal(Direction::Cardinal sourceDirection, int sourceLane, Direction::Cardinal destinationDirection) const;
	bool isPathLegal(Direction::Cardinal sourceDirection, int sourceLane, Direction::Cardinal destinationDirection, int destinationLane) const;
    int getLegalPathsCount(VehiclePath *currentPath, int sourceLane) const;
	int getLegalPathsCount(Direction::Cardinal sourceDirection, int sourceLane) const;
	int getDestinationPathsCount(const IntersectionLane &sourceLane) const;
	bool hasPathInCurrentStateOptions(const Direction::Cardinal &sourceDirection, const Direction::Cardinal &destinationDirection) const;
	bool hasPath(const Direction::Cardinal &sourceDirection, const Direction::Cardinal &destinationDirection) const;
	bool hasPath(const Direction::Cardinal &sourceDirection, int sourceLane, const Direction::Cardinal &destinationDirection) const;
	bool hasPath(const Direction::Cardinal &sourceDirection, int sourceLane, const Direction::Cardinal &destinationDirection, int destinationLane) const;
	std::vector<Direction::Cardinal> getPossibleDirections(const IntersectionLane &sourceLane) const;
	std::vector<IntersectionLane> getPossiblePaths(const IntersectionLane &sourceLane) const;

	void update();

	static bool mustStop(const TrafficLightColor &lightColor);

private:
	double getCurrentTimePosition() const;
	double getPriorTimePosition(double timePriorDifferenceInSeconds) const;
	TrafficLight::TrafficLightStateOptions getCurrentStateOption(double timeElasped) const;
	std::vector<TrafficLightStateOptions> getCurrentStateOptions() const;
};

class TrafficLight::TrafficLightStateOptions {
public:
	static const TrafficLightStateOptions ALL_RED;
	static const TrafficLightStateOptions LEFT_GREEN_RHD_1;
	static const TrafficLightStateOptions LEFT_YELLOW_RHD_1;
	static const TrafficLightStateOptions STRAIGHT_GREEN_RHD_1;
	static const TrafficLightStateOptions STRAIGHT_YELLOW_RHD_1;
	static const TrafficLightStateOptions LEFT_GREEN_RHD_2;
	static const TrafficLightStateOptions LEFT_YELLOW_RHD_2;
	static const TrafficLightStateOptions STRAIGHT_GREEN_RHD_2;
	static const TrafficLightStateOptions STRAIGHT_YELLOW_RHD_2;

	static const TrafficLightStateOptions RIGHT_GREEN_LHD_1;
	static const TrafficLightStateOptions RIGHT_YELLOW_LHD_1;
	static const TrafficLightStateOptions STRAIGHT_GREEN_LHD_1;
	static const TrafficLightStateOptions STRAIGHT_YELLOW_LHD_1;
	static const TrafficLightStateOptions RIGHT_GREEN_LHD_2;
	static const TrafficLightStateOptions RIGHT_YELLOW_LHD_2;
	static const TrafficLightStateOptions STRAIGHT_GREEN_LHD_2;
	static const TrafficLightStateOptions STRAIGHT_YELLOW_LHD_2;

private:
	static unsigned int idAssigner;

private:
	unsigned int id;
	double durationSeconds;
	TrafficLightColor lightColor;
	QVector<std::array<Direction::Cardinal, 2>> directions;

	TrafficLightStateOptions();
	TrafficLightStateOptions(double durationSeconds, const TrafficLightColor &lightColor, const QVector<std::array<Direction::Cardinal, 2>> &directions);

public:
	unsigned int getID() const;
	double getDuration() const;
	TrafficLightColor getLightColor() const;
	const QVector<std::array<Direction::Cardinal, 2>> getDirections() const;

	bool operator==(const TrafficLightStateOptions &option) const;
};

class TrafficLight::TrafficLightState {
	const TrafficLight *trafficLight;
	double timePosition;
	TrafficLight::TrafficLightStateOptions state;

public:
	TrafficLightState(const TrafficLight *trafficLight, double timePosition, const TrafficLight::TrafficLightStateOptions &state);

	TrafficLightColor getLightColor(Direction::Cardinal sourceDirection, int sourceLane) const;

    /**
     * @brief getLightColor Gets the traffic light color of a path in the intersection.
     * @param sourceDirection the direction relative to the intersection of where the vehicle begins the path across the intersection
     * @param sourceLane the lane in which the vehicle begins the path across the intersection. The lane index starts at 0 and increases clockwise
     * @param destinationDirection the direction relative to the intersection of where the vehicle ends the path across the intersection
     * @param destinationLane the lane in which the vehicle ends the path across the intersection. The lane index starts at 0 and increases clockwise
     * @return
     */
	TrafficLightColor getLightColor(Direction::Cardinal sourceDirection, int sourceLane, Direction::Cardinal destinationDirection, int destinationLane) const;

	double getTimeToNextLight(const TrafficLightColor &lightColor, const IntersectionLane &sourceLane) const;
	double getTimeToNextLight(const TrafficLightColor &lightColor, const IntersectionLane &sourceLane, const IntersectionLane &destinationLane) const;

	bool hasPath(const Direction::Cardinal &sourceDirection, const Direction::Cardinal &destinationDirection) const;
};

class TrafficLight::LightControlComboBox : public QComboBox {
	Q_OBJECT

	TrafficLight *trafficLight;

public:
	LightControlComboBox(TrafficLight *trafficLight, QWidget *parent = nullptr);

	void updateOptions();

private slots:
	void indexChanged(int index);

};

class TrafficLight::SpeedLimitTextBox : public QDoubleSpinBox {
	Q_OBJECT

	TrafficLight *trafficLight;

public:
	SpeedLimitTextBox(TrafficLight *trafficLight, QWidget *parent = nullptr);

private slots:
	void valueChangedSlot(double newValue);

};

#endif // TRAFFICLIGHT_H
