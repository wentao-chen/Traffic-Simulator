#include "trafficlight.h"

#include "constants.h"
#include "general.h"
#include "intersection.h"
#include "trafficengine.h"
#include "road.h"

#include <QVector>
#include <QDebug>

#include <assert.h>
#include <limits>
#include <vector>

const TrafficLight::TrafficLightStateOptions TrafficLight::TrafficLightStateOptions::ALL_RED = TrafficLight::TrafficLightStateOptions();

const TrafficLight::TrafficLightStateOptions TrafficLight::TrafficLightStateOptions::LEFT_GREEN_RHD_1 = TrafficLight::TrafficLightStateOptions(3, TrafficLightColor::GREEN, QVector<std::array<Direction::Cardinal, 2>> {
																									std::array<Direction::Cardinal, 2>{Direction::Cardinal::NORTH, Direction::Cardinal::EAST},
																									std::array<Direction::Cardinal, 2>{Direction::Cardinal::SOUTH, Direction::Cardinal::WEST},
																									std::array<Direction::Cardinal, 2>{Direction::Cardinal::EAST, Direction::Cardinal::NORTH},
																									std::array<Direction::Cardinal, 2>{Direction::Cardinal::WEST, Direction::Cardinal::SOUTH}
																								});
const TrafficLight::TrafficLightStateOptions TrafficLight::TrafficLightStateOptions::LEFT_YELLOW_RHD_1 = TrafficLight::TrafficLightStateOptions(1, TrafficLightColor::YELLOW, QVector<std::array<Direction::Cardinal, 2>> {
																									 std::array<Direction::Cardinal, 2>{Direction::Cardinal::NORTH, Direction::Cardinal::EAST},
																									 std::array<Direction::Cardinal, 2>{Direction::Cardinal::SOUTH, Direction::Cardinal::WEST},
																									 std::array<Direction::Cardinal, 2>{Direction::Cardinal::EAST, Direction::Cardinal::NORTH},
																									 std::array<Direction::Cardinal, 2>{Direction::Cardinal::WEST, Direction::Cardinal::SOUTH}
																								 });
const TrafficLight::TrafficLightStateOptions TrafficLight::TrafficLightStateOptions::STRAIGHT_GREEN_RHD_1 = TrafficLight::TrafficLightStateOptions(6, TrafficLightColor::GREEN, QVector<std::array<Direction::Cardinal, 2>> {
																										std::array<Direction::Cardinal, 2>{Direction::Cardinal::NORTH, Direction::Cardinal::SOUTH},
																										std::array<Direction::Cardinal, 2>{Direction::Cardinal::SOUTH, Direction::Cardinal::NORTH},
																										std::array<Direction::Cardinal, 2>{Direction::Cardinal::NORTH, Direction::Cardinal::WEST},
																										std::array<Direction::Cardinal, 2>{Direction::Cardinal::SOUTH, Direction::Cardinal::EAST}
																									});
const TrafficLight::TrafficLightStateOptions TrafficLight::TrafficLightStateOptions::STRAIGHT_YELLOW_RHD_1 = TrafficLight::TrafficLightStateOptions(2, TrafficLightColor::YELLOW, QVector<std::array<Direction::Cardinal, 2>> {
																										 std::array<Direction::Cardinal, 2>{Direction::Cardinal::NORTH, Direction::Cardinal::SOUTH},
																										 std::array<Direction::Cardinal, 2>{Direction::Cardinal::SOUTH, Direction::Cardinal::NORTH},
																										 std::array<Direction::Cardinal, 2>{Direction::Cardinal::NORTH, Direction::Cardinal::WEST},
																										 std::array<Direction::Cardinal, 2>{Direction::Cardinal::SOUTH, Direction::Cardinal::EAST}
																									 });
const TrafficLight::TrafficLightStateOptions TrafficLight::TrafficLightStateOptions::LEFT_GREEN_RHD_2 = TrafficLight::TrafficLightStateOptions(3, TrafficLightColor::GREEN, QVector<std::array<Direction::Cardinal, 2>> {
																									std::array<Direction::Cardinal, 2>{Direction::Cardinal::WEST, Direction::Cardinal::NORTH},
																									std::array<Direction::Cardinal, 2>{Direction::Cardinal::EAST, Direction::Cardinal::SOUTH},
																									std::array<Direction::Cardinal, 2>{Direction::Cardinal::NORTH, Direction::Cardinal::WEST},
																									std::array<Direction::Cardinal, 2>{Direction::Cardinal::SOUTH, Direction::Cardinal::EAST}
																								});
const TrafficLight::TrafficLightStateOptions TrafficLight::TrafficLightStateOptions::LEFT_YELLOW_RHD_2 = TrafficLight::TrafficLightStateOptions(1, TrafficLightColor::YELLOW, QVector<std::array<Direction::Cardinal, 2>> {
																									 std::array<Direction::Cardinal, 2>{Direction::Cardinal::WEST, Direction::Cardinal::NORTH},
																									 std::array<Direction::Cardinal, 2>{Direction::Cardinal::EAST, Direction::Cardinal::SOUTH},
																									 std::array<Direction::Cardinal, 2>{Direction::Cardinal::NORTH, Direction::Cardinal::WEST},
																									 std::array<Direction::Cardinal, 2>{Direction::Cardinal::SOUTH, Direction::Cardinal::EAST}
																								 });
const TrafficLight::TrafficLightStateOptions TrafficLight::TrafficLightStateOptions::STRAIGHT_GREEN_RHD_2 = TrafficLight::TrafficLightStateOptions(6, TrafficLightColor::GREEN, QVector<std::array<Direction::Cardinal, 2>> {
																										std::array<Direction::Cardinal, 2>{Direction::Cardinal::WEST, Direction::Cardinal::EAST},
																										std::array<Direction::Cardinal, 2>{Direction::Cardinal::EAST, Direction::Cardinal::WEST},
																										std::array<Direction::Cardinal, 2>{Direction::Cardinal::WEST, Direction::Cardinal::SOUTH},
																										std::array<Direction::Cardinal, 2>{Direction::Cardinal::EAST, Direction::Cardinal::NORTH}
																									});
const TrafficLight::TrafficLightStateOptions TrafficLight::TrafficLightStateOptions::STRAIGHT_YELLOW_RHD_2 = TrafficLight::TrafficLightStateOptions(2, TrafficLightColor::YELLOW, QVector<std::array<Direction::Cardinal, 2>> {
																										 std::array<Direction::Cardinal, 2>{Direction::Cardinal::WEST, Direction::Cardinal::EAST},
																										 std::array<Direction::Cardinal, 2>{Direction::Cardinal::EAST, Direction::Cardinal::WEST},
																										 std::array<Direction::Cardinal, 2>{Direction::Cardinal::WEST, Direction::Cardinal::SOUTH},
																										 std::array<Direction::Cardinal, 2>{Direction::Cardinal::EAST, Direction::Cardinal::NORTH}
																									 });

const std::array<TrafficLight::TrafficLightStateOptions, 8> TrafficLight::VALUES_RHD = std::array<TrafficLight::TrafficLightStateOptions, 8> {TrafficLight::TrafficLightStateOptions::LEFT_GREEN_RHD_1,
																																									TrafficLight::TrafficLightStateOptions::LEFT_YELLOW_RHD_1,
																																									TrafficLight::TrafficLightStateOptions::STRAIGHT_GREEN_RHD_1,
																																									TrafficLight::TrafficLightStateOptions::STRAIGHT_YELLOW_RHD_1,
																																									TrafficLight::TrafficLightStateOptions::LEFT_GREEN_RHD_2,
																																									TrafficLight::TrafficLightStateOptions::LEFT_YELLOW_RHD_2,
																																									TrafficLight::TrafficLightStateOptions::STRAIGHT_GREEN_RHD_2,
																																									TrafficLight::TrafficLightStateOptions::STRAIGHT_YELLOW_RHD_2
																																									};

const TrafficLight::TrafficLightStateOptions TrafficLight::TrafficLightStateOptions::RIGHT_GREEN_LHD_1 = TrafficLight::TrafficLightStateOptions(3, TrafficLightColor::GREEN, QVector<std::array<Direction::Cardinal, 2>> {
																									std::array<Direction::Cardinal, 2>{Direction::Cardinal::NORTH, Direction::Cardinal::WEST},
																									std::array<Direction::Cardinal, 2>{Direction::Cardinal::SOUTH, Direction::Cardinal::EAST},
																									std::array<Direction::Cardinal, 2>{Direction::Cardinal::EAST, Direction::Cardinal::SOUTH},
																									std::array<Direction::Cardinal, 2>{Direction::Cardinal::WEST, Direction::Cardinal::NORTH}
																								});
const TrafficLight::TrafficLightStateOptions TrafficLight::TrafficLightStateOptions::RIGHT_YELLOW_LHD_1 = TrafficLight::TrafficLightStateOptions(1, TrafficLightColor::YELLOW, QVector<std::array<Direction::Cardinal, 2>> {
																									 std::array<Direction::Cardinal, 2>{Direction::Cardinal::NORTH, Direction::Cardinal::WEST},
																									 std::array<Direction::Cardinal, 2>{Direction::Cardinal::SOUTH, Direction::Cardinal::EAST},
																									 std::array<Direction::Cardinal, 2>{Direction::Cardinal::EAST, Direction::Cardinal::SOUTH},
																									 std::array<Direction::Cardinal, 2>{Direction::Cardinal::WEST, Direction::Cardinal::NORTH}
																								 });
const TrafficLight::TrafficLightStateOptions TrafficLight::TrafficLightStateOptions::STRAIGHT_GREEN_LHD_1 = TrafficLight::TrafficLightStateOptions(6, TrafficLightColor::GREEN, QVector<std::array<Direction::Cardinal, 2>> {
																										std::array<Direction::Cardinal, 2>{Direction::Cardinal::NORTH, Direction::Cardinal::SOUTH},
																										std::array<Direction::Cardinal, 2>{Direction::Cardinal::SOUTH, Direction::Cardinal::NORTH},
																										std::array<Direction::Cardinal, 2>{Direction::Cardinal::NORTH, Direction::Cardinal::EAST},
																										std::array<Direction::Cardinal, 2>{Direction::Cardinal::SOUTH, Direction::Cardinal::WEST}
																									});
const TrafficLight::TrafficLightStateOptions TrafficLight::TrafficLightStateOptions::STRAIGHT_YELLOW_LHD_1 = TrafficLight::TrafficLightStateOptions(2, TrafficLightColor::YELLOW, QVector<std::array<Direction::Cardinal, 2>> {
																										 std::array<Direction::Cardinal, 2>{Direction::Cardinal::NORTH, Direction::Cardinal::SOUTH},
																										 std::array<Direction::Cardinal, 2>{Direction::Cardinal::SOUTH, Direction::Cardinal::NORTH},
																										 std::array<Direction::Cardinal, 2>{Direction::Cardinal::NORTH, Direction::Cardinal::EAST},
																										 std::array<Direction::Cardinal, 2>{Direction::Cardinal::SOUTH, Direction::Cardinal::WEST}
																									 });
const TrafficLight::TrafficLightStateOptions TrafficLight::TrafficLightStateOptions::RIGHT_GREEN_LHD_2 = TrafficLight::TrafficLightStateOptions(3, TrafficLightColor::GREEN, QVector<std::array<Direction::Cardinal, 2>> {
																									std::array<Direction::Cardinal, 2>{Direction::Cardinal::WEST, Direction::Cardinal::SOUTH},
																									std::array<Direction::Cardinal, 2>{Direction::Cardinal::EAST, Direction::Cardinal::NORTH},
																									std::array<Direction::Cardinal, 2>{Direction::Cardinal::NORTH, Direction::Cardinal::EAST},
																									std::array<Direction::Cardinal, 2>{Direction::Cardinal::SOUTH, Direction::Cardinal::WEST}
																								});
const TrafficLight::TrafficLightStateOptions TrafficLight::TrafficLightStateOptions::RIGHT_YELLOW_LHD_2 = TrafficLight::TrafficLightStateOptions(1, TrafficLightColor::YELLOW, QVector<std::array<Direction::Cardinal, 2>> {
																									 std::array<Direction::Cardinal, 2>{Direction::Cardinal::WEST, Direction::Cardinal::SOUTH},
																									 std::array<Direction::Cardinal, 2>{Direction::Cardinal::EAST, Direction::Cardinal::NORTH},
																									 std::array<Direction::Cardinal, 2>{Direction::Cardinal::NORTH, Direction::Cardinal::EAST},
																									 std::array<Direction::Cardinal, 2>{Direction::Cardinal::SOUTH, Direction::Cardinal::WEST}
																								 });
const TrafficLight::TrafficLightStateOptions TrafficLight::TrafficLightStateOptions::STRAIGHT_GREEN_LHD_2 = TrafficLight::TrafficLightStateOptions(6, TrafficLightColor::GREEN, QVector<std::array<Direction::Cardinal, 2>> {
																										std::array<Direction::Cardinal, 2>{Direction::Cardinal::WEST, Direction::Cardinal::EAST},
																										std::array<Direction::Cardinal, 2>{Direction::Cardinal::EAST, Direction::Cardinal::WEST},
																										std::array<Direction::Cardinal, 2>{Direction::Cardinal::WEST, Direction::Cardinal::NORTH},
																										std::array<Direction::Cardinal, 2>{Direction::Cardinal::EAST, Direction::Cardinal::SOUTH}
																									});
const TrafficLight::TrafficLightStateOptions TrafficLight::TrafficLightStateOptions::STRAIGHT_YELLOW_LHD_2 = TrafficLight::TrafficLightStateOptions(2, TrafficLightColor::YELLOW, QVector<std::array<Direction::Cardinal, 2>> {
																										 std::array<Direction::Cardinal, 2>{Direction::Cardinal::WEST, Direction::Cardinal::EAST},
																										 std::array<Direction::Cardinal, 2>{Direction::Cardinal::EAST, Direction::Cardinal::WEST},
																										 std::array<Direction::Cardinal, 2>{Direction::Cardinal::WEST, Direction::Cardinal::NORTH},
																										 std::array<Direction::Cardinal, 2>{Direction::Cardinal::EAST, Direction::Cardinal::SOUTH}
																									 });

const std::array<TrafficLight::TrafficLightStateOptions, 8> TrafficLight::VALUES_LHD = std::array<TrafficLight::TrafficLightStateOptions, 8> {TrafficLight::TrafficLightStateOptions::RIGHT_GREEN_LHD_1,
																																									TrafficLight::TrafficLightStateOptions::RIGHT_YELLOW_LHD_1,
																																									TrafficLight::TrafficLightStateOptions::STRAIGHT_GREEN_LHD_1,
																																									TrafficLight::TrafficLightStateOptions::STRAIGHT_YELLOW_LHD_1,
																																									TrafficLight::TrafficLightStateOptions::RIGHT_GREEN_LHD_2,
																																									TrafficLight::TrafficLightStateOptions::RIGHT_YELLOW_LHD_2,
																																									TrafficLight::TrafficLightStateOptions::STRAIGHT_GREEN_LHD_2,
																																									TrafficLight::TrafficLightStateOptions::STRAIGHT_YELLOW_LHD_2
																																									};

unsigned int TrafficLight::TrafficLightStateOptions::idAssigner = 0;

TrafficLight::TrafficLightStateOptions::TrafficLightStateOptions() : TrafficLight::TrafficLightStateOptions::TrafficLightStateOptions(1, TrafficLightColor::ILLEGAL, QVector<std::array<Direction::Cardinal, 2>>()) {
}

TrafficLight::TrafficLightStateOptions::TrafficLightStateOptions(double durationSeconds, const TrafficLightColor &lightColor, const QVector<std::array<Direction::Cardinal, 2>> &directions) : id(idAssigner++), durationSeconds(durationSeconds), lightColor(lightColor), directions(directions) {
}

bool TrafficLight::mustStop(const TrafficLightColor &lightColor) {
	return lightColor == TrafficLightColor::ILLEGAL || lightColor == TrafficLightColor::RED;
}

unsigned int TrafficLight::TrafficLightStateOptions::getID() const {
	return this->id;
}

double TrafficLight::TrafficLightStateOptions::getDuration() const {
	return this->durationSeconds;
}

TrafficLightColor TrafficLight::TrafficLightStateOptions::getLightColor() const {
	return this->lightColor;
}

const QVector<std::array<Direction::Cardinal, 2>> TrafficLight::TrafficLightStateOptions::getDirections() const {
	return this->directions;
}

bool TrafficLight::TrafficLightStateOptions::operator==(const TrafficLight::TrafficLightStateOptions &option) const {
	return this->id == option.id;
}

TrafficLight::TrafficLight(Intersection *intersection) :
		intersection(intersection),
		timer(QElapsedTimer()),
		timeOffset(CustomRandom::getRand()),
		previousStateOptionID(std::numeric_limits<unsigned int>::max()),
		forceTimeState(0),
		lightControlComboBox(new LightControlComboBox(this)),
		speedLimitTextBox(new SpeedLimitTextBox(this)) {
    this->timer.start();
}

TrafficLight::TrafficLightState TrafficLight::getCurrentState() const {
	return getPriorState(0);
}

TrafficLight::TrafficLightState TrafficLight::getPriorState(double timePriorDifference) const {
	double timeElapsed = getPriorTimePosition(timePriorDifference);
	return TrafficLightState(this, timeElapsed, getCurrentStateOption(timeElapsed));
}

double TrafficLight::getTimerSecElapsed(bool includePause) const {
	return (this->timer.nsecsElapsed() - (includePause ? this->intersection->getTrafficEngine()->getPausedTime() : 0)) / 1000000000.0;
}

Intersection *TrafficLight::getIntersection() const {
	return this->intersection;
}

TrafficLight::LightControlComboBox *TrafficLight::getLightControlComboBox() const {
	return this->lightControlComboBox.get();
}

TrafficLight::SpeedLimitTextBox *TrafficLight::getSpeedLimitTextBox() const {
	return this->speedLimitTextBox.get();
}

bool TrafficLight::isPathLegal(Direction::Cardinal sourceDirection, Direction::Cardinal destinationDirection) const {
	int sourceLanesCount = this->intersection->getLanes(sourceDirection);
	for (int i = 0; i < sourceLanesCount; i++) {
		if (isPathLegal(sourceDirection, i, destinationDirection)) {
			return true;
		}
	}
	return false;
}

bool TrafficLight::isPathLegal(Direction::Cardinal sourceDirection, int sourceLane, Direction::Cardinal destinationDirection) const {
	int destinationLanesCount = this->intersection->getLanes(destinationDirection);
	for (int i = 0; i < destinationLanesCount; i++) {
		if (isPathLegal(sourceDirection, sourceLane, destinationDirection, i)) {
			return true;
		}
	}
	return false;
}

bool TrafficLight::isPathLegal(Direction::Cardinal sourceDirection, int sourceLane, Direction::Cardinal destinationDirection, int destinationLane) const {
	Intersection* intersection = getIntersection();
    int sourceLaneCount = intersection->getLanes(sourceDirection);
	int destinationLaneCount = intersection->getLanes(destinationDirection);
    if (sourceLane < 0 || sourceLane >= sourceLaneCount || destinationLane < 0 || destinationLane >= destinationLaneCount) {
		return false;
	} else if (sourceDirection == destinationDirection) {
		if (sourceLane != destinationLane && intersection->getConnectionsCount() == 1) {
			Road* connection = intersection->getConnection(destinationDirection);
			if (connection != nullptr) {
				int divider = connection->getDirectionDividerLane();
				bool isFixedIntersection = connection->getFixedIntersection() == intersection;
				if (isFixedIntersection) {
					if (divider >= 0) {
						return destinationLane >= destinationLaneCount - divider;
					} else {
						return destinationLane < -divider;
					}
				} else {
					if (divider >= 0) {
						return destinationLane >= divider;
					} else {
						return destinationLane < destinationLaneCount + divider;
					}
				}

			}
		}
		return false;
	} else {
		Road* sourceRoad = intersection->getConnection(sourceDirection);
		Road* destinationRoad = intersection->getConnection(destinationDirection);
		Direction::Cardinal left = Direction::getLeft(sourceDirection);
		Direction::Cardinal opposite = Direction::getOpposite(sourceDirection);
		Direction::Cardinal right = Direction::getRight(sourceDirection);
		Road* leftRoad = intersection->getConnection(left);
		Road* oppositeRoad = intersection->getConnection(opposite);
		Road* rightRoad = intersection->getConnection(right);
		if (sourceRoad == nullptr || destinationRoad == nullptr) {
			return false;
		} else {
			bool isFixedIntersectionForSource = sourceRoad->getFixedIntersection() == intersection;
			bool isFixedIntersectionForDestination = destinationRoad->getFixedIntersection() == intersection;
			bool isFixedIntersectionForLeft = leftRoad != nullptr && leftRoad->getFixedIntersection() == intersection;
			bool isFixedIntersectionForRight = rightRoad != nullptr && rightRoad->getFixedIntersection() == intersection;
			std::array<int, 2> availableSourceLanesRange = sourceRoad->getExitingLanesRange(isFixedIntersectionForSource);
			if (!isFixedIntersectionForSource) {
				int temp = availableSourceLanesRange[0];
				availableSourceLanesRange[0] = sourceLaneCount - 1 - availableSourceLanesRange[1];
				availableSourceLanesRange[1] = sourceLaneCount - 1 - temp;
			}
			std::array<int, 2> availableDestinationLanesRange = destinationRoad->getEnteringLanesRange(isFixedIntersectionForDestination);
			if (!isFixedIntersectionForDestination) {
				int temp = availableDestinationLanesRange[0];
				availableDestinationLanesRange[0] = destinationLaneCount - 1 - availableDestinationLanesRange[1];
				availableDestinationLanesRange[1] = destinationLaneCount - 1 - temp;
			}
			std::array<int, 2> availableLeftRoadLanesRange;
			if (leftRoad != nullptr) {
				availableLeftRoadLanesRange = leftRoad->getEnteringLanesRange(isFixedIntersectionForLeft);
				if (!isFixedIntersectionForLeft) {
					int temp = availableLeftRoadLanesRange[0];
					availableLeftRoadLanesRange[0] = leftRoad->getLanes() - 1 - availableLeftRoadLanesRange[1];
					availableLeftRoadLanesRange[1] = leftRoad->getLanes() - 1 - temp;
				}
			} else {
				availableLeftRoadLanesRange[0] = -2;
				availableLeftRoadLanesRange[1] = -1;
			}
			std::array<int, 2> availableRightRoadLanesRange;
			if (rightRoad != nullptr) {
				availableRightRoadLanesRange = rightRoad->getEnteringLanesRange(isFixedIntersectionForRight);
				if (!isFixedIntersectionForRight) {
					int temp = availableLeftRoadLanesRange[0];
					availableRightRoadLanesRange[0] = rightRoad->getLanes() - 1 - availableRightRoadLanesRange[1];
					availableRightRoadLanesRange[1] = rightRoad->getLanes() - 1 - temp;
				}
			} else {
				availableRightRoadLanesRange[0] = -2;
				availableRightRoadLanesRange[1] = -1;
			}
			const int availableSourceLanesCount = availableSourceLanesRange.at(1) - availableSourceLanesRange.at(0) + 1;
			const int availableDestinationLanesCount = availableDestinationLanesRange.at(1) - availableDestinationLanesRange.at(0) + 1;
			if (availableSourceLanesCount <= 0 || availableDestinationLanesCount <= 0) {
				return false;
			} else if (sourceLane < availableSourceLanesRange.at(0) || sourceLane > availableSourceLanesRange.at(1)) {
				return false;
			} else if (destinationLane < availableDestinationLanesRange.at(0) || destinationLane > availableDestinationLanesRange.at(1)) {
				return false;
			} else {
				QMap<int, std::vector<Direction::Cardinal>> possibleLaneDirections;
				for (int i = availableSourceLanesRange.at(0); i <= availableSourceLanesRange.at(1); i++) {
					possibleLaneDirections.insert(i, std::vector<Direction::Cardinal>());
				}
				int turnRightLanes = std::min((availableSourceLanesCount + 2) / 3, (availableDestinationLanesCount + 1) / 2);
				int turnLeftLanes = std::min((availableSourceLanesCount + 2) / 3, (availableDestinationLanesCount + 1) / 2);
				for (int i = 0; i < turnRightLanes; i++) {
					possibleLaneDirections[availableSourceLanesRange.at(0) + i].push_back(right);
				}
				if (turnRightLanes < availableSourceLanesCount - turnLeftLanes) {
					for (int i = std::max(turnRightLanes, 1) - 1; i <= availableSourceLanesCount - std::max(turnLeftLanes, 1); i++) {
						possibleLaneDirections[availableSourceLanesRange.at(0) + i].push_back(opposite);
					}
				} else if (oppositeRoad != nullptr) {
					int availableOppositeRoadLanes = oppositeRoad->getEnteringLanesCount(oppositeRoad->getFixedIntersection() == intersection);
					if (sourceRoad->getDirectionDividerLane() > 0) {
						for (int i = std::max(turnRightLanes - 1, 0); i < availableOppositeRoadLanes; i++) {
							possibleLaneDirections[availableSourceLanesRange.at(0) + i].push_back(opposite);
						}
					} else {
						for (int i = std::max(turnLeftLanes - 1, 0); i < availableOppositeRoadLanes; i++) {
							possibleLaneDirections[availableSourceLanesRange.at(1) - i].push_back(opposite);
						}
					}
				}
				for (int i = availableSourceLanesCount - turnLeftLanes; i < availableSourceLanesCount; i++) {
					possibleLaneDirections[availableSourceLanesRange.at(0) + i].push_back(left);
				}
				if (leftRoad == nullptr) {
					for (auto &k : possibleLaneDirections.keys()) {
						std::vector<Direction::Cardinal> &possibleDirections = possibleLaneDirections[k];
						std::replace(possibleDirections.begin(), possibleDirections.end(), left, opposite);
					}
				}
				if (rightRoad == nullptr) {
					for (auto &k : possibleLaneDirections.keys()) {
						std::vector<Direction::Cardinal> &possibleDirections = possibleLaneDirections[k];
						std::replace(possibleDirections.begin(), possibleDirections.end(), right, opposite);
					}
				}
				if (oppositeRoad == nullptr) {
					if (leftRoad == nullptr && rightRoad == nullptr) {
						return false;
					} else if (leftRoad == nullptr) {
						for (auto &k : possibleLaneDirections.keys()) {
							std::vector<Direction::Cardinal> &possibleDirections = possibleLaneDirections[k];
							std::replace(possibleDirections.begin(), possibleDirections.end(), opposite, right);
						}
					} else if (rightRoad == nullptr) {
						for (auto &k : possibleLaneDirections.keys()) {
							std::vector<Direction::Cardinal> &possibleDirections = possibleLaneDirections[k];
							std::replace(possibleDirections.begin(), possibleDirections.end(), opposite, left);
						}
					} else {
						int directionSplitLane = availableSourceLanesRange.at(0) + availableSourceLanesCount / 2;
						for (auto &k : possibleLaneDirections.keys()) {
							std::vector<Direction::Cardinal> &possibleDirections = possibleLaneDirections[k];
							std::replace(possibleDirections.begin(), possibleDirections.end(), opposite, k < directionSplitLane ? right : left);
						}
					}
				}
				if (destinationDirection == Direction::getLeft(sourceDirection)) {
					const std::vector<Direction::Cardinal> &possibleDirections = possibleLaneDirections.value(sourceLane);
					if (std::find(possibleDirections.begin(), possibleDirections.end(), left) == possibleDirections.end()) {
						return false;
					} else {
						int sourcePosition = availableSourceLanesRange.at(1) - sourceLane;
						int destinationPosition = destinationLane - availableDestinationLanesRange.at(0);
						return destinationPosition == sourcePosition || (destinationPosition < sourcePosition && destinationPosition + 1 >= availableDestinationLanesCount);
					}
				} else if (destinationDirection == Direction::getRight(sourceDirection)) {
					const std::vector<Direction::Cardinal> &possibleDirections = possibleLaneDirections.value(sourceLane);
					if (std::find(possibleDirections.begin(), possibleDirections.end(), right) == possibleDirections.end()) {
						return false;
					} else {
						int sourcePosition = sourceLane - availableSourceLanesRange.at(0);
						int destinationPosition = availableDestinationLanesRange.at(1) - destinationLane;
						return destinationPosition == sourcePosition || (destinationPosition < sourcePosition && destinationPosition + 1 >= availableDestinationLanesCount);
					}
				} else {
					const std::vector<Direction::Cardinal> &possibleDirections = possibleLaneDirections.value(sourceLane);
					if (std::find(possibleDirections.begin(), possibleDirections.end(), opposite) == possibleDirections.end()) {
						return false;
					} else {
						int towardOppositeMinLane = sourceLaneCount - 1;
						int towardOppositeMaxLane = 0;
						for (auto &k : possibleLaneDirections.keys()) {
							const std::vector<Direction::Cardinal> &possibleDirections2 = possibleLaneDirections.value(k);
							if (std::find(possibleDirections2.begin(), possibleDirections2.end(), opposite) != possibleDirections2.end()) {
								towardOppositeMinLane = std::min(towardOppositeMinLane, k);
								towardOppositeMaxLane = std::max(towardOppositeMaxLane, k);
							}
						}
						if (availableSourceLanesCount <= availableDestinationLanesCount) {
							int sourcePosition = sourceLane - availableSourceLanesRange.at(0);
							int destinationPosition = availableDestinationLanesRange.at(1) - destinationLane;
							return sourcePosition == destinationPosition;
						} else {
							int towardOppositeCenterLane = (towardOppositeMinLane + towardOppositeMaxLane) / 2;
							int oppositeCenterLane = (availableDestinationLanesRange.at(0) + availableDestinationLanesRange.at(1)) / 2;
							int destinationRightSideLanesCount = availableDestinationLanesCount / 2;
							if (sourceLane <= towardOppositeCenterLane) {
								int sourcePosition = sourceLane - towardOppositeMinLane;
								int destinationPosition = availableDestinationLanesRange.at(1) - destinationLane;
								return sourcePosition == destinationPosition || (sourcePosition >= destinationRightSideLanesCount && destinationLane == oppositeCenterLane);
							} else {
								int sourcePosition = towardOppositeMaxLane - sourceLane;
								int destinationPosition = destinationLane - availableDestinationLanesRange.at(0);
								return sourcePosition == destinationPosition || (sourcePosition >= destinationLaneCount - destinationRightSideLanesCount && destinationLane == oppositeCenterLane);
							}
						}
					}
				}
			}
		}
    }
    return true;
}

int TrafficLight::getLegalPathsCount(VehiclePath *currentPath, int sourceLane) const {
    Intersection *intersection = getIntersection();
    if (intersection != nullptr && currentPath != intersection) {
        for (int i = 0; i < Direction::DIRECTIONS_COUNT; i++) {
            Direction::Cardinal c = Direction::getCardinal(i);
            if (currentPath == intersection->getConnection(c)) {
                return getLegalPathsCount(c, sourceLane);
            }
        }
    }
	return 0;
}

int TrafficLight::getLegalPathsCount(Direction::Cardinal sourceDirection, int sourceLane) const {
    int legalPaths = 0;
    Intersection *intersection = getIntersection();
    for (int i = 0; i < Direction::DIRECTIONS_COUNT; i++) {
        Direction::Cardinal c = Direction::getCardinal(i);
        int lanes = intersection->getLanes(c);
        for (int j = 0; j < lanes; j++) {
            if (isPathLegal(sourceDirection, sourceLane, c, j)) {
                legalPaths++;
            }
        }
    }
    return legalPaths;
}

int TrafficLight::getDestinationPathsCount(const IntersectionLane &sourceLane) const {
	int count = 0;
	Intersection *intersection = getIntersection();
	for (int i = 0; i < Direction::DIRECTIONS_COUNT; i++) {
		Direction::Cardinal c = Direction::getCardinal(i);
		int lanes = intersection->getLanes(c);
		for (int j = 0; j < lanes; j++) {
			if (isPathLegal(sourceLane.getDirection(), sourceLane.getLane(), c, j)) {
				count++;
			}
		}
	}
	return count;
}
bool TrafficLight::hasPathInCurrentStateOptions(const Direction::Cardinal &sourceDirection, const Direction::Cardinal &destinationDirection) const {
	for (auto &o : getCurrentStateOptions()) {
		if (!TrafficLight::mustStop(o.getLightColor())) {
			for (auto &d : o.getDirections()) {
				if (d.at(0) == sourceDirection && d.at(1) == destinationDirection) {
					return true;
				}
			}
		}
	}
	return false;
}

bool TrafficLight::hasPath(const Direction::Cardinal &sourceDirection, const Direction::Cardinal &destinationDirection) const {
	return isPathLegal(sourceDirection, destinationDirection) && hasPathInCurrentStateOptions(sourceDirection, destinationDirection);
}

bool TrafficLight::hasPath(const Direction::Cardinal &sourceDirection, int sourceLane, const Direction::Cardinal &destinationDirection) const {
	return isPathLegal(sourceDirection, sourceLane, destinationDirection) && hasPathInCurrentStateOptions(sourceDirection, destinationDirection);
}

bool TrafficLight::hasPath(const Direction::Cardinal &sourceDirection, int sourceLane, const Direction::Cardinal &destinationDirection, int destinationLane) const {
	return isPathLegal(sourceDirection, sourceLane, destinationDirection, destinationLane) && hasPathInCurrentStateOptions(sourceDirection, destinationDirection);
}

std::vector<Direction::Cardinal> TrafficLight::getPossibleDirections(const IntersectionLane &sourceLane) const {
	std::vector<Direction::Cardinal> possibleDirections;
	for (int i = 0; i < Direction::DIRECTIONS_COUNT; i++) {
		const Direction::Cardinal &d = Direction::getCardinal(i);
		if (hasPath(sourceLane.getDirection(), sourceLane.getLane(), d)) {
			possibleDirections.push_back(d);
		}
	}
	return possibleDirections;
}

std::vector<IntersectionLane> TrafficLight::getPossiblePaths(const IntersectionLane &sourceLane) const {
	std::vector<IntersectionLane> possiblePaths;
	for (int i = 0; i < Direction::DIRECTIONS_COUNT; i++) {
		const Direction::Cardinal &d = Direction::getCardinal(i);
		int lanes = this->intersection->getLanes(d);
		for (int l = 0; l < lanes; l++) {
			if (hasPath(sourceLane.getDirection(), sourceLane.getLane(), d, l)) {
				possiblePaths.push_back(IntersectionLane(d, l));
			}
		}
	}
	return possiblePaths;
}

void TrafficLight::update() {
	unsigned int currentID = getCurrentStateOption(getCurrentTimePosition()).getID();
	if (currentID != this->previousStateOptionID) {
		this->previousStateOptionID = currentID;
		this->intersection->update();
	}
}

double TrafficLight::getCurrentTimePosition() const {
	return getPriorTimePosition(0);
}

double TrafficLight::getPriorTimePosition(double timePriorDifferenceInSeconds) const {
	double totalDuration = 0;
	for (auto &o : getCurrentStateOptions()) {
		totalDuration += o.getDuration();
	}
	return fmodf(fmodf(getTimerSecElapsed(true) + (1 - this->timeOffset) * totalDuration / 1000000000.0 - timePriorDifferenceInSeconds, totalDuration) + totalDuration, totalDuration);
}

TrafficLight::TrafficLightStateOptions TrafficLight::getCurrentStateOption(double timeElasped) const {
	const std::vector<TrafficLightStateOptions> &avaliableStateOptions = getCurrentStateOptions();
	if (this->forceTimeState <= 0) {
		TrafficLightStateOptions currentStateOption = TrafficLightStateOptions::ALL_RED;
		for (unsigned int i = 0; i < avaliableStateOptions.size(); i++) {
			const TrafficLightStateOptions &o = avaliableStateOptions.at(i);
			timeElasped -= o.getDuration();
			if (timeElasped < 0) {
				currentStateOption = o;
				break;
			}
		}
		return currentStateOption;
	} else {
		return avaliableStateOptions.at((this->forceTimeState - 1) % avaliableStateOptions.size());
	}
}

std::vector<TrafficLight::TrafficLightStateOptions> TrafficLight::getCurrentStateOptions() const {
	Intersection *intersection = getIntersection();
	std::vector<TrafficLightStateOptions> avaliableStateOptions;
	const std::array<TrafficLightStateOptions, 8> &stateOptions = this->intersection->getTrafficEngine()->isRightHandDrive() ? TrafficLight::VALUES_RHD : TrafficLight::VALUES_LHD;
	for (unsigned int i = 0; i < stateOptions.size(); i++) {
		const TrafficLightStateOptions &o = stateOptions[i];
		const QVector<std::array<Direction::Cardinal, 2>> &directions = o.getDirections();
		for (int j = 0; j < directions.size(); j++) {
			const std::array<Direction::Cardinal, 2> &sourceAndDestination = directions.at(j);
			if (intersection->hasLaneEntering(sourceAndDestination[0]) && intersection->hasLaneExiting(sourceAndDestination[1])) {
				avaliableStateOptions.push_back(o);
				break;
			}
		}
	}
	// each avaliableStateOptionsAvailablePaths corresponds the avaliableStateOptions with the same index
	QVector<QVector<std::array<Direction::Cardinal, 2>>> avaliableStateOptionsAvailablePaths;
	for (auto &o : avaliableStateOptions) {
		QVector<std::array<Direction::Cardinal, 2>> availablePaths;
		for (auto &p : o.getDirections()) {
			if (intersection->hasLaneEntering(p[0]) && intersection->hasLaneExiting(p[1])) {
				availablePaths.append(p);
			}
		}
		avaliableStateOptionsAvailablePaths.append(availablePaths);
	}
	QMap<int, int> superset;
	QSet<int> toBeRemovedOptions;
	for (int i = 0; i < avaliableStateOptionsAvailablePaths.size(); i++) {
		const QVector<std::array<Direction::Cardinal, 2>> &availablePaths1 = avaliableStateOptionsAvailablePaths.at(i);
		if (!toBeRemovedOptions.contains(i)) {
			for (int j = 0; j < avaliableStateOptionsAvailablePaths.size(); j++) {
				if (i != j && !toBeRemovedOptions.contains(j)) {
					const QVector<std::array<Direction::Cardinal, 2>> &availablePaths2 = avaliableStateOptionsAvailablePaths.at(j);
					int pathsFoundIn1 = 0;
					for (int k = 0; k < availablePaths2.size(); k++) {
						const std::array<Direction::Cardinal, 2> &a2 = availablePaths2.at(k);
						for (int l = 0; l < availablePaths1.size(); l++) {
							const std::array<Direction::Cardinal, 2> &a1 = availablePaths1.at(l);
							if (a1[0] == a2[0] && a1[1] == a2[1]) {
								pathsFoundIn1++;
								break;
							}
						}
					}
					if (pathsFoundIn1 == availablePaths2.size()) {
						// availablePaths2 is a subset of availablePaths1
						const TrafficLightStateOptions &o1 = avaliableStateOptions.at(i);
						const TrafficLightStateOptions &o2 = avaliableStateOptions.at(j);
						// if (availablePaths1.size() == availablePaths2.size() && o1.getDuration() < o2.getDuration()) {
						if (availablePaths1.size() == availablePaths2.size() && o1.getLightColor() != o2.getLightColor() &&
								(o2.getLightColor() == TrafficLightColor::GREEN
								|| (o2.getLightColor() == TrafficLightColor::YELLOW && o1.getLightColor() != TrafficLightColor::GREEN)
								|| (o2.getLightColor() == TrafficLightColor::RED && o1.getLightColor() == TrafficLightColor::ILLEGAL))) {
							superset.insert(i, superset.value(j, j));
						} else {
							superset.insert(j, superset.value(i, i));
						}
						if (o1.getLightColor() == o2.getLightColor()) {
							toBeRemovedOptions.insert(j);
							superset.remove(j);
						}
					}
				}
			}
		}
	}
	if (superset.size() + 1 == avaliableStateOptionsAvailablePaths.size() - toBeRemovedOptions.size()) {
		for (int i = 0; i < avaliableStateOptionsAvailablePaths.size(); i++) {
			if (superset.contains(i)) {
				toBeRemovedOptions.insert(i);
			}
		}
	}
	for (int i = avaliableStateOptions.size() - 1; i >= 0; i--) {
		if (toBeRemovedOptions.contains(i)) {
			avaliableStateOptions.erase(avaliableStateOptions.begin() + i);
		}
	}
	return avaliableStateOptions;
}

TrafficLight::TrafficLightState::TrafficLightState(const TrafficLight *trafficLight, double timePosition, const TrafficLight::TrafficLightStateOptions &state) : trafficLight(trafficLight), timePosition(timePosition), state(state) {
}

TrafficLightColor TrafficLight::TrafficLightState::getLightColor(Direction::Cardinal sourceDirection, int sourceLane) const {
	const QVector<std::array<Direction::Cardinal, 2>> &stateOptions = this->state.getDirections();
	for (int i = 0; i < stateOptions.size(); i++) {
		const std::array<Direction::Cardinal, 2> &element = stateOptions.at(i);
		if (element[0] == sourceDirection && this->trafficLight->isPathLegal(sourceDirection, sourceLane, element[1])) {
			return this->state.getLightColor();
		}
	}
	if (this->trafficLight->getIntersection()->getConnectionsCount() == 1) {
		for (int i = 0; i < this->trafficLight->getIntersection()->getLanes(sourceDirection); i++) {
			if (this->trafficLight->isPathLegal(sourceDirection, sourceLane, sourceDirection, i)) {
				return TrafficLightColor::GREEN;
			}
		}
	}
	return TrafficLightColor::ILLEGAL;
}

TrafficLightColor TrafficLight::TrafficLightState::getLightColor(Direction::Cardinal sourceDirection, int sourceLane, Direction::Cardinal destinationDirection, int destinationLane) const {
	if (sourceDirection == destinationDirection) {
		return this->trafficLight->isPathLegal(sourceDirection, sourceLane, destinationDirection, destinationLane) ? TrafficLightColor::GREEN : TrafficLightColor::ILLEGAL;
	} else if (this->trafficLight->isPathLegal(sourceDirection, sourceLane, destinationDirection, destinationLane)) {
		const QVector<std::array<Direction::Cardinal, 2>> &stateOptions = this->state.getDirections();
		for (int i = 0; i < stateOptions.size(); i++) {
			const std::array<Direction::Cardinal, 2> &element = stateOptions.at(i);
			if (element[0] == sourceDirection && element[1] == destinationDirection) {
				return this->state.getLightColor();
			}
		}
	}
    return TrafficLightColor::ILLEGAL;
}

double TrafficLight::TrafficLightState::getTimeToNextLight(const TrafficLightColor &lightColor, const IntersectionLane &sourceLane) const {
	return getTimeToNextLight(lightColor, sourceLane, sourceLane);
}

double TrafficLight::TrafficLightState::getTimeToNextLight(const TrafficLightColor &lightColor, const IntersectionLane &sourceLane, const IntersectionLane &destinationLane) const {
	bool ignoreDestination = sourceLane == destinationLane;
	const TrafficLightColor &currentColor = ignoreDestination ?
				getLightColor(sourceLane.getDirection(), sourceLane.getLane()) :
				getLightColor(sourceLane.getDirection(), sourceLane.getLane(), destinationLane.getDirection(), destinationLane.getLane());
	if (currentColor == lightColor) {
		return 0;
	} else {
		if (this->trafficLight->forceTimeState <= 0) {
			const std::vector<TrafficLightStateOptions> &avaliableStateOptions = this->trafficLight->getCurrentStateOptions();
			double totalDuration = 0;
			for (unsigned int i = 0; i < avaliableStateOptions.size(); i++) {
				totalDuration += avaliableStateOptions.at(i).getDuration();
			}
			const TrafficLightStateOptions &currentStateOption = this->state;
			int currentStateOptionIndex = -1;
			for (unsigned int i = 0; i < avaliableStateOptions.size(); i++) {
				if (avaliableStateOptions.at(i) == currentStateOption) {
					currentStateOptionIndex = i;
					break;
				}
			}
			if (currentStateOptionIndex == -1) {
				return INFINITY;
			} else {
				double timeRemainingOnCurrentState = std::fmod(this->timePosition, totalDuration);
				for (int i = 0; i <= currentStateOptionIndex; i++) {
					timeRemainingOnCurrentState -= avaliableStateOptions.at(i).getDuration();
				}
				double waitTime = -timeRemainingOnCurrentState;
				for (unsigned int i = currentStateOptionIndex + 1; i < currentStateOptionIndex + avaliableStateOptions.size(); i++) {
					const TrafficLightStateOptions &o = avaliableStateOptions.at(i % avaliableStateOptions.size());

					TrafficLightColor optionLightColorForSelectedPath = TrafficLightColor::ILLEGAL;
					const QVector<std::array<Direction::Cardinal, 2>> &directions = o.getDirections();
					for (int j = 0; j < directions.size(); j++) {
						const std::array<Direction::Cardinal, 2> &sourceDestinationDirections = directions.at(j);
						if (sourceDestinationDirections[0] == sourceLane.getDirection() && (ignoreDestination || sourceDestinationDirections[1] == destinationLane.getDirection()) &&
								this->trafficLight->isPathLegal(sourceLane.getDirection(), sourceLane.getLane(), sourceDestinationDirections[1])) {
							optionLightColorForSelectedPath = o.getLightColor();
							break;
						}
					}

					if (optionLightColorForSelectedPath == lightColor) {
						return waitTime;
					}
					waitTime += o.getDuration();
				}
				return INFINITY;
			}
		} else {
			return INFINITY;
		}
	}
}

bool TrafficLight::TrafficLightState::hasPath(const Direction::Cardinal &sourceDirection, const Direction::Cardinal &destinationDirection) const {
	for (auto &d : this->state.getDirections()) {
		if (d.at(0) == sourceDirection && d.at(1) == destinationDirection) {
			return true;
		}
	}
	return false;
}

TrafficLight::LightControlComboBox::LightControlComboBox(TrafficLight *trafficLight, QWidget *parent) : QComboBox(parent), trafficLight(trafficLight) {
	connect(this, SIGNAL(currentIndexChanged(int)), this, SLOT(indexChanged(int)));
	addItem(QString::fromStdString("Running"));
	updateOptions();
}

void TrafficLight::LightControlComboBox::updateOptions() {
	int itemCount = count();
	const std::vector<TrafficLight::TrafficLightStateOptions> &options = this->trafficLight->getCurrentStateOptions();
	int optionsCount = options.size() + 1;
	if (optionsCount < itemCount) {
		for (int i = itemCount - 1; i >= optionsCount; i--) {
			removeItem(i);
		}
	} else if (itemCount < optionsCount) {
		for (int i = itemCount; i < optionsCount; i++) {
			addItem(QString::fromStdString("State " + std::to_string(i)));
		}
	}
}

void TrafficLight::LightControlComboBox::indexChanged(int index) {
	this->trafficLight->forceTimeState = index;
	this->trafficLight->update();
}

TrafficLight::SpeedLimitTextBox::SpeedLimitTextBox(TrafficLight *trafficLight, QWidget *parent) : QDoubleSpinBox(parent), trafficLight(trafficLight) {
	connect(this, SIGNAL(valueChanged(double)), this, SLOT(valueChangedSlot(double)));
	setSingleStep(5.0);
	setMinimum(singleStep());
	setMaximum(Speeds::MAXIMUM_SPEED_LIMIT);
	setValue(trafficLight->getIntersection()->getSpeedLimit());
}

void TrafficLight::SpeedLimitTextBox::valueChangedSlot(double newValue) {
	this->trafficLight->getIntersection()->setSpeedLimit(newValue, false);
}

QDebug operator<<(QDebug debug, const TrafficLightColor &trafficLightColor) {
	if (trafficLightColor == TrafficLightColor::GREEN) {
		debug << "GREEN";
	} else if (trafficLightColor == TrafficLightColor::YELLOW) {
		debug << "YELLOW";
	} else if (trafficLightColor == TrafficLightColor::RED) {
		debug << "RED";
	} else {
		debug << "ILLEGAL";
	}
	return debug;
}
