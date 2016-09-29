#ifndef DIRECTION_H
#define DIRECTION_H

#include <QDebug>
#include <string>

enum class Orientation {
    HORIZONTAL = 0,
    VERTICAL
};

namespace Direction {
	enum class Cardinal {
		NORTH = 0,
		EAST,
		SOUTH,
		WEST
	};

	const int ORIENTATIONS_COUNT = 2;
	const int DIRECTIONS_COUNT = 4;

	int intValue(Orientation orientation);

	Orientation getOrientation(int i);

	int intValue(Cardinal cardinal);

	Cardinal getCardinal(int i);

	Orientation getOrientation(Cardinal cardinal);

	Cardinal getLeft(Cardinal cardinal);

	Cardinal getOpposite(Cardinal cardinal);

	Cardinal getRight(Cardinal cardinal);

	Orientation getOpposite(Orientation orientation);

	std::string toString(const Cardinal &cardinal);
}

QDebug operator<<(QDebug debug, const Direction::Cardinal &direction);

#endif // DIRECTION_H
