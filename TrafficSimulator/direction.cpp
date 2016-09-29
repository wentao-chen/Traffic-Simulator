#include "direction.h"

int Direction::intValue(Orientation orientation) {
    return orientation == Orientation::HORIZONTAL ? 0 : 1;
}

Orientation Direction::getOrientation(Direction::Cardinal cardinal) {
    return cardinal == Direction::Cardinal::NORTH || cardinal == Direction::Cardinal::SOUTH ? Orientation::HORIZONTAL : Orientation::VERTICAL;
}

int Direction::intValue(Direction::Cardinal cardinal) {
    if (cardinal == Direction::Cardinal::NORTH) {
        return 0;
    } else if (cardinal == Direction::Cardinal::EAST) {
        return 1;
    } else if (cardinal == Direction::Cardinal::SOUTH) {
        return 2;
    } else {
        return 3;
    }
}

Direction::Cardinal Direction::getCardinal(int i) {
    i = (i % 4 + 4) % 4;
    if (i == 0) {
        return Direction::Cardinal::NORTH;
    } else if (i == 1) {
        return Direction::Cardinal::EAST;
    } else if (i == 2) {
        return Direction::Cardinal::SOUTH;
    } else {
        return Direction::Cardinal::WEST;
    }
}

Direction::Cardinal Direction::getLeft(Direction::Cardinal cardinal) {
    if (cardinal == Direction::Cardinal::NORTH) {
        return Direction::Cardinal::EAST;
    } else if (cardinal == Direction::Cardinal::EAST) {
        return Direction::Cardinal::SOUTH;
    } else if (cardinal == Direction::Cardinal::SOUTH) {
        return Direction::Cardinal::WEST;
    } else {
        return Direction::Cardinal::NORTH;
    }
}

Direction::Cardinal Direction::getOpposite(Direction::Cardinal cardinal) {
    if (cardinal == Direction::Cardinal::NORTH) {
        return Direction::Cardinal::SOUTH;
    } else if (cardinal == Direction::Cardinal::EAST) {
        return Direction::Cardinal::WEST;
    } else if (cardinal == Direction::Cardinal::SOUTH) {
        return Direction::Cardinal::NORTH;
    } else {
        return Direction::Cardinal::EAST;
    }
}

Direction::Cardinal Direction::getRight(Direction::Cardinal cardinal) {
    if (cardinal == Direction::Cardinal::NORTH) {
        return Direction::Cardinal::WEST;
    } else if (cardinal == Direction::Cardinal::EAST) {
        return Direction::Cardinal::NORTH;
    } else if (cardinal == Direction::Cardinal::SOUTH) {
        return Direction::Cardinal::EAST;
    } else {
        return Direction::Cardinal::SOUTH;
    }
}

Orientation Direction::getOpposite(Orientation orientation) {
    return orientation == Orientation::HORIZONTAL ? Orientation::VERTICAL : Orientation::HORIZONTAL;
}

Orientation Direction::getOrientation(int i) {
    return (i % 2 + 2) % 2 == 0 ? Orientation::HORIZONTAL : Orientation::VERTICAL;
}

std::string Direction::toString(const Cardinal &cardinal) {
	if (cardinal == Direction::Cardinal::NORTH) {
		return "North";
	} else if (cardinal == Direction::Cardinal::EAST) {
		return "East";
	} else if (cardinal == Direction::Cardinal::SOUTH) {
		return "South";
	} else {
		return "West";
	}
}

QDebug operator<<(QDebug debug, const Direction::Cardinal &direction) {
	debug << Direction::toString(direction).c_str();
	return debug;
}
