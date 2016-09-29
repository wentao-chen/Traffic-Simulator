#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <string>

namespace FrameTime {
	const float FRAME_DELAY = 1.0 / 30.0;

	const float INTERSECTION_FLASH_DURATION = 0.2;
	const float INTERSECTION_FLASH_PERIOD = 0.5;
}

namespace Sizes {
    const float LANE_WIDTH = 20;
}

namespace Distances {
	const double LANE_CHANGE_DISTANCE = 40;
	const double LANE_CHANGE_BUFFER_DISTANCE = 50;
}

namespace Speeds {
	const double INTERSECTION_SPEED = 60;
    const double LOW_SPEED = 40;
    const double AVERAGE_SPEED = 60;
    const double HIGH_SPEED = 100;

    const double AVERAGE_ACCELERATION = 40;
    const double COMFORTABLE_DECELERATION = 30;

    const double AVERAGE_SAFETY_DISTANCE = 10;

	const double MAXIMUM_SPEED_LIMIT = 200;

	const double LANE_CHANGE_RATE = 0.5;
}

namespace Description {
	const std::string NO_VEHICLE_AHEAD_MESSAGE = "Open road";

	const std::string LEFT_HAND_TRAFFIC = "Left-Hand Traffic";
	const std::string RIGHT_HAND_TRAFFIC = "Right-Hand Traffic";
}

namespace Help {
	const std::string MAIN_HELP_MESSAGE = std::string("Intersections:\n") +
			"  - Drag to move intersections\n" +
			"  - Hover over the top-right or bottom-left corner of the intersection and drag to change the intersection size" + "\n" +
			"  - Hover over the bottom-right of the intersection and drag to rotate the intersection" + "\n" +
			"  - Hover over the inner edge of any intersection and drag to create a new road" + "\n" +
			"\n" +
			"Roads:" + "\n" +
			"  - Drag road to edge of a different intersection when creating a road to connect it to the intersection" + "\n" +
			"  - Right click when creating a road to remove";
}

#endif // CONSTANTS_H
