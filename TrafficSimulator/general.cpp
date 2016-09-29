#include "general.h"

#include <QDebug>

double CustomRandom::getRand() {
	return ((double) rand() / (RAND_MAX));
}
