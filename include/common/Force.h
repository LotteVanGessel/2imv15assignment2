#pragma once

#include "Particle.h"
#include <vector>

using namespace std;


class Force {
public:
	std::vector<Particle*> particles;
	virtual void apply() = 0;
	virtual void draw() = 0;
};