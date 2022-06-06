#pragma once

#include <gfx/vec2.h>

class Particle
{
public:

	Particle(const Vec2f & ConstructPos, float mass);
	virtual ~Particle(void);

	void reset();
	void draw();
	void setAnchor();

	Vec2f m_ConstructPos;
	Vec2f m_Position;
	Vec2f m_Velocity;
	Vec2f m_Force;
	Vec2f v_old;
	Vec2f pos_old;
	Vec2f F_old;
	float m_Mass;
	bool anchored;
};
