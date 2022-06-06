#include "Constraint.h"

float Constraint::constraint() {};
void Constraint::derCon() {};
float Constraint::constraintDerivative() {};
std::vector<Vec2f> Constraint::J() {};
std::vector<Vec2f> Constraint::JDerivative() {};
void Constraint::draw() {};
void Constraint::apply() {};
