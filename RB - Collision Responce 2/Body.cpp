#define _USE_MATH_DEFINES
#include <cmath>
#include "Body.h"



Body::Body()
{
}


Body::~Body()
{
}

void Body::translate(const glm::vec3 &vect) {

	m_pos = m_pos + vect;
	m_mesh.translate(vect);
}

void Body::rotate(float angle, const glm::vec3 &vect) { 
	m_mesh.rotate(angle, vect); }

void Body::scale(const glm::vec3 &vect) {
	m_mesh.scale(vect); }