#include <iostream>
#include <cmath>
#include "Force.h"
#include "Body.h"
#include "glm/ext.hpp"


glm::vec3 Force::apply(float mass, const glm::vec3& pos, const glm::vec3& vel)
{ return glm::vec3(0.0f); }
/* ** GRAVITY */
glm::vec3 Gravity::apply(float mass, const glm::vec3& pos, const glm::vec3& vel) 
{ // complete. Should return the gravity force 
	return glm::vec3(0.0f, -9.8f, 0.0f);
}
	/* ** DRAG */
	glm::vec3 Drag::apply(float mass, const glm::vec3 & pos, const glm::vec3 & vel) { // complete. Should return the aerodynamic drag force 
		glm::vec3 drag = (1.225f * (normalize(vel) * normalize(vel)) * 1.05f * (-vel / -normalize(vel)) / 2);
		return drag;
	}

	/* ** HOOKE’S LAW */ 
	glm::vec3 Hooke::apply(float mass, const glm::vec3& pos, const glm::vec3& vel) { 

	
		glm::vec3 force;
	/*	glm::vec3 I = normalize(m_b1->getPos() - m_b2->getPos());

		glm::vec3 e = (m_b2->getPos() - m_b1->getPos()) / I;
		glm::vec3 v1 = glm::vec3(dot(e, m_b1->getVel()));
		glm::vec3 v2 = glm::vec3(dot(e, m_b2->getVel()));*/




		float l = length(m_b2->getPos() - m_b1->getPos());
		glm::vec3 e = normalize(m_b2->getPos() - m_b1->getPos());
		float v1 = glm::dot(e, m_b1->getVel());
		float v2 = glm::dot(e, m_b2->getVel());
		float springX = -m_ks * (m_rest - l);
		float dampX = -m_kd * (v1 - v2);
		force = (springX + dampX) * e;



		return force;

		}