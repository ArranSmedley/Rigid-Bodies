#define _USE_MATH_DEFINES
#include <cmath>
#include "Particle.h"



Particle::Particle()
{
	setMesh(Mesh::Mesh(Mesh::QUAD));
	scale(glm::vec3(.1f, .1f, .1f)); 
	rotate((GLfloat)M_PI_2, glm::vec3(1.0f, 0.0f, 0.0f));
	translate(glm::vec3(0.0f, 2.5f, 0.0f));
	setAcc(glm::vec3(0.0f, 0.0f, 0.0f)); 
	setVel(glm::vec3(0.0f, 0.0f, 0.0f));

	setMass(1.0f); 
	setCor(1.0f);

	getMesh().setShader(Shader("resources/shaders/solid.vert", "resources/shaders/solid_red.frag"));
}


Particle::~Particle()
{
}
