#pragma region Includes
#pragma once
// Math constants
#define _USE_MATH_DEFINES
#include <cmath>  
#include <random>

// Std. Includes
#include <string>
#include <time.h>

// GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_operation.hpp>
#include "glm/ext.hpp"

// Other Libs
#include "SOIL2/SOIL2.h"

// project includes
#include "Application.h"
#include "Shader.h"
#include "Mesh.h"
#include "Particle.h"
#include "RigidBody.h"

using namespace glm;

#pragma endregion

void integrate(RigidBody& rb, double time, double deltaTime);
void impulse(RigidBody& rb, vec3 impulse, vec3 point);

Gravity g = Gravity(vec3(0, -9.8f, 0));


// main function
int main()
{
	// create application
	Application app = Application::Application();
	app.initRender();
	Application::camera.setCameraPosition(glm::vec3(0.0f, 5.0f, 20.0f));

	Mesh plane = Mesh::Mesh(Mesh::QUAD);
	// scale it up x5
	plane.scale(glm::vec3(100.0f, 10.0f, 100.0f));
	Shader lambert = Shader("resources/shaders/physics.vert", "resources/shaders/physics.frag");
	plane.setShader(lambert);

	RigidBody rb = RigidBody();
	Mesh m = Mesh::Mesh(Mesh::CUBE);
	m.scale(vec3(1, 3, 1));
	rb.setMesh(m);
	Shader rbShader = Shader("resources/shaders/physics.vert", "resources/shaders/physics.frag");
	rb.getMesh().setShader(rbShader);

	rb.setMass(2.0f);
	rb.translate(vec3(0.0f, 8.0f, 0.0f));
	rb.setVel(vec3(0.0f, 0.0f, 0.0f));
	rb.setAngVel(vec3(0.5f, 0.5f, 0.5f));

	rb.addForce(&g);

	mat3 inertiamatrix = mat3(((rb.getMass() * (pow(2, 2) + pow(6, 2))) / 12), 0, 0,0, (rb.getMass() * (pow(2, 2) + pow(2, 2)) / 12), 0, 0, 0, (rb.getMass() * (pow(6, 2) + pow(2, 2)) / 12));

	rb.setInvInertia(inertiamatrix);

	std::cout << to_string(rb.getInvInertia()) << std::endl;

	bool impulseApplied = false;

	double time = 0.0f;
	double deltaTime = 0.001f;

	double currentTime = glfwGetTime();
	double accumulator = 0.0f;



	// Game loop
	while (!glfwWindowShouldClose(app.getWindow()))
	{
		double newTime = glfwGetTime();
		double frameTime = newTime - currentTime;
		currentTime = newTime;
		accumulator += frameTime;

		while (accumulator >= deltaTime)
		{
			
			rb.setInvInertia((rb.getRotate() * inverse(rb.getInvInertia()) * transpose(rb.getRotate())));
			
			integrate(rb, time, deltaTime);

			accumulator -= deltaTime;
			time += deltaTime;
		}


		app.doMovement(deltaTime * 100);
		app.clear();
		app.draw(plane);
		app.draw(rb.getMesh());
		app.display();
	}

	app.terminate();

	return EXIT_SUCCESS;
}

void impulse(RigidBody& rb, vec3 impulse, vec3 point)
{
	rb.setVel(impulse / rb.getMass());
	vec3 r = point - rb.getPos();
	rb.setAngVel(rb.getInvInertia() * (cross(r, impulse)));
}


void integrate(RigidBody& rb, double time, double deltaTime)
{
	//Elasticity
	float e = 0.7f;

	//Calculate forces
	vec3 f = rb.applyForces(rb.getPos(), rb.getVel(), time, deltaTime);

	vec3 v = rb.getVel() + f / rb.getMass() * deltaTime;

	//Calculate position
	vec3 r = rb.getPos() + v * deltaTime;

	rb.setAngVel(rb.getAngVel() + deltaTime * rb.getAngAcc());

	//Calculate rotate matrix
	mat3 angVelSkew = matrixCross3(rb.getAngVel());
	mat3 R = mat3(rb.getRotate());
	R += deltaTime * angVelSkew * R;
	R = orthonormalize(R);

	rb.translate(r - rb.getPos());
	rb.setPos(r);
	rb.setVel(v);
	rb.setRotate(mat4(R));

	//Check for collision
	std::vector<Vertex> vert = rb.getMesh().getVertices();
	mat4 m = rb.getMesh().getModel();
	bool colResponce = false;
	std::vector<Vertex> vertices = std::vector<Vertex>();
	vec3 average = vec3();

	//Checking all vertices and detecting if any vertices are less than or equal to 0
	for (int i = 0; i < vert.size(); i++)
	{
		vert[i].setCoord(vec3(m * vec4(vert[i].getCoord(), 1)));

		if (vert[i].getCoord().y <= 0)
		{
			vert.push_back(vert[i]);
			colResponce = true;
		}
	}

	//if collision repsonce is true then take an avergage of the points colliding 
	
	if (colResponce == true)
	{
		if (vert.size() > 1)
		{
			for (int i = 0; i < vert.size(); i++)
			{
				average += vert[i].getCoord();
			}
			average /= vert.size();
		}
		else
		{
			average = vert[0].getCoord();
		}

		if (average.y < 0)
		{
			rb.setPos(rb.getPos() + vec3(0, -average.y, 0));
		}



		//position collision equals the average take away the position
		vec3 rcol = average - rb.getPos();

		//speed collition equals the velocity + the cross product of the angular and position collision
		vec3 vcol = rb.getVel() + cross(rb.getAngVel(), rcol);

		//normal vec3
		vec3 n = vec3(0, 1, 0);

		//jr equation
		float jr = dot(-(1 + e) * vcol, n) / (1 / rb.getMass()) + dot(n, (cross(rb.getInvInertia() * cross(rcol, n), rb.getPos())));

		vec3 vaftercol = rb.getVel() + jr / rb.getMass() * n;
		vec3 waftercol = rb.getAngVel() - jr * rb.getInvInertia() * cross(r, n);

		rb.setVel(vaftercol);
		rb.setAngVel(waftercol);

		colResponce = false;
	}


}


