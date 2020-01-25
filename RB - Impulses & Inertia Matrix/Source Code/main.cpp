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

void integrate(Body& rb, double time, double deltaTime);
void forces(RigidBody& rb, double time, double deltaTime);
void ApplyImpulse(RigidBody& rb, vec3 impulse, vec3 point);

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
	plane.scale(glm::vec3(10.0f, 10.0f, 10.0f));
	Shader lambert = Shader("resources/shaders/physics.vert", "resources/shaders/physics.frag");
	plane.setShader(lambert);

	RigidBody rb = RigidBody();
	Mesh m = Mesh::Mesh(Mesh::CUBE);
	vec3 scale = vec3(1, 3, 1);
	m.scale(scale);
	rb.setMesh(m);

	Shader rbShader = Shader("resources/shaders/physics.vert", "resources/shaders/physics.frag");
	rb.getMesh().setShader(rbShader);

	rb.setMass(2.0f);
	rb.translate(vec3(0.0f, 3.0f, 0.0f));
	rb.setVel(vec3(2.0f, 0.0f, 0.0f));
	rb.setAngVel(vec3(0.0f, 0.0f, 0.0f));

//	rb.addForce(&g);

	mat3 inertiamat = mat3(((rb.getMass() * (pow(2, 2) + pow(6, 2))) / 12), 0, 0,
		0, (rb.getMass() * (pow(2, 2) + pow(2, 2)) / 12), 0,
		0, 0, (rb.getMass() * (pow(6, 2) + pow(2, 2)) / 12));

	rb.setInvInertia(inertiamat);

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

		mat3 prevrot = rb.getRotate();
	

		while (accumulator >= deltaTime)
		{

			//cout << to_string((rb.GetRotate() * deltaTime) * rb.GetInvInertia() * (transpose(rb.GetRotate()) * deltaTime)) << endl;
			//cout << to_string(inverse((rb.GetRotate() * deltaTime) * rb.GetInvInertia() * transpose(rb.GetRotate() * deltaTime))) << endl;
			//cout << to_string(rb.GetInvInertia()) << endl;



			if (prevrot != rb.getRotate())
			{
				rb.setInvInertia((rb.getRotate() * deltaTime) * inverse(rb.getInvInertia()) * transpose(rb.getRotate() * deltaTime));
			}
			if (currentTime >= 2.0f && !impulseApplied)
			{
				vec3 impulse = vec3(-0.1f, 0, 0);
				vec3 CoM = rb.getPos();
				vec3 impulsePoint = CoM + vec3(1, -0.5f, 0);

				//cout << to_string(rb.GetInvInertia()) << endl;

				ApplyImpulse(rb, impulse, impulsePoint);

				impulseApplied = true;
			}

			std::vector<Vertex> v = rb.getMesh().getVertices();
			vec3 average;

			for (int i = 0; i < v.size(); i++)
			{
				v[i].setCoord(vec3(m.getModel() * rb.getMesh().getScale() * vec4(v[i].getCoord(), 1)));

				if (v[i].getCoord().y <= 0)
				{
					
					average += v[i].getCoord();
			


				}
			}

			std::cout << to_string(average);

			integrate(rb, time, deltaTime);

			rb.setAngVel(rb.getAngVel() + deltaTime * rb.getAngAcc());
			mat3 angVelSkew = matrixCross3(rb.getAngVel());
			mat3 R = mat3(rb.getRotate());

			R += deltaTime * angVelSkew * R;
			R = orthonormalize(R);
			rb.setRotate(mat4(R));


			accumulator -= deltaTime;
			time += deltaTime;
		}


		// Manage interaction
		app.doMovement(deltaTime * 100);

		// clear buffer
		app.clear();

		//Draw Particles
		app.draw(plane);

		app.draw(rb.getMesh());

		app.display();
	}

	app.terminate();

	return EXIT_SUCCESS;
}

void ApplyImpulse(RigidBody& rb, vec3 impulse, vec3 point)
{
	rb.setVel(impulse / rb.getMass());
	vec3 r = point - rb.getPos();
	rb.setAngVel(rb.getInvInertia() * (cross(r, impulse)));
}

void integrate(Body& rb, double time, double deltaTime)
{
	vec3 a = rb.applyForces(rb.getPos(), rb.getVel(), time, deltaTime) / rb.getMass();
	rb.setAcc(a);

	vec3 v = rb.getVel() + (rb.getAcc() * deltaTime);

	vec3 r = rb.getPos() + (rb.getVel() * deltaTime);

	rb.setVel(v);
	rb.setPos(r);



}

void forces(RigidBody& rb, double time, double deltaTime)
{
	//Calculate Forces
	vec3 f = rb.applyForces(rb.getPos(), rb.getVel(), time, deltaTime);
	vec3 torque = rb.getPos() * f;

	//Impulse
	float e = 0.9f;
	vec3 j = vec3();
	vec3 d = vec3();

	vec3 a = (f + torque) / rb.getMass();

	vec3 v = rb.getVel() + a + j;

	vec3 r = rb.getPos() + v * deltaTime;

	vec3 L = (rb.getPos() * (rb.getMass() * rb.getVel())) + torque * deltaTime;

	mat3 w;

	mat3 R = rb.getRotate() + w * rb.getRotate();
	R = orthonormalize(R);

	rb.setVel(v);
	rb.setPos(r);
	rb.translate(r - rb.getPos());
	rb.setRotate(R);
}
