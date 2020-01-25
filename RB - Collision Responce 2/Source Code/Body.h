
#pragma once
#include "Mesh.h"
#pragma once
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>
#include <vector>
#include "Force.h"
//#include "RigidBody.h"

class Body
{

private:
	float m_cor;
	float m_mass;
	Mesh m_mesh;
	glm::vec3 m_acc;
	glm::vec3 m_pos;
	glm::vec3 m_vel;
	glm::mat4 m_rotate;
	std::vector<Force*> m_forces;

public:
	Body();

	Mesh& getMesh() { return m_mesh; }

	glm::vec3& getAcc() { return m_acc; }
	glm::vec3& getVel() { return m_vel; }
	glm::vec3& getPos() { return m_pos; }

	glm::mat3 getTranslate() const { return m_mesh.getTranslate(); }
	glm::mat3 getRotate() const { return m_rotate; }
	glm::mat3 getScale() const { return m_mesh.getScale(); }

	

	float getMass() const { return m_mass; }
	float getCor() { return m_cor; }


	void setMesh(Mesh m) { m_mesh = m; }

	void setAcc(const glm::vec3 &vect) { m_acc = vect; }
	void setVel(const glm::vec3 &vect) { m_vel = vect; }
	void setVel(int i, float v) { m_vel[i] = v; }

	void setPos(const glm::vec3 &vect) { m_pos = vect; m_mesh.setPos(vect); } 
	void setPos(int i, float p) { m_pos[i] = p; m_mesh.setPos(i, p); }

	void setCor(float cor) { m_cor = cor; }
	void setMass(float mass) { m_mass = mass; }

	void setRotate(const glm::mat4& mat) { m_rotate = mat; m_mesh.setRotate(mat); }

	void translate(const glm::vec3 &vect);
	void rotate(float angle, const glm::vec3 &vect); 
	void scale(const glm::vec3 &vect);

	std::vector<Force*> getForces() { return m_forces; } 
	void addForce(Force* f) { m_forces.push_back(f); }

	// sum of all forces applied to a body and return acceleration 
	glm::vec3 applyForces(glm::vec3 x, glm::vec3 v, float t, float dt) 
	{
		glm::vec3 fAccumulator = glm::vec3(0.0f);
		for (auto& f : m_forces) { fAccumulator += f->apply(getMass(), x, v); } return fAccumulator / getMass();
	}



	~Body();
};

