#pragma once 
#include <glm/glm.hpp> 
#include <iostream>

class Body;

class Force
{
public: Force() {}
	  ~Force() {}
	  virtual glm::vec3 apply(float mass, const glm::vec3& pos, const glm::vec3& vel);

};

class Gravity : public Force {
public: // constructors
	Gravity() {} 
	Gravity(const glm::vec3 &gravity) { m_gravity = gravity; }
// get and set methods 
	glm::vec3 getGravity()
	const { return m_gravity; } 
	
	void setGravity(glm::vec3 gravity) { m_gravity = gravity; }
// physics 
	glm::vec3 apply(float mass, const glm::vec3 &pos, const glm::vec3 &vel);
private: glm::vec3 m_gravity = glm::vec3(0.0f, -9.8f, 0.0f);
};
/* ** DRAG CLASS */ class Drag : public Force {

public: Drag() {}
	  // physics 
	  glm::vec3 apply(float mass, const glm::vec3& pos, const glm::vec3& vel);
private:
};

class Hooke : public Force {
public: 
	Hooke() {} 
	Hooke(Body* b1, Body* b2, float ks, float kd, float rest)
	{ 
		m_ks = ks; m_kd = kd; m_rest = rest; m_b1 = b1; m_b2 = b2;
	}

	float getStiff() 
	const {
		return m_ks;

	}

	void setStiff(float ks) 
	{
		m_ks = ks;
	}


	float getDamp()
		const {
		return m_kd;

	}

	void setDamp(float kd) 
	{ 
		m_kd = kd; 
	}

	float getRest()
		const {
		return m_rest;

	}

	void setRest(float rest) 
	{ 
		m_rest = rest; 
	}

	Body* getBody1()
		const {
		return m_b1;

	}

	void setBody1(Body* b1)
	{ 
		m_b1 = b1; 
	}

	Body* getBody2()
		const {
		return m_b2;

	}

	void setBody2(Body* b2) { m_b2 = b2; }



	glm::vec3 apply(float mass, const glm::vec3& pos, const glm::vec3& vel);
private: 
	float m_ks; // spring stiffness 
	float m_kd; // damping coefficient
	float m_rest; // spring rest length
    Body* m_b1; // pointer to the body connected to one extremity of the spring 
	Body* m_b2; // pointer to the body connected to the other extremity
};