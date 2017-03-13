#pragma once
#ifndef __JSBSIM_WRAPPER_ENG_INTERFACE_H__
#define __JSBSIM_WRAPPER_ENG_INTERFACE_H__

#include <comm/str.h>
#include <glm/glm.hpp>

class eng_interface
{
public:

	enum ELogTypes {
		LogInfo,
		LogWarning,
		LogDebug,
		LogError,
	};

	virtual void write_log(const coid::charstr &text)=0;

	virtual bool elevation_over_terrain(
		const glm::dvec3 &pos,
		float *dist,
		glm::vec3 *normal,
		glm::dvec3 *surface)=0;
};

#endif // __JSBSIM_WRAPPER_ENG_INTERFACE_H__
