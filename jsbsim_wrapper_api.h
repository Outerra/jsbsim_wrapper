#pragma once
#ifndef __JSBSIM_WRAPPER_API_H__
#define __JSBSIM_WRAPPER_API_H__

#ifdef JSBSIM_WRAPPER_EXPORTS
	#define JSBSIM_WRAPPER_API __declspec(dllexport)
#else
	#define JSBSIM_WRAPPER_API __declspec(dllimport)
#endif

#endif // __JSBSIM_WRAPPER_API_H__