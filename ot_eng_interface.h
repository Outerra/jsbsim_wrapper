#pragma once
#ifndef __OT_ENG_INTERFACE_H__
#define __OT_ENG_INTERFACE_H__

#include <comm/str.h>
#include <glm/glm.hpp>

namespace ot {

class eng_interface
{
public:

    //@param text text to log, optionally prefixed with error: warning: info: dbg: etc
    virtual void write_log(const char* text) = 0;

    virtual double get_earth_radius() const = 0;

    virtual float elevation_over_terrain(
        const glm::dvec3 &pos,
        float maxdist,
        glm::vec3 *normal,
        glm::dvec3 *surface,
        glm::vec3 *v,
        glm::vec3 *w,
        glm::dvec3 *ground_pos,
        double* ground_mass_inv,
        glm::dmat3x3* ground_j_inv) const = 0;
};

} // end of namspace ot

#endif // __OT_ENG_INTERFACE_H__
