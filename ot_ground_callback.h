#pragma once
#ifndef __JSBSIM_WRAPPER_OT_GROUND_CALLBACK_H__
#define __JSBSIM_WRAPPER_OT_GROUND_CALLBACK_H__

#include "input_output\FGGroundCallback.h"

namespace ot {

class eng_interface;

class ground_callback
    : public JSBSim::FGGroundCallback
{
protected:

    double _earth_radius;
    double _water_radius;
    eng_interface* _eng;

public:

    ground_callback(eng_interface* eng);
    virtual ~ground_callback();

    // IMPLEMENTS JSBSim::FGGroundCallback

    double GetAGLevel(
        double t,
        const JSBSim::FGLocation& l,
        JSBSim::FGLocation& cont,
        JSBSim::FGColumnVector3& n,
        JSBSim::FGColumnVector3& v,
        JSBSim::FGColumnVector3& w) const override;

    double GetContact(
        double t,
        double maxdist,
        const JSBSim::FGLocation& l,
        JSBSim::FGLocation& cont,
        JSBSim::FGColumnVector3& n,
        JSBSim::FGColumnVector3& v,
        JSBSim::FGColumnVector3& w,
        JSBSim::FGColumnVector3& ground_position,
        double& ground_mass_inverse,
        JSBSim::FGMatrix33& ground_j_inverse) const override;
};

} // end of namespace ot

#endif // __JSBSIM_WRAPPER_OT_GROUND_CALLBACK_H__
