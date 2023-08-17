#pragma once
#ifndef __JSBSIM_WRAPPER_IMPL_H__
#define __JSBSIM_WRAPPER_IMPL_H__

#include "jsbsim_wrapper.h"
#include "input_output/FGGroundCallback.h"

#include <ot/glm/glm_types.h>

class eng_interface;

namespace ot {
class jsbsim_root;
}

namespace JSBSim {

class FGFDMExec;
class FGState;
class FGAtmosphere;
class FGFCS;
class FGPropulsion;
class FGMassBalance;
class FGAerodynamics;
class FGInertial;
class FGAircraft;
class FGPropagate;
class FGAuxiliary;
class FGOutput;
class FGInitialCondition;
class FGGroundReactions;
class FGPropertyNode;

class jsbsim_wrapper_impl
    : public jsbsim_wrapper
{
protected:

    iref<ot::jsbsim_root> _jsbroot;
    ref<FGFDMExec> _jsbexec;
    FGInitialCondition* _jsbic;
    FGAtmosphere* _atmosphere;
    FGFCS* _FCS;
    FGPropulsion* _propulsion;
    FGMassBalance* _massBalance;
    FGAircraft* _aircraft;
    FGPropagate* _propagate;
    FGAuxiliary* _auxiliary;
    FGAerodynamics* _aerodynamics;
    FGGroundReactions* _groundReactions;
    FGInertial* _inertial;
    ot::aircraft_data _aircraft_data;
    double _time_rest;
    double _earth_radius;

    FGPropertyNode* _props;

    //double3 _accum_vel;
    //double3 _accum_avel;
    double3 _next_pos;
    quat _next_rot;
    double3 _prev_pos;
    quat _prev_rot;
    double3 _prev_vel;
    double3 _prev_pqr;

protected:

    static double M2F() { return 3.2808399; }
    static double F2M() { return 1.0 / M2F(); }

public:

    jsbsim_wrapper_impl(ot::eng_interface* eng);
    virtual ~jsbsim_wrapper_impl();

    void update_aircraft_data();

    // IMPLEMENTS JSBSim::jsbsim_wrapper

    bool load_aircraft(
        const coid::token& root_dir,
        const coid::token& aircrafts_dir,
        const coid::token& engines_dir,
        const coid::token& systems_dir,
        const coid::token& model) override;

    void set_initial_condition(
        const glm::dvec2& lat_lon,
        const float altitude,
        const glm::quat& rot,
        const float speed_kts,
        const float engines_thrust,
        const bool trim) override;

    void set_initial_condition(
        const glm::dvec2& lat_lon,
        const float altitude,
        const float3& hpr,
        const float speed_kts,
        const float engines_thrust,
        const bool trim) override;

    void set_initial_condition(
        const glm::dvec2& lat_lon,
        const float altitude,
        const float3& hpr,
        const float3& vel,
        const float engines_thrust,
        const bool trim) override;

    void reset_ic() override;
    void initialize_ic() override;

    const ot::aircraft_data* get_aircraft_data() override { return &_aircraft_data; }

    void set_aircraft_data(const ot::aircraft_data* ad) override { _aircraft_data = *ad; }

    void set_controls(const glm::vec4& controls) override;

    void set_engine(const bool on_off) override;

    void set_gear_brakes(const glm::vec3& brakes) override;

    void set_flaps(const float angle) override;

    void set_engine_throttle(const int engine, const float throttle) override;

    void set_engine_mixture(const int engine, const float mixture) override;

    void set_elevator_trim(const float trim) override;

    void set_aileron_trim(const float trim) override;

    void set_rudder_trim(const float trim) override;

    void pause() override;

    void resume() override;

    void update(const float dt) override;

    // 0: off, 1: sould be default, 2: full hover mode
    void set_afcs(const int level) override;

    bool is_running() const override;

    double get_property(const char* name) override;
    void set_property(const char* name, double value) override;

    void set_gear(const bool down) override;

    uint get_num_gear_contact_point() override;
    float3 get_gear_contact_point(const uint idx) override;

    void enable_log(bool enable) override;
};

} // end of namespace JSBSim

#endif // __JSBSIM_WRAPPER_IMPL_H__
