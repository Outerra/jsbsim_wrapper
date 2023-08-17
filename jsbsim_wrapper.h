#pragma once
#ifndef __JSBSIM_WRAPPER_H__
#define __JSBSIM_WRAPPER_H__

#include "jsbsim_wrapper_api.h"
#include <comm/ref.h>
#include <ot/sys/aircraft_data.h>

namespace ot {
class eng_interface;
}

namespace JSBSim {

class jsbsim_wrapper;

typedef void (*jsbsim_create_wrapper_pfn)(iref<jsbsim_wrapper>&, ot::eng_interface* eng);

class jsbsim_wrapper
    : public policy_intrusive_base
{
public:

    virtual bool load_aircraft(
        const coid::token& root_dir,
        const coid::token& aircraft_dir,
        const coid::token& engine_dir,
        const coid::token& systems_dir,
        const coid::token& fdm_file) = 0;

    virtual void set_initial_condition(
        const glm::dvec2& lat_lon,
        const float altitude,
        const glm::quat& rot,
        const float speed_kts,
        const float engines_thrust,
        const bool trim) = 0;

    virtual void set_initial_condition(
        const glm::dvec2& lat_lon,
        const float altitude,
        const float3& hpr,
        const float speed_kts,
        const float engines_thrust,
        const bool trim) = 0;

    virtual void set_initial_condition(
        const glm::dvec2& lat_lon,
        const float altitude,
        const float3& hpr,
        const float3& vel,
        const float engines_thrust,
        const bool trim) = 0;

    virtual void reset_ic() = 0;
    virtual void initialize_ic() = 0;

    virtual const ot::aircraft_data* get_aircraft_data() = 0;

    virtual void set_aircraft_data(const ot::aircraft_data* ad) = 0;

    virtual void set_controls(const glm::vec4& controls) = 0;

    virtual void set_engine(const bool on_off) = 0;

    virtual void set_gear_brakes(const glm::vec3& brakes) = 0;

    virtual void set_flaps(const float angle) = 0;

    virtual void set_engine_throttle(const int engine, const float throttle) = 0;

    virtual void set_engine_mixture(const int engine, const float mixture) = 0;

    virtual void set_elevator_trim(const float trim) = 0;

    virtual void set_aileron_trim(const float trim) = 0;

    virtual void set_rudder_trim(const float trim) = 0;

    virtual void pause() = 0;

    virtual void resume() = 0;

    virtual void update(const float dt) = 0;

    virtual void set_afcs(const int enable) = 0;

    virtual bool is_running() const = 0;

    virtual double get_property(const char* name) = 0;
    virtual void set_property(const char* name, double value) = 0;

    virtual void set_gear(const bool down) = 0;

    virtual uint get_num_gear_contact_point() = 0;
    virtual float3 get_gear_contact_point(const uint idx) = 0;

    virtual void enable_log(bool enable) = 0;
};

} // end of namespace

#endif // __JSBSIM_WRAPPER_H__
