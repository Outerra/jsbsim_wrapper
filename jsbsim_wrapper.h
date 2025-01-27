#pragma once
#ifndef __JSBSIM_WRAPPER_H__
#define __JSBSIM_WRAPPER_H__

#include <comm/token.h>
#include <comm/dynarray.h>
#include <ot/glm/glm_types.h>

namespace ot {
    class eng_interface;
    struct aircraft_data;
}

class jsbsim_wrapper
{
public:

    struct property
    {
        property(void* handle) : handle(handle) {}

        virtual coid::token name() const;

        virtual double get_double_value() const;
        virtual bool get_bool_value() const;

        virtual void set_value(double value);
        virtual void set_value(bool value);

        virtual property add_child_property(const coid::token& name, int index = 0);

        virtual void get_children_properties(coid::dynarray<property>& list) const;

    protected:

        void* handle = 0;
    };


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

    virtual property root() = 0;

    virtual void set_gear(const bool down) = 0;

    virtual void enable_log(bool enable) = 0;

    virtual uint get_steer_type(uint wheel_id) = 0;

    virtual uint get_num_contact_points(bool gearsonly) = 0;

    virtual float3 get_contact_point_pos(const uint idx, bool gearsonly) = 0;

    virtual float3 get_wheel_axis_vel(uint wheel_id) = 0;
};

#endif // __JSBSIM_WRAPPER_H__
