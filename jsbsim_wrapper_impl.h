#pragma once
#ifndef __JSBSIM_WRAPPER_IMPL_H__
#define __JSBSIM_WRAPPER_IMPL_H__

#include "jsbsim_wrapper.h"
#include "input_output/FGGroundCallback.h"

#include <ot/glm/glm_types.h>
#include <ot/sys/aircraft_data.h>

#include <memory>

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
}

class SGPropertyNode;


class jsbsim_wrapper_impl
    : public jsbsim_wrapper
{
private:

    std::shared_ptr<JSBSim::FGFDMExec> _jsbexec;

    JSBSim::FGInitialCondition* _jsbic = 0;
    JSBSim::FGInertial* _inertial = 0;
    JSBSim::FGAtmosphere* _atmosphere = 0;
    JSBSim::FGFCS* _FCS = 0;
    JSBSim::FGPropulsion* _propulsion = 0;
    JSBSim::FGMassBalance* _massBalance = 0;
    JSBSim::FGAircraft* _aircraft = 0;
    JSBSim::FGPropagate* _propagate = 0;
    JSBSim::FGPropertyNode* _props = 0;
    JSBSim::FGAuxiliary* _auxiliary = 0;
    JSBSim::FGAerodynamics* _aerodynamics = 0;
    JSBSim::FGGroundReactions* _groundReactions = 0;

    /// /////////////////////

    ot::eng_interface* _eng_ifc = 0;
    ot::aircraft_data _aircraft_data;

    double _time_rest;
    double _earth_radius;

    double3 _next_pos;
    quat _next_rot;
    double3 _prev_pos;
    quat _prev_rot;
    double3 _prev_vel;
    double3 _prev_pqr;

    struct prop_stack_entry {
        SGPropertyNode* node = 0;
        uint child_index = 0;
        uint child_count = 0;
    };

    coid::dynarray32<prop_stack_entry> _property_stack;


protected:

    static double M2F() { return 3.2808399; }
    static double F2M() { return 1.0 / M2F(); }

public:
    COIDNEWDELETE(jsbsim_wrapper_impl);

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

    void set_gear_brakes(const float3& brakes) override;

    void set_flaps(const float angle) override;

    void set_engine_throttle(const int engine, const float throttle) override;

    void set_engine_mixture(const int engine, const float mixture) override;

    void set_elevator_trim(const float trim) override;

    void set_aileron_trim(const float trim) override;

    void set_rudder_trim(const float trim) override;

    void pause() override;

    void resume() override;

    void update(const float dt) override;

    void set_afcs(const int level) override;

    bool is_running() const override;

    double get_property(const char* name) override;
    void set_property(const char* name, double value) override;


    /// @brief reset current property to root
    void root();

    /// @brief move current property to the first child of current property
    /// @return false if there's no child
    bool prop_first_child();

    /// @brief move current property to the next sibling of current property
    /// @return false if there's no next sibling
    bool prop_next_sibling();

    /// @brief move current property to the parent of current property
    /// @return false if this was the root
    bool prop_parent();

    /// @return current property name 
    coid::token prop_name() const;
    jsbsim_prop_type prop_type() const;

    double prop_get_double_value() const;
    bool prop_get_bool_value() const;

    bool prop_set_double_value(double value);
    bool prop_set_bool_value(bool value);

    bool prop_add_child(const coid::token& name, int min_index);

    void set_gear(const bool down) override;

    void enable_log(bool enable) override;

    uint get_steer_type(uint wheel_id) override;

    uint get_num_contact_points(bool gearsonly) override;

    float3 get_contact_point_pos(const uint idx, bool gearsonly) override;

    float3 get_wheel_axis_vel(uint wheel_id) override;
};

#endif // __JSBSIM_WRAPPER_IMPL_H__
