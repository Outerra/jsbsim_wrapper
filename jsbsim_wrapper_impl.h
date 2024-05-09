#pragma once
#ifndef __JSBSIM_WRAPPER_IMPL_H__
#define __JSBSIM_WRAPPER_IMPL_H__

#include "jsbsim_wrapper.h"
#include "input_output/FGGroundCallback.h"

#include <ot/glm/glm_types.h>

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

    class jsbsim_wrapper_impl
        : public jsbsim_wrapper
    {
    private:

        std::shared_ptr<FGFDMExec> _jsbexec;

        FGInitialCondition* _jsbic = 0;
        FGInertial* _inertial = 0;
        FGAtmosphere* _atmosphere = 0;
        FGFCS* _FCS = 0;
        FGPropulsion* _propulsion = 0;
        FGMassBalance* _massBalance = 0;
        FGAircraft* _aircraft = 0;
        FGPropagate* _propagate = 0;
        FGAuxiliary* _auxiliary = 0;
        FGAerodynamics* _aerodynamics = 0;
        FGGroundReactions* _groundReactions = 0;

        /// /////////////////////

        ot::eng_interface* _eng_ifc = 0;
        ot::aircraft_data _aircraft_data;
        ot::aircraft_data::GndReactions GndReactdata;
        ot::aircraft_data::FCS FCSdata;
        ot::aircraft_data::Propulsion Propdata;
        ot::aircraft_data::Propagate Propagdata;
        ot::aircraft_data::Atmosphere Atmosdata;
        ot::aircraft_data::Accelerations Acceldata;
        ot::aircraft_data::MassBalance Massbaldata;
        ot::aircraft_data::Aerodynamics Aerodyndata;
        ot::aircraft_data::Aircraft Aircraftdata;
        ot::aircraft_data::Auxiliary Auxiliarydata;

        double _time_rest;
        double _earth_radius;

        FGPropertyNode* _props = 0;



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

        jsbsim_wrapper_impl(ot::eng_interface* eng, uint object_id);
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

        void set_gear(const bool down) override;
        
        void enable_log(bool enable) override;

        uint get_steer_type(uint wheel_id) override;

        uint get_num_contact_points(bool gearsonly) override;

        float3 get_contact_point_pos(const uint idx, bool gearsonly) override;

        float3 get_wheel_axis_vel(uint wheel_id) override;
    };

} // end of namespace JSBSim

#endif // __JSBSIM_WRAPPER_IMPL_H__
