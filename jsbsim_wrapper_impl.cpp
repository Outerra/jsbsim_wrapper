#include <ot/glm/glm_ext.h>

#include "jsbsim_wrapper_impl.h"

#include "ot_jsbsim_root.h"
#include "ot_ground_callback.h"

#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <initialization/FGInitialCondition.h>
#include <initialization/FGTrim.h>
#include <models/FGModel.h>
#include <models/FGAircraft.h>
#include <models/FGFCS.h>
#include <models/FGPropagate.h>
#include <models/FGAuxiliary.h>
#include <models/FGInertial.h>
#include <models/FGAtmosphere.h>
#include <models/FGMassBalance.h>
#include <models/FGAerodynamics.h>
#include <models/FGAccelerations.h>
#include <models/FGLGear.h>
#include <models/FGPropulsion.h>
#include <models/FGGroundReactions.h>
#include <models/propulsion/FGEngine.h>
#include <models/propulsion/FGPiston.h>
#include <models/propulsion/FGTurbine.h>
#include <models/propulsion/FGTurboProp.h>
#include <models/propulsion/FGRocket.h>
#include <models/propulsion/FGElectric.h>
#include <models/propulsion/FGNozzle.h>
#include <models/propulsion/FGPropeller.h>
#include <models/propulsion/FGTank.h>
#include <input_output/FGPropertyManager.h>
#include <input_output/FGGroundCallback.h>

#include <string>

#include "ot_eng_interface.h"


using namespace JSBSim;

//const float integration_step = 0.01f;

const int64 STEPns = 15625000 / 2;
const double STEPs = STEPns * 1e-9;
//const double STEPs = 1.0 / 200.0;


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

jsbsim_wrapper_impl::jsbsim_wrapper_impl(ot::eng_interface* eng)
    : _jsbroot(ot::jsbsim_root::_inst == 0
        ? new ot::jsbsim_root(eng)
        : ot::jsbsim_root::_inst)
    , _jsbexec(new FGFDMExec(
        _jsbroot->get_gc()
        /*ot::jsbsim_root::get().get_pm(),
        ot::jsbsim_root::get().get_counter()*/))
    , _jsbic(_jsbexec->GetIC())
    , _atmosphere(_jsbexec->GetAtmosphere())
    , _FCS(_jsbexec->GetFCS())
    , _propulsion(_jsbexec->GetPropulsion())
    , _massBalance(_jsbexec->GetMassBalance())
    , _aircraft(_jsbexec->GetAircraft())
    , _propagate(_jsbexec->GetPropagate())
    , _auxiliary(_jsbexec->GetAuxiliary())
    , _aerodynamics(_jsbexec->GetAerodynamics())
    , _groundReactions(_jsbexec->GetGroundReactions())
    , _inertial(_jsbexec->GetInertial())
    , _time_rest(0.0f)
    , _earth_radius(_jsbroot->get_earth_radius())
    , _props(_jsbexec->GetPropertyManager()->GetNode())
{
    DASSERT(_jsbexec.get() != 0);
    //_jsbexec->Setdt(integration_step);
    //_jsbexec->SetGroundCallback(ot::jsbsim_root::get().get_gc()); // set in the jsbsim_root!

    _jsbexec->Setdt(STEPs);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

jsbsim_wrapper_impl::~jsbsim_wrapper_impl()
{
    _jsbexec.release();
    _jsbroot.release();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::update_aircraft_data()
{
    _aircraft_data.airspeed_kts = float(_auxiliary->GetVcalibratedKTS());
    _aircraft_data.vertical_speed = float(_propagate->GetVel(3));

    _aircraft_data.heading = float(_propagate->GetEuler(3));
    _aircraft_data.pitch = float(_propagate->GetEuler(2));
    _aircraft_data.roll = float(_propagate->GetEuler(1));

    _aircraft_data.altitude_asl = float(_propagate->GetAltitudeASL() * F2M());

    _aircraft_data.gear_down = _FCS->GetGearCmd() != 0 ? true : false;

    if (_propulsion)
    {
        unsigned int neng = _propulsion->GetNumEngines();
        if (neng >= ot::aircraft_data::MAX_ENGINES)
            neng = ot::aircraft_data::MAX_ENGINES;

        uint running_engines = 0;

        for (unsigned int i = 0; i < neng; ++i)
        {
            FGEngine* eng = _propulsion->GetEngine(i);
            ot::aircraft_data::engine& data = _aircraft_data.engines[i];

            data.rpm = float(eng->GetThruster()->GetRPM());
            data.throttle = float(_FCS->GetThrottlePos()[0]);
            data.mixture = float(_FCS->GetMixturePos()[0]);
            data.state = eng->GetCranking() ? ot::aircraft_data::engine_state::cranking :
                eng->GetRunning() ? ot::aircraft_data::engine_state::running :
                ot::aircraft_data::engine_state::stopped;

            if (data.state > ot::aircraft_data::engine_state::stopped)
                running_engines++;

            switch (eng->GetType()) {
            case FGEngine::etElectric:
                break;
            case FGEngine::etTurbine:
                data.rpm = static_cast<FGTurbine*>(eng)->GetCutoff() == false ? 1.0f : 0.0f;
                data.egt = float(static_cast<FGTurbine*>(eng)->GetEGT());
                data.ffl = float(static_cast<FGTurbine*>(eng)->getFuelFlow_gph());
                data.mixture = float(_FCS->GetMixturePos()[0]);
                data.n1 = float(static_cast<FGTurbine*>(eng)->GetN1());
                data.n2 = float(static_cast<FGTurbine*>(eng)->GetN2());
                break;
            case FGEngine::etPiston:
                data.rpm = float(static_cast<FGPiston*>(eng)->getRPM());
                data.egt = float(static_cast<FGPiston*>(eng)->GetEGT());
                data.ffl = float(static_cast<FGPiston*>(eng)->getFuelFlow_gph());
                break;
            }
        }

        _aircraft_data.running_engines = running_engines;
    }
    //else
    //    _aircraft_data.engine_running = false;

    _aircraft_data.gear_brakes.x = float(_FCS->GetLBrake());
    _aircraft_data.gear_brakes.y = float(_FCS->GetRBrake());

    _aircraft_data.elevator_angle = float(_FCS->GetDeCmd());
    _aircraft_data.flaps_angle = float(_FCS->GetDfCmd());
    _aircraft_data.aileron_left_angle = float(_FCS->GetDaCmd());
    _aircraft_data.aileron_right_angle = float(-_FCS->GetDaCmd());
    _aircraft_data.rudder_angle = float(_FCS->GetDrCmd());

    _aircraft_data.elevator_trim_angle = float(_FCS->GetPitchTrimCmd());
    _aircraft_data.aileron_trim_angle = float(_FCS->GetRollTrimCmd());
    _aircraft_data.rudder_trim_angle = float(_FCS->GetYawTrimCmd());

    const FGLocation& loc_cg = _propagate->GetLocation();
    FGLocation loc = loc_cg.LocalToLocation(_propagate->GetTb2l() * _auxiliary->in.VRPBody);
    //const FGLocation &loc = _auxiliary->GetLocationVRP();
    _aircraft_data.pos_ecef = double3(
        loc.Entry(1) * F2M(),
        loc.Entry(2) * F2M(),
        loc.Entry(3) * F2M());

    _aircraft_data.pos_cg_offset = float3(
        (loc_cg(1) - loc.Entry(1)) * F2M(),
        (loc_cg(2) - loc.Entry(2)) * F2M(),
        (loc_cg(3) - loc.Entry(3)) * F2M());

    _aircraft_data.pos_latlon = double2(
        loc.GetLatitudeDeg(),
        loc.GetLongitudeDeg());
    _aircraft_data.pos_latlon_cg = double2(
        loc_cg.GetLatitudeDeg(),
        loc_cg.GetLongitudeDeg());

    const FGMatrix33& m = _propagate->GetTb2ec();
    /*_aircraft_data.drot_body_ecef =
        glm::quat_cast(double3x3(
            m(1, 1), m(2, 1), m(3, 1),
            m(1, 2), m(2, 2), m(3, 2),
            m(1, 3), m(2, 3), m(3, 3)))
        * dquat(0.0, 0.70710677, 0.70710677, 0.0);

    _aircraft_data.rot_body_ecef = quat(
        float(_aircraft_data.drot_body_ecef.w),
        float(_aircraft_data.drot_body_ecef.x),
        float(_aircraft_data.drot_body_ecef.y),
        float(_aircraft_data.drot_body_ecef.z));*/

    _aircraft_data.rot_body_ecef =
        glm::quat_cast(float3x3(
            m(1, 1), m(2, 1), m(3, 1),
            m(1, 2), m(2, 2), m(3, 2),
            m(1, 3), m(2, 3), m(3, 3)))
        * quat(0.0, 0.70710677, 0.70710677, 0.0);

    const FGColumnVector3 vel = _propagate->GetECEFVelocity();
    _aircraft_data.vel_ecef = float3(vel(1) * F2M(), vel(2) * F2M(), vel(3) * F2M());

    const FGColumnVector3 avele = _propagate->GetPQR();
    _aircraft_data.avel_ecef = float3(avele(1), avele(2), avele(3));

    const FGColumnVector3 velb = _propagate->GetUVW();
    _aircraft_data.vel_body = float3(velb(1) * F2M(), velb(2) * F2M(), velb(3) * F2M());

    const FGColumnVector3 acc = _propagate->GetTb2ec() * _jsbexec->GetAccelerations()->GetBodyAccel();
    _aircraft_data.acc_body = float3(acc(1) * F2M(), acc(2) * F2M(), acc(3) * F2M());

    const FGColumnVector3 avel = _propagate->GetTb2ec() * _propagate->GetPQR();
    _aircraft_data.avel_body = float3(avel(1), avel(2), avel(3));

    const FGColumnVector3 aacc = _propagate->GetTb2ec() * _jsbexec->GetAccelerations()->GetPQRdot();
    _aircraft_data.aacc_body = float3(aacc(1), aacc(2), aacc(3));

    float3 normal;
    float3 cvel;
    float3 rvel;
    double3 surface;
    double3 ground_pos;
    double ground_mass_inv;
    glm::dmat3x3 ground_j_inv;

    float maxdist = float(glm::length(_aircraft_data.pos_ecef));
    float dist = _jsbroot->get_eng()->elevation_over_terrain(
        _aircraft_data.pos_ecef,
        maxdist,
        &normal,
        &surface,
        &cvel,
        &rvel,
        &ground_pos,
        &ground_mass_inv,
        &ground_j_inv
    );

    if (dist < maxdist)
        _aircraft_data.altitude_agl = float(dist);
    else
        _aircraft_data.altitude_agl = -1.0f;



///////////////////////////////////
    

    uint num = get_num_contact_points(false);

    _aircraft_data.gears.realloc(num);
    
    for (uint i = 0; i < num; i++)
    {
        JSBSim::FGLGear* wheel = _jsbexec->GetGroundReactions()->GetGearUnit(i);
        ot::aircraft_data::gear& geardata = _aircraft_data.gears[i];
        geardata.steer_type = get_steer_type(i);
        geardata.contact_point_pos = get_contact_point_pos(i);
        geardata.axis_velocities = get_wheel_axis_vel(i);
        if (geardata.wheel_name.is_empty())
        {
            geardata.wheel_name = wheel->GetName().c_str();
        }
    }
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//
// IMPLEMENTS JSBSim::jsbsim_wrapper
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bool jsbsim_wrapper_impl::load_aircraft(
    const coid::token& root_dir,
    const coid::token& aircrafts_dir,
    const coid::token& engines_dir,
    const coid::token& systems_dir,
    const coid::token& model)
{
    const std::string root_dir_str(root_dir.ptr(), root_dir.len());
    const std::string aircrafts_dir_str(aircrafts_dir.ptr(), aircrafts_dir.len());
    const std::string engines_dir_str(engines_dir.ptr(), engines_dir.len());
    const std::string systems_dir_str(systems_dir.ptr(), systems_dir.len());
    const std::string model_str(model.ptr(), model.len());

    _jsbexec->SetRootDir(SGPath(root_dir_str));

    if (!_jsbexec->LoadModel(
        SGPath(aircrafts_dir_str),
        SGPath(engines_dir_str),
        SGPath(systems_dir_str),
        model_str)) {
        return false;
    }

    return true;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::enable_log(bool enable)
{
    if (enable)
        _jsbexec->EnableOutput();
    else
        _jsbexec->DisableOutput();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_initial_condition(
    const glm::dvec2& lat_lon,
    const float altitude,
    const glm::quat& rot,
    const float speed_kts,
    const float engines_thrust,
    const bool trim)
{
    float3x3 jrot = glm::mat3_cast(rot);

    const bool is_eng_running = _propulsion ? _propulsion->GetEngine(0)->GetRunning() : false;

    FGMatrix33 mat(
        jrot[0].x, jrot[0].y, jrot[0].z,
        jrot[1].x, jrot[1].y, jrot[1].z,
        jrot[2].x, jrot[2].y, jrot[2].z);
    FGColumnVector3 angles = mat.GetQuaternion().GetEuler();

    _jsbic->InitializeIC();
    _jsbic->SetPsiRadIC(angles(3));
    _jsbic->SetThetaRadIC(angles(2));
    _jsbic->SetPhiRadIC(angles(1));
    _jsbic->SetVcalibratedKtsIC(speed_kts);
    _jsbic->SetLatitudeRadIC(glm::radians(lat_lon.x));
    _jsbic->SetLongitudeRadIC(glm::radians(lat_lon.y));
    _jsbic->SetAltitudeASLFtIC(altitude * M2F());

    /*_jsbic->ResetIC(
        0,0,0,
        0,0,0,
        0,0,
        angles(1),angles(2),angles(3),
        glm::radians(lat_lon.x),
        glm::radians(lat_lon.y),
        altitude * M2F(),
        0);*/

    _jsbic->SetVcalibratedKtsIC(speed_kts);
    _jsbexec->ResetToInitialConditions(0);
    set_engine_throttle(-1, engines_thrust);
    _jsbexec->RunIC();
    set_engine(is_eng_running);

    if (false && trim && speed_kts >= 60.0f) {
        _FCS->SetMixturePos(0, 1.0);
        _FCS->SetThrottlePos(0, 0.80);
        for (int i = 0; i < 100; ++i)
            _propulsion->GetEngine(0)->Calculate();
        _jsbexec->DoTrim(tLongitudinal);
        _aircraft_data.trim_successful = _jsbexec->GetTrimStatus();
    }

    update_aircraft_data();

    _prev_pos = _aircraft_data.pos_ecef;
    _prev_rot = _aircraft_data.rot_body_ecef;

}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::reset_ic()
{
    _jsbic->InitializeIC();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::initialize_ic()
{
    _jsbexec->ResetToInitialConditions(0);
    _jsbexec->RunIC();

    update_aircraft_data();
    _prev_pos = _aircraft_data.pos_ecef;
    _prev_rot = _aircraft_data.rot_body_ecef;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_initial_condition(
    const glm::dvec2& lat_lon,
    const float altitude,
    const float3& hpr,
    const float speed_kts,
    const float engines_thrust,
    const bool trim)
{
    const bool is_eng_running = _propulsion ? _propulsion->GetEngine(0)->GetRunning() : 0;

    _jsbic->InitializeIC();
    _jsbic->SetPsiRadIC(hpr.x);
    _jsbic->SetThetaRadIC(hpr.y);
    _jsbic->SetPhiRadIC(hpr.z);
    _jsbic->SetVcalibratedKtsIC(speed_kts);
    _jsbic->SetLatitudeRadIC(glm::radians(lat_lon.x));
    _jsbic->SetLongitudeRadIC(glm::radians(lat_lon.y));
    _jsbic->SetAltitudeASLFtIC(altitude * M2F());

    /*_jsbic->ResetIC(
        0, 0, 0,
        0, 0, 0,
        0, 0,
        hpr.x, hpr.y, hpr.z,
        glm::radians(lat_lon.x),
        glm::radians(lat_lon.y),
        altitude * M2F(),
        0);*/

    _jsbexec->ResetToInitialConditions(0);
    set_engine_throttle(-1, engines_thrust);

    _jsbexec->RunIC();
    set_engine(is_eng_running);

    update_aircraft_data();

    _prev_pos = _aircraft_data.pos_ecef;
    _prev_rot = _aircraft_data.rot_body_ecef;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_initial_condition(
    const glm::dvec2& lat_lon,
    const float altitude,
    const float3& hpr,
    const float3& vel,
    const float engines_thrust,
    const bool trim)
{
    const bool is_eng_running = _propulsion ? _propulsion->GetEngine(0)->GetRunning() : 0;

    _jsbic->InitializeIC();
    _jsbic->SetPsiRadIC(hpr.x);
    _jsbic->SetThetaRadIC(hpr.y);
    _jsbic->SetPhiRadIC(hpr.z);

    //_jsbic->SetVcalibratedKtsIC(speed_kts);

    _jsbic->SetLatitudeRadIC(glm::radians(lat_lon.x));
    _jsbic->SetLongitudeRadIC(glm::radians(lat_lon.y));
    _jsbic->SetAltitudeASLFtIC(altitude * M2F());

    /*_jsbic->ResetIC(
    0, 0, 0,
    0, 0, 0,
    0, 0,
    hpr.x, hpr.y, hpr.z,
    glm::radians(lat_lon.x),
    glm::radians(lat_lon.y),
    altitude * M2F(),
    0);*/

    _jsbexec->ResetToInitialConditions(0);
    set_engine_throttle(-1, engines_thrust);

    FGColumnVector3 vel_b =
        _propagate->GetTec2b() * FGColumnVector3(vel.x, vel.y, vel.z);

    _jsbic->SetUBodyFpsIC(vel_b(1) * M2F());
    _jsbic->SetVBodyFpsIC(vel_b(2) * M2F());
    _jsbic->SetWBodyFpsIC(vel_b(3) * M2F());

    _jsbexec->RunIC();
    set_engine(is_eng_running);

    update_aircraft_data();

    _prev_pos = _aircraft_data.pos_ecef;
    _prev_rot = _aircraft_data.rot_body_ecef;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_controls(const glm::vec4& controls)
{
    if (_jsbexec->Holding())
        return;

    _FCS->SetDeCmd(controls.x);
    _FCS->SetDaCmd(controls.y);
    _FCS->SetDrCmd(controls.z);
    _FCS->SetDsCmd(-controls.z);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_engine(const bool on_off)
{
    if (!_propulsion)
        return;

    _propulsion->SetActiveEngine(-1);

    if (on_off) {
        _propulsion->SetCutoff(0);
        _propulsion->SetMagnetos(1);
        _propulsion->SetStarter(1);
        _propulsion->InitRunning(-1);
    }
    else {
        _propulsion->SetCutoff(1);
        _propulsion->SetMagnetos(0);
        _propulsion->SetStarter(0);
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_gear_brakes(const glm::vec3& brakes)
{
    _FCS->SetLBrake(brakes.x);
    _FCS->SetCBrake(brakes.y);
    _FCS->SetRBrake(brakes.z);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_flaps(const float angle)
{
    _FCS->SetDfCmd(angle);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_engine_throttle(
    const int engine, const float throttle)
{
    _FCS->SetThrottleCmd(engine, throttle);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_engine_mixture(
    const int engine, const float mixture)
{
    _FCS->SetMixtureCmd(engine, mixture);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_elevator_trim(const float trim)
{
    _FCS->SetPitchTrimCmd(trim);
    set_property("fcs/manual/pitch-trim-cmd-norm", trim);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_aileron_trim(const float trim)
{
    _FCS->SetRollTrimCmd(trim);
    set_property("fcs/manual/roll-trim-cmd-norm", trim);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_rudder_trim(const float trim)
{
    _FCS->SetYawTrimCmd(trim);
    set_property("fcs/manual/yaw-trim-cmd-norm", trim);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::pause() { _jsbexec->Hold(); }

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::resume() { _jsbexec->Resume(); }

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::update(const float dt)
{
    if (dt == 0.0f)
        return;

    const double step_time = dt + _time_rest;
    double t = step_time;

    bool step_made = false;

    while (t >= STEPs) {
        _jsbexec->Run();
        t -= STEPs;
        step_made = true;
    }

    if (step_made) {
        const FGMatrix33& m = _propagate->GetTb2ec();
        _prev_rot =
            glm::quat_cast(float3x3(
                m(1, 1), m(2, 1), m(3, 1),
                m(1, 2), m(2, 2), m(3, 2),
                m(1, 3), m(2, 3), m(3, 3)))
            * quat(0.0, 0.70710677, 0.70710677, 0.0);

        const FGLocation& loc = _auxiliary->GetLocationVRP();
        _prev_pos = double3(
            loc.Entry(1) * F2M(),
            loc.Entry(2) * F2M(),
            loc.Entry(3) * F2M());

        FGColumnVector3 vel = _propagate->GetTl2ec() * _propagate->GetVel();

        const double c = F2M();
        _prev_vel = double3(vel(1) * c, vel(2) * c, vel(3) * c);

        FGColumnVector3 pqr = _propagate->GetPQR();
        _prev_pqr = double3(pqr(1), pqr(2), pqr(3));;

        _jsbexec->Run();
        t -= STEPs;
    }

    // update aircraft_data structure
    update_aircraft_data();

    if (step_made) {
        //_accum_vel = _aircraft_data.vel_ecef;
        //_accum_avel = _aircraft_data.avel_ecef;
    }

    _time_rest = t;

    _aircraft_data.rot_body_ecef = glm::slerp(_prev_rot, _aircraft_data.rot_body_ecef, 1.0f + float(t / STEPs), true);

    const float len = glm::length(_aircraft_data.rot_body_ecef);
    _aircraft_data.rot_body_ecef *= 1.0f / len;

    // EXTRAPOLATE POSITION FROM THE VELOCITY

    const double3 vel2 = (_aircraft_data.pos_ecef - _prev_pos) / STEPs;
    const double3 pos_correction = _prev_vel * (STEPs + t);

    _aircraft_data.pos_ecef += pos_correction;
    _aircraft_data.vel_ecef = float3(_prev_vel);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_afcs(const int level)
{
    switch (level) {
    case 2:
        set_property("ap/afcs/x-lat-trim", -0.0308);
        set_property("ap/afcs/x-lon-trim", 0.025);

        set_property("ap/afcs/psi-trim-rad", _propagate->GetEuler(3));
        set_property("ap/afcs/theta-trim-rad", 0.0);
        set_property("ap/afcs/phi-trim-rad", 0.0);
        set_property("ap/afcs/h-agl-trim-ft", _aircraft_data.altitude_agl * M2F());

        set_property("ap/afcs/yaw-channel-active-norm", 1.0);
        set_property("ap/afcs/pitch-channel-active-norm", 1.0);
        set_property("ap/afcs/roll-channel-active-norm", 1.0);
        set_property("ap/afcs/altitude-channel-active-norm", 1.0);
        break;
    case 1:
        set_property("ap/afcs/x-lat-trim", 0.0);
        set_property("ap/afcs/x-lon-trim", 0.0);

        set_property("ap/afcs/psi-trim-rad", 0.0);
        set_property("ap/afcs/theta-trim-rad", 0.0);
        set_property("ap/afcs/phi-trim-rad", 0.0);
        set_property("ap/afcs/h-agl-trim-ft", 0.0);

        set_property("ap/afcs/yaw-channel-active-norm", 1.0);
        set_property("ap/afcs/pitch-channel-active-norm", 0.1);
        set_property("ap/afcs/roll-channel-active-norm", 0.1);
        set_property("ap/afcs/altitude-channel-active-norm", 0.0);
        break;
    default:
        set_property("ap/afcs/yaw-channel-active-norm", 0.0);
        set_property("ap/afcs/pitch-channel-active-norm", 0.0);
        set_property("ap/afcs/roll-channel-active-norm", 0.0);
        set_property("ap/afcs/altitude-channel-active-norm", 0.0);
        break;

    }
    _aircraft_data.afcs_level = level;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bool jsbsim_wrapper_impl::is_running() const { return !_jsbexec->Holding(); }

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

double jsbsim_wrapper_impl::get_property(const char* name)
{
    return _props->GetDouble(name);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_property(const char* name, double value)
{
    _props->SetDouble(name, value);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_gear(const bool down)
{
    _FCS->SetGearCmd(down ? 1 : 0);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//if bool parameter is false, return all contact points ( "BOGEY" (on gears) and "STRUCTURE" (on wings) )
//if bool parameter is true, return only gear/bogey contact points
uint jsbsim_wrapper_impl::get_num_contact_points(bool gearsonly)
{
    if (gearsonly)
    {
        int gearcount = 0;

        for(int i = 0; i < _jsbexec->GetGroundReactions()->GetNumGearUnits(); i++)
        {
            gearcount += _jsbexec->GetGroundReactions()->GetGearUnit(i)->IsBogey();
        }
        return gearcount;
    }
    else
    {
        return _jsbexec->GetGroundReactions()->GetNumGearUnits();
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

double3 jsbsim_wrapper_impl::get_contact_point_pos(const uint idx)
{
    const uint n = _jsbexec->GetGroundReactions()->GetNumGearUnits();

    if (idx < n) {
        FGLGear* const gear = _jsbexec->GetGroundReactions()->GetGearUnit(idx);
        quat georot = geomob->get_rot();
        double3 pos = geomob->get_pos();
        float3 gearloc = { gear->GetLocationY(), - gear->GetLocationX(), gear->GetLocationZ()};
        //from inch to meters
        gearloc *= 0.0254f;
        //change the original rotation, to the object rotation
        glm::vec3 newgearloc = glm::rotate(georot, glm::vec3(gearloc));
        //gear location is local, therefore add game object location
        pos += {newgearloc.x, newgearloc.y, newgearloc.z};

        return pos;
    }

    return double3();
}

//uint id = wheel id
//returned steer types: 0 - steerable  ; 1 - fix, 2- caster 
uint jsbsim_wrapper_impl::get_steer_type(uint id)
{
    int steer_type;
    const uint n = _jsbexec->GetGroundReactions()->GetNumGearUnits();

    if (id < n)
    {
        steer_type = _jsbexec->GetGroundReactions()->GetGearUnit(id)->GetSteerType();
        return steer_type;
    }

    return uint();
}

// 1.param = for which wheel you want to get the axis velocity
float3 jsbsim_wrapper_impl::get_wheel_axis_vel(uint wheel_id)
{
    const uint n = _jsbexec->GetGroundReactions()->GetNumGearUnits();

    if (wheel_id < n)
    {
        float3 velocities;
        velocities.x = _jsbexec->GetGroundReactions()->GetGearUnit(wheel_id)->GetWheelVel(1);
        velocities.y = _jsbexec->GetGroundReactions()->GetGearUnit(wheel_id)->GetWheelVel(2);
        velocities.z = _jsbexec->GetGroundReactions()->GetGearUnit(wheel_id)->GetWheelVel(3);
        return velocities;
    }

   return float3();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
