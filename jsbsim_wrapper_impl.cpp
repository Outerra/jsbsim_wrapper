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

#include <ot/glm/glm_ext.h>

using namespace JSBSim;

//const float integration_step = 0.01f;

const int64 STEPns = 15625000 / 2;
const double STEPs = STEPns * 1e-9;
//const double STEPs = 1.0 / 200.0;


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

jsbsim_wrapper_impl::jsbsim_wrapper_impl(ot::eng_interface *eng)
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
    DASSERT(_jsbexec.get()!=0);
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

    _aircraft_data.altitude_asl = float(_propagate->GetAltitudeASL()*F2M());

    _aircraft_data.gear_down = _FCS->GetGearCmd() != 0 ? true : false;

    if(_propulsion)
    {
        FGEngine *eng = _propulsion->GetEngine(0);
        _aircraft_data.engine_rpm = float(eng->GetThruster()->GetRPM());
        _aircraft_data.engine_throttle = float(_FCS->GetThrottlePos()[0]);
        _aircraft_data.engine_mixture = float(_FCS->GetMixturePos()[0]);
        _aircraft_data.engine_running = eng->GetRunning();

        switch(eng->GetType()) {
        case FGEngine::etElectric:
            break;
        case FGEngine::etTurbine:
            _aircraft_data.engine_egt = float(static_cast<FGTurbine*>(eng)->GetEGT());
            _aircraft_data.engine_ffl = float(static_cast<FGTurbine*>(eng)->getFuelFlow_gph());
            _aircraft_data.engine_rpm = static_cast<FGTurbine*>(eng)->GetCutoff() == false ? 1.0f : 0.0f;
            _aircraft_data.engine_mixture = float(_FCS->GetMixturePos()[0]);
            _aircraft_data.engine_n1 = float(static_cast<FGTurbine*>(eng)->GetN1());
            _aircraft_data.engine_n2 = float(static_cast<FGTurbine*>(eng)->GetN2());
            break;
        case FGEngine::etPiston:
            _aircraft_data.engine_egt = float(static_cast<FGPiston*>(eng)->GetEGT());
            _aircraft_data.engine_ffl = float(static_cast<FGPiston*>(eng)->getFuelFlow_gph());
            break;
        }
    }
    else
        _aircraft_data.engine_running = false;

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

    const FGLocation &loc_cg = _propagate->GetLocation();
    FGLocation loc = loc_cg.LocalToLocation(_propagate->GetTb2l() * _auxiliary->in.VRPBody);
    //const FGLocation &loc = _auxiliary->GetLocationVRP();
    _aircraft_data.pos_ecef = double3(
        loc.Entry(1) * F2M(),
        loc.Entry(2) * F2M(),
        loc.Entry(3) * F2M());

    _aircraft_data.pos_cg_offset = double3(
        (loc_cg(1) - loc.Entry(1)) * F2M(),
        (loc_cg(2) - loc.Entry(2)) * F2M(),
        (loc_cg(3) - loc.Entry(3)) * F2M());

    _aircraft_data.pos_latlon = double2(
        loc.GetLatitudeDeg(),
        loc.GetLongitudeDeg());
    _aircraft_data.pos_latlon_cg = double2(
        loc_cg.GetLatitudeDeg(),
        loc_cg.GetLongitudeDeg());

    const FGMatrix33 &m = _propagate->GetTb2ec();
    _aircraft_data.drot_body_ecef =
        glm::quat_cast(double3x3(
            m(1, 1), m(2, 1), m(3, 1),
            m(1, 2), m(2, 2), m(3, 2),
            m(1, 3), m(2, 3), m(3, 3)))
        * dquat(0.0, 0.70710677, 0.70710677, 0.0);

    _aircraft_data.rot_body_ecef = quat(
        float(_aircraft_data.drot_body_ecef.w),
        float(_aircraft_data.drot_body_ecef.x),
        float(_aircraft_data.drot_body_ecef.y),
        float(_aircraft_data.drot_body_ecef.z));

    const FGColumnVector3 vel = _propagate->GetECEFVelocity();
    _aircraft_data.vel_ecef = double3(vel(1) * F2M(), vel(2) * F2M(), vel(3) * F2M());

    const FGColumnVector3 avele = _propagate->GetPQR();
    _aircraft_data.avel_ecef = double3(avele(1), avele(2), avele(3));

    const FGColumnVector3 velb = _propagate->GetUVW();
    _aircraft_data.vel_body = double3(velb(1) * F2M(), velb(2) * F2M(), velb(3) * F2M());

    const FGColumnVector3 acc = _propagate->GetTb2ec() * _jsbexec->GetAccelerations()->GetBodyAccel();
    _aircraft_data.acc_body = double3(acc(1) * F2M(), acc(2) * F2M(), acc(3) * F2M());

    const FGColumnVector3 avel = _propagate->GetTb2ec() * _propagate->GetPQR();
    _aircraft_data.avel_body = double3(avel(1), avel(2), avel(3));

    const FGColumnVector3 aacc = _propagate->GetTb2ec() * _jsbexec->GetAccelerations()->GetPQRdot();
    _aircraft_data.aacc_body = double3(aacc(1), aacc(2), aacc(3));

    glm::vec3 normal;
    glm::vec3 cvel;
    glm::vec3 rvel;
    glm::dvec3 surface;
    float maxdist = float(glm::length(_aircraft_data.pos_ecef));
    float dist = _jsbroot->get_eng()->elevation_over_terrain(
        _aircraft_data.pos_ecef,
        maxdist,
        &normal,
        &surface,
        &cvel,
        &rvel,
        0,
        0,
        0);

    if(dist < maxdist)
        _aircraft_data.altitude_agl = float(dist);
    else
        _aircraft_data.altitude_agl = -1.0f;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//
// IMPLEMENTS JSBSim::jsbsim_wrapper
//
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bool jsbsim_wrapper_impl::load_aircraft(
    const coid::token &root_dir,
    const coid::token &aircrafts_dir,
    const coid::token &engines_dir,
    const coid::token &systems_dir,
    const coid::token &model)
{
    const std::string root_dir_str(root_dir.ptr(), root_dir.len());
    const std::string aircrafts_dir_str(aircrafts_dir.ptr(), aircrafts_dir.len());
    const std::string engines_dir_str(engines_dir.ptr(), engines_dir.len());
    const std::string systems_dir_str(systems_dir.ptr(), systems_dir.len());
    const std::string model_str(model.ptr(), model.len());

    _jsbexec->SetRootDir(SGPath(root_dir_str));

    if(!_jsbexec->LoadModel(
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
    if(enable)
        _jsbexec->EnableOutput();
    else
        _jsbexec->DisableOutput();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_initial_condition(
    const glm::dvec2 &lat_lon,
    const float altitude,
    const glm::quat &rot,
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

    if(false && trim && speed_kts>=60.0f) {
        _FCS->SetMixturePos(0, 1.0);
        _FCS->SetThrottlePos(0, 0.80);
        for(int i = 0; i<100; ++i)
            _propulsion->GetEngine(0)->Calculate();
        _jsbexec->DoTrim(tLongitudinal);
        _aircraft_data.trim_successful = _jsbexec->GetTrimStatus();
    }

    update_aircraft_data();

    _prev_pos = _aircraft_data.pos_ecef;
    _prev_rot = _aircraft_data.drot_body_ecef;
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
    _prev_rot = _aircraft_data.drot_body_ecef;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_initial_condition(
    const glm::dvec2 &lat_lon,
    const float altitude,
    const float3 &hpr,
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
    _prev_rot = _aircraft_data.drot_body_ecef;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_initial_condition(
    const glm::dvec2 &lat_lon,
    const float altitude,
    const float3 &hpr,
    const float3 &vel,
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
    _prev_rot = _aircraft_data.drot_body_ecef;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_controls(const glm::vec4 &controls)
{
    if(_jsbexec->Holding())
        return;

    _FCS->SetDeCmd(controls.x);
    _FCS->SetDaCmd(controls.y);
    _FCS->SetDrCmd(controls.z);
    _FCS->SetDsCmd(-controls.z);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_engine(const bool on_off)
{
    if(!_propulsion)
        return;

    _propulsion->SetActiveEngine(-1);

    if(on_off) {
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

void jsbsim_wrapper_impl::set_gear_brakes(const glm::vec3 &brakes)
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
    if(dt == 0.0f)
        return;

    const double step_time = dt + _time_rest;
    double t = step_time;

    bool step_made = false;

    while(t >= STEPs) {
        _jsbexec->Run();
        t -= STEPs;
        step_made = true;
    }

    if(step_made) {
        const FGMatrix33 &m = _propagate->GetTb2ec();
        _prev_rot =
            glm::quat_cast(double3x3(
                m(1, 1), m(2, 1), m(3, 1),
                m(1, 2), m(2, 2), m(3, 2),
                m(1, 3), m(2, 3), m(3, 3)))
            * dquat(0.0, 0.70710677, 0.70710677, 0.0);

        const FGLocation &loc = _auxiliary->GetLocationVRP();
        _prev_pos = double3(
            loc.Entry(1) * F2M(),
            loc.Entry(2) * F2M(),
            loc.Entry(3) * F2M());

        FGColumnVector3 vel = _propagate->GetTl2ec() * _propagate->GetVel();

        _prev_vel = double3(vel(1), vel(2), vel(3));
        _prev_vel *= F2M();

        FGColumnVector3 pqr = _propagate->GetPQR();
        _prev_pqr = double3(pqr(1), pqr(2), pqr(3));;

        _jsbexec->Run();
        t -= STEPs;
    }

    // update aircraft_data structure
    update_aircraft_data();

    if(step_made) {
        //_accum_vel = _aircraft_data.vel_ecef;
        //_accum_avel = _aircraft_data.avel_ecef;
    }

    _time_rest = t;

    {
        _aircraft_data.drot_body_ecef =
            glm::slerp(_prev_rot, _aircraft_data.drot_body_ecef, 1.0 + (t / STEPs), true);

        const double len = glm::length(_aircraft_data.drot_body_ecef);
        _aircraft_data.drot_body_ecef *= 1.0 / len;
    }

    // EXTRAPOLATE POSITION FROM THE VELOCITY

    const double3 vel2 = (_aircraft_data.pos_ecef - _prev_pos) / STEPs;
    const double3 pos_correction = _prev_vel * (STEPs + t);

    _aircraft_data.pos_ecef += pos_correction;
    _aircraft_data.vel_ecef = _prev_vel;

    _aircraft_data.rot_body_ecef = quat(
        float(_aircraft_data.drot_body_ecef.w),
        float(_aircraft_data.drot_body_ecef.x),
        float(_aircraft_data.drot_body_ecef.y),
        float(_aircraft_data.drot_body_ecef.z));
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_afcs(const int level)
{
    switch(level) {
    case 2:
        set_property("ap/afcs/x-lat-trim", -0.0308);
        set_property("ap/afcs/x-lon-trim", 0.025);

        set_property("ap/afcs/psi-trim-rad", _propagate->GetEuler(3));
        set_property("ap/afcs/theta-trim-rad", 0.0);
        set_property("ap/afcs/phi-trim-rad", 0.0);
        set_property("ap/afcs/h-agl-trim-ft", _aircraft_data.altitude_agl*M2F());

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

double jsbsim_wrapper_impl::get_property(const char *name)
{
    return _props->GetDouble(name);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_property(const char *name, double value)
{
    _props->SetDouble(name, value);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_gear(const bool down)
{
    _FCS->SetGearCmd(down ? 1 : 0);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

uint jsbsim_wrapper_impl::get_num_gear_contact_point()
{
    return _jsbexec->GetGroundReactions()->GetNumGearUnits();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

float3 jsbsim_wrapper_impl::get_gear_contact_point(const uint idx)
{
    const uint n = _jsbexec->GetGroundReactions()->GetNumGearUnits();

    if(idx < n) {
        FGLGear * const gear = _jsbexec->GetGroundReactions()->GetGearUnit(idx);
        return float3(
            gear->GetLocationY(),
            -gear->GetLocationX(),
            gear->GetLocationZ())  * 0.0254f;
    }

    return float3();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
