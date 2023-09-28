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

jsbsim_wrapper_impl::jsbsim_wrapper_impl(ot::eng_interface* eng, uint object_id)
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

    sketch = ot::sketch::get();
    world = ot::world::get();
    object = world->get_object(object_id);
    geomob = object->get_geomob(0);

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


    //////////// Ground reactions & Gears

    ot::aircraft_data::GndReactions GndReactdata;

    GndReactdata.BumpHeight = _groundReactions->GetBumpHeight();
    GndReactdata.Bumpiness = _groundReactions->GetBumpiness();
    GndReactdata.SteeringCmd = _groundReactions->GetDsCmd();
    GndReactdata.GndreactMaximumForce = _groundReactions->GetMaximumForce();
    GndReactdata.RollingFrictFactor = _groundReactions->GetRollingFFactor();
    GndReactdata.SurfaceSolidStatus = _groundReactions->GetSolid();
    GndReactdata.StaticFrictFactor = _groundReactions->GetStaticFFactor();
    GndReactdata.WOW = _groundReactions->GetWOW();
    GndReactdata.GndreactForces = { _groundReactions->GetForces().Entry(1), _groundReactions->GetForces().Entry(2) ,_groundReactions->GetForces().Entry(3) };
    GndReactdata.GndreactMoments = { _groundReactions->GetMoments().Entry(1), _groundReactions->GetMoments().Entry(2) , _groundReactions->GetMoments().Entry(3) };

    uint NumCP = get_num_contact_points(false);
    _aircraft_data.Gears.realloc(NumCP);

    for (uint i = 0; i < NumCP; i++)
    {
        ot::aircraft_data::Gear& Geardata = _aircraft_data.Gears[i];
        JSBSim::FGLGear* wheel = _groundReactions->GetGearUnit(i);

        Geardata.SteerType = get_steer_type(i);
        Geardata.ContactPointPos = get_contact_point_pos(i);
        Geardata.AxisVelocities = get_wheel_axis_vel(i);
        Geardata.GearWOW = wheel->GetWOW();
        Geardata.GearAnglesToBody = { wheel->GetAnglesToBody().Entry(1),wheel->GetAnglesToBody().Entry(2) ,wheel->GetAnglesToBody().Entry(3) };
        Geardata.GearBodyForces = { wheel->GetBodyForces().Entry(1),wheel->GetBodyForces().Entry(2) ,wheel->GetBodyForces().Entry(3) };
        Geardata.GearBrakeGroup = wheel->GetBrakeGroup();
        Geardata.GearCompForce = wheel->GetCompForce();
        Geardata.GearCompLength = wheel->GetCompLen();
        Geardata.GearPos = wheel->GetGearUnitPos();
        Geardata.LocalGear = { wheel->GetLocalGear().Entry(1),wheel->GetLocalGear().Entry(2) ,wheel->GetLocalGear().Entry(3) };
        Geardata.GearMoments = { wheel->GetMoments().Entry(1),wheel->GetMoments().Entry(2) ,wheel->GetMoments().Entry(3) };
        Geardata.GearPitch = wheel->GetPitch();
        Geardata.GearYaw = wheel->GetYaw();
        Geardata.GearisRetractable = wheel->GetRetractable();
        Geardata.GearStaticFrictCoeff = wheel->GetstaticFCoeff();
        Geardata.GearSteerAngleDeg = wheel->GetSteerAngleDeg();
        Geardata.GearSteerNorm = wheel->GetSteerNorm();
        Geardata.GearTransformType = wheel->GetTransformType();
        Geardata.GearRollForce = wheel->GetWheelRollForce();
        Geardata.GearRollVel = wheel->GetWheelRollVel();
        Geardata.GearSideForce = wheel->GetWheelSideForce();
        Geardata.GearSideVel = wheel->GetWheelSideVel();
        Geardata.GearSlipAngle = wheel->GetWheelSlipAngle();
        Geardata.GearVelX = wheel->GetWheelVel(1);
        Geardata.GearVelY = wheel->GetWheelVel(2);
        Geardata.GearVelZ = wheel->GetWheelVel(3);
        Geardata.GearisBogey = wheel->IsBogey();

        if (Geardata.WheelName.is_empty())
        {
            Geardata.WheelName = wheel->GetName().c_str();
        }
    }


    //////////// FCS

    ot::aircraft_data::FCS FCSdata;

    FCSdata.LeftAileronPosRad = _FCS->GetDaLPos();
    FCSdata.RightAileronPosRad = _FCS->GetDaRPos();
    FCSdata.ElevatorPosRad = _FCS->GetDePos();
    FCSdata.FlapsPosRad = _FCS->GetDfPos();
    FCSdata.RudderPosRad = _FCS->GetDrPos();
    FCSdata.SpeedBrakePosRad = _FCS->GetDsbPos();
    FCSdata.SpoilerPosRad = _FCS->GetDspPos();
    FCSdata.RightBrakePos = _FCS->GetRBrake();
    FCSdata.LeftBrakePos = _FCS->GetLBrake();
    FCSdata.CenterBrakePos = _FCS->GetCBrake();
    // tailhook position  0 - up,  1 - down
    FCSdata.TailhookPos = _FCS->GetTailhookPos();
    //wing fold position 0 - unfolded, 1 - folded
    FCSdata.WingFoldPos = _FCS->GetWingFoldPos();
    FCSdata.TrimStatus = _FCS->GetTrimStatus();
    FCSdata.ChannelDeltaT = _FCS->GetChannelDeltaT();
    FCSdata.AileronCmd = _FCS->GetDaCmd();
    FCSdata.ElevatorCmd = _FCS->GetDeCmd();
    FCSdata.FlapsCmd = _FCS->GetDfCmd();
    FCSdata.RudderCmd = _FCS->GetDrCmd();
    FCSdata.SpeedbrakeCmd = _FCS->GetDsbCmd();
    FCSdata.SteeringCmd = _FCS->GetDsCmd();
    FCSdata.SpoilerCmd = _FCS->GetDspCmd();
    FCSdata.GearCmd = _FCS->GetGearCmd();
    FCSdata.PitchTrimCmd = _FCS->GetPitchTrimCmd();
    FCSdata.RollTrimCmd = _FCS->GetRollTrimCmd();
    FCSdata.YawTrimCmd = _FCS->GetYawTrimCmd();


    //////////// Propulsion & Tanks & Engines + EngineFCS

    ot::aircraft_data::Propulsion Propdata;

    Propdata.GetActiveEngine = _propulsion->GetActiveEngine();
    Propdata.FuelFreezeStatus = _propulsion->GetFuelFreeze();
    //num of fuel tanks currently actively supplying fuel
    Propdata.ActiveFuelTanks = _propulsion->GetnumSelectedFuelTanks();
    //num of fuel tanks currently actively supplying oxidizer
    Propdata.ActiveOxiTanks = _propulsion->GetnumSelectedOxiTanks();
    Propdata.NumTanks = _propulsion->GetNumTanks();
    Propdata.NumEngines = _propulsion->GetNumEngines();
    Propdata.TanksWeight = _propulsion->GetTanksWeight();
    Propdata.PropForces = { _propulsion->GetForces().Entry(1), _propulsion->GetForces().Entry(2), _propulsion->GetForces().Entry(3) };
    Propdata.PropMoments = { _propulsion->GetMoments().Entry(1), _propulsion->GetMoments().Entry(2), _propulsion->GetMoments().Entry(3) };
    Propdata.TanksMoment = { _propulsion->GetTanksMoment().Entry(1), _propulsion->GetTanksMoment().Entry(2), _propulsion->GetTanksMoment().Entry(3) };

    //Tanks
    _aircraft_data.Tanks.realloc(Propdata.NumTanks);

    for (int i = 0; i < Propdata.NumTanks; i++)
    {
        ot::aircraft_data::Tank& Tankdata = _aircraft_data.Tanks[i];
        JSBSim::FGTank* _tank = _propulsion->GetTank(i);

        Tankdata.TankCapacityLbs = _tank->GetCapacity();
        Tankdata.TankCapacityGal = _tank->GetCapacityGallons();
        Tankdata.TankContentsLbs = _tank->GetContents();
        Tankdata.TankContentsGal = _tank->GetContentsGallons();
        Tankdata.TankDensity = _tank->GetDensity();
        Tankdata.TankExternalFlow = _tank->GetExternalFlow();
        //fill level in percents 0-100
        Tankdata.TankFillLvlPct = _tank->GetPctFull();
        Tankdata.TankPriority = _tank->GetPriority();
        Tankdata.TankSupplyStatus = _tank->GetSelected();
        Tankdata.TankStandpipe = _tank->GetStandpipe();
        Tankdata.TankTempDegF = _tank->GetTemperature();
        Tankdata.TankTempDegC = _tank->GetTemperature_degC();
        //0-undefined, 1-fuel, 2-oxidizer
        Tankdata.TankType = _tank->GetType();
        Tankdata.TankXYZ = { _tank->GetXYZ().Entry(1), _tank->GetXYZ().Entry(2), _tank->GetXYZ().Entry(3) };
    }

    //Engines + EngineFCS
    _aircraft_data.Engines.realloc(Propdata.NumEngines);
    _aircraft_data.EnginesFCS.realloc(Propdata.NumEngines);

    for (int i = 0; i < Propdata.NumEngines; i++)
    {
        ot::aircraft_data::Engine& Enginedata = _aircraft_data.Engines[i];
        JSBSim::FGEngine* _engine = _propulsion->GetEngine(i);
        //total fuel requirements for this engine in pounds
        Enginedata.EngFuelNeeded = _engine->CalcFuelNeed();
        Enginedata.EngOxiNeeded = _engine->CalcOxidizerNeed();
        Enginedata.EngCrankingStatus = _engine->GetCranking();
        Enginedata.EngFuelFlowRate = _engine->GetFuelFlowRate();
        Enginedata.EngFuelUsedLbs = _engine->GetFuelUsedLbs();
        Enginedata.EngNumSourceTanks = _engine->GetNumSourceTanks();
        Enginedata.EngPowerAvailable = _engine->GetPowerAvailable();
        Enginedata.EngSourceTank = _engine->GetSourceTank(0);
        Enginedata.EngStarvedStatus = _engine->GetStarved();
        Enginedata.EngType = _engine->GetType();
        Enginedata.EngStarter = _engine->GetStarter();
        Enginedata.EngRunning = _engine->GetRunning();
        Enginedata.EngThrust = _engine->GetThrust();
        Enginedata.EngineMoments = { _engine->GetMoments().Entry(1), _engine->GetMoments().Entry(2), _engine->GetMoments().Entry(3) };
        Enginedata.EngineForces = { _engine->GetBodyForces().Entry(1), _engine->GetBodyForces().Entry(2), _engine->GetBodyForces().Entry(3) };

        if (Enginedata.EngName.is_empty())
        {
            Enginedata.EngName = _engine->GetName().c_str();
        }

        JSBSim::FGThruster* _thruster = _engine->GetThruster();

        Enginedata.ThrusterThrust = _thruster->GetThrust();
        Enginedata.ThrusterEngineRPM = _thruster->GetEngineRPM();
        Enginedata.ThrusterRPM = _thruster->GetRPM();
        Enginedata.ThrusterGearRatio = _thruster->GetGearRatio();
        Enginedata.ThrusterPitch = _thruster->GetPitch();
        Enginedata.ThrusterPowerRequired = _thruster->GetPowerRequired();
        Enginedata.ThrusterReverseAngle = _thruster->GetReverserAngle();
        Enginedata.ThrusterType = _thruster->GetType();
        Enginedata.ThrusterTransformType = _thruster->GetTransformType();
        Enginedata.ThrusterYaw = _thruster->GetYaw();
        Enginedata.ThrusterAnglesToBody = { _thruster->GetAnglesToBody().Entry(1), _thruster->GetAnglesToBody().Entry(2), _thruster->GetAnglesToBody().Entry(3) };
        Enginedata.ThrusterBodyForces = { _thruster->GetBodyForces().Entry(1), _thruster->GetBodyForces().Entry(2), _thruster->GetBodyForces().Entry(3) };
        Enginedata.ThrusterLoc = { _thruster->GetLocation().Entry(1), _thruster->GetLocation().Entry(2), _thruster->GetLocation().Entry(3) };
        Enginedata.ThrusterMoments = { _thruster->GetMoments().Entry(1), _thruster->GetMoments().Entry(2), _thruster->GetMoments().Entry(3) };
        Enginedata.ThrusterActingLoc = { _thruster->GetActingLocation().Entry(1), _thruster->GetActingLocation().Entry(2), _thruster->GetActingLocation().Entry(3) };

        ot::aircraft_data::EngineFCS& EngineFCSdata = _aircraft_data.EnginesFCS[i];

        //EngineFCS
        //param - engine number
        EngineFCSdata.Mixture = _FCS->GetMixturePos(i);
        EngineFCSdata.Throttle = _FCS->GetThrottlePos(i);
        EngineFCSdata.PropAdvance = _FCS->GetPropAdvance(i);
        EngineFCSdata.PropFeatherStatus = _FCS->GetPropFeather(i);
        EngineFCSdata.ThrottleCmd = _FCS->GetThrottleCmd(i);
        EngineFCSdata.PropAdvanceCmd = _FCS->GetPropAdvanceCmd(i);
        EngineFCSdata.MixtureCmd = _FCS->GetMixtureCmd(i);
        EngineFCSdata.FeatherCmd = _FCS->GetFeatherCmd(i);
    }


    //////////// Propagate

    ot::aircraft_data::Propagate Propagdata;

    Propagdata.AltitudeASL = _propagate->GetAltitudeASL();
    Propagdata.AltitudeASLm = _propagate->GetAltitudeASLmeters();
    //retrieves sine of vehicle Euler angle component (Phi - 1, Theta - 2 or Psi - 3)
    Propagdata.SinEulerPhi = _propagate->GetSinEuler(1);
    Propagdata.SinEulerTheta = _propagate->GetSinEuler(2);
    Propagdata.SinEulerPsi = _propagate->GetSinEuler(3);
    //retrieves cosine of vehicle Euler angle component (Phi - 1, Theta - 2 or Psi - 3)
    Propagdata.CosEulerPhi = _propagate->GetCosEuler(1);
    Propagdata.CosEulerTheta = _propagate->GetCosEuler(2);
    Propagdata.CosEulerPsi = _propagate->GetCosEuler(3);
    Propagdata.DistanceAGL = _propagate->GetDistanceAGL();
    Propagdata.DistanceAGLkm = _propagate->GetDistanceAGLKm();
    Propagdata.EarthPosAngle = _propagate->GetEarthPositionAngle();
    Propagdata.EarthPosAngleDeg = _propagate->GetEarthPositionAngleDeg();
    Propagdata.ECEFvel = { _propagate->GetECEFVelocity().Entry(1), _propagate->GetECEFVelocity().Entry(2) , _propagate->GetECEFVelocity().Entry(3) };
    Propagdata.Euler = { _propagate->GetEuler().Entry(1),_propagate->GetEuler().Entry(2) ,_propagate->GetEuler().Entry(3) };
    Propagdata.EulerDeg = { _propagate->GetEulerDeg().Entry(1),_propagate->GetEulerDeg().Entry(2) ,_propagate->GetEulerDeg().Entry(3) };
    Propagdata.GeodAlt = _propagate->GetGeodeticAltitude();
    Propagdata.GeodAltKm = _propagate->GetGeodeticAltitudeKm();
    Propagdata.GeodLatDeg = _propagate->GetGeodLatitudeDeg();
    Propagdata.GeodLatRad = _propagate->GetGeodLatitudeRad();
    Propagdata.CurrentAltRate = _propagate->Gethdot();
    Propagdata.InertialPos = { _propagate->GetInertialPosition().Entry(1), _propagate->GetInertialPosition().Entry(2), _propagate->GetInertialPosition().Entry(3) };
    Propagdata.InertialVel = { _propagate->GetInertialVelocity().Entry(1), _propagate->GetInertialVelocity().Entry(2), _propagate->GetInertialVelocity().Entry(3) };
    Propagdata.InertialVelMag = _propagate->GetInertialVelocityMagnitude();
    Propagdata.Latitude = _propagate->GetLatitude();
    Propagdata.LatitudeDeg = _propagate->GetLatitudeDeg();
    Propagdata.LocalTerrainRadius = _propagate->GetLocalTerrainRadius();
    Propagdata.PropagateLoc = { _propagate->GetLocation().Entry(1), _propagate->GetLocation().Entry(2), _propagate->GetLocation().Entry(3) };
    Propagdata.Longtitude = _propagate->GetLongitude();
    Propagdata.LongtitudeDeg = _propagate->GetLongitudeDeg();
    Propagdata.NEDvelMagn = _propagate->GetNEDVelocityMagnitude();
    Propagdata.PropagatePQR = { _propagate->GetPQR().Entry(1), _propagate->GetPQR().Entry(2), _propagate->GetPQR().Entry(3) };
    Propagdata.PropagatePQRi = { _propagate->GetPQRi().Entry(1), _propagate->GetPQRi().Entry(2), _propagate->GetPQRi().Entry(3) };
    Propagdata.PropagateRadius = _propagate->GetRadius();
    Propagdata.TerrainAngularVel = { _propagate->GetTerrainAngularVelocity().Entry(1),_propagate->GetTerrainAngularVelocity().Entry(2), _propagate->GetTerrainAngularVelocity().Entry(3) };
    Propagdata.TerrainElevation = _propagate->GetTerrainElevation();
    Propagdata.TerrainVel = { _propagate->GetTerrainVelocity().Entry(1), _propagate->GetTerrainVelocity().Entry(2), _propagate->GetTerrainVelocity().Entry(3) };
    Propagdata.PropagateUVW = { _propagate->GetUVW().Entry(1), _propagate->GetUVW().Entry(2), _propagate->GetUVW().Entry(3) };
    Propagdata.PropagateVel = { _propagate->GetVel().Entry(1), _propagate->GetVel().Entry(2), _propagate->GetVel().Entry(3) };


    //////////// atmosphere

    ot::aircraft_data::Atmosphere Atmosdata;

    Atmosdata.AtmosDensity = _atmosphere->GetDensity();
    Atmosdata.AbsoluteViscosity = _atmosphere->GetAbsoluteViscosity();
    Atmosdata.DensityAltitude = _atmosphere->GetDensityAltitude();
    /// Returns the ratio of at-altitude density over the sea level value.
    Atmosdata.DensityRatio = _atmosphere->GetDensityRatio();
    /// Returns the sea level density in slugs/ft^3
    Atmosdata.DensitySL = _atmosphere->GetDensitySL();
    Atmosdata.KinematicViscosity = _atmosphere->GetKinematicViscosity();
    //can take param double altitude - returns pressure at specified altitude in psf
    //altitude is in ft
    Atmosdata.AtmosPressure = _atmosphere->GetPressure(Propagdata.AltitudeASL);
    Atmosdata.PressureAltitude = _atmosphere->GetPressureAltitude();
    Atmosdata.PressureRatio = _atmosphere->GetPressureRatio();
    Atmosdata.PressureSL = _atmosphere->GetPressureSL();
    //can take param double altitude(ft?) - returns speed of sound ft/sec at given altitude in ft
    Atmosdata.SoundSpeed = _atmosphere->GetSoundSpeed(Propagdata.AltitudeASL);
    /// Returns the ratio of at-altitude sound speed over the sea level value.
    Atmosdata.SoundSpeedRatio = _atmosphere->GetSoundSpeedRatio();
    /// Returns the sea level speed of sound in ft/sec.
    Atmosdata.SoundSpeedSL = _atmosphere->GetSoundSpeedSL();
    /// Returns the actual, modeled temperature at the current altitude in degrees Rankine.
    Atmosdata.TemperatureRa = _atmosphere->GetTemperature(Propagdata.AltitudeASL);
    /// Returns the ratio at-current-altitude temperature as modeled over the sea level value
    Atmosdata.TemperatureRatio = _atmosphere->GetTemperatureRatio(Propagdata.AltitudeASL);
    /// Returns the actual, modeled sea level temperature in degrees Rankine.
    Atmosdata.TemperatureSLRa = _atmosphere->GetTemperatureSL();


    //////////// accelerations

    ot::aircraft_data::Accelerations Acceldata;
    JSBSim::FGAccelerations* _accelerations = _jsbexec->GetAccelerations();

    Acceldata.GravAccelMagnitude = _accelerations->GetGravAccelMagnitude();

    //retrieves acceleration resulting from applied forces
    Acceldata.BodyAccel = { _accelerations->GetBodyAccel().Entry(1), _accelerations->GetBodyAccel().Entry(2), _accelerations->GetBodyAccel().Entry(3) };
    //retrieves the total forces applied on the body
    Acceldata.AccelForces = { _accelerations->GetForces().Entry(1), _accelerations->GetForces().Entry(2), _accelerations->GetForces().Entry(3) };
    Acceldata.GravAccel = { _accelerations->GetGravAccel().Entry(1), _accelerations->GetGravAccel().Entry(2), _accelerations->GetGravAccel().Entry(3) };
    //retrieves ground forces applied on the body
    Acceldata.GroundForces = { _accelerations->GetGroundForces().Entry(1), _accelerations->GetGroundForces().Entry(2), _accelerations->GetGroundForces().Entry(3) };
    //retrieves ground moments applied on the body
    Acceldata.GroundMoments = { _accelerations->GetGroundMoments().Entry(1), _accelerations->GetGroundMoments().Entry(2), _accelerations->GetGroundMoments().Entry(3) };
    //retrieves a component of the total moments applied on the body
    Acceldata.AccelMoments = { _accelerations->GetMoments().Entry(1), _accelerations->GetMoments().Entry(2), _accelerations->GetMoments().Entry(3) };
    //retrieves the body axis angular acceleration vector
    Acceldata.AccelPQRdot = { _accelerations->GetPQRdot().Entry(1),_accelerations->GetPQRdot().Entry(2), _accelerations->GetPQRdot().Entry(3) };
    //retrieves the body axis angular acceleration vector in ECI frame
    Acceldata.AccelPQRidot = { _accelerations->GetPQRidot().Entry(1), _accelerations->GetPQRidot().Entry(2),_accelerations->GetPQRidot().Entry(3) };
    //retrieves the body axis acceleration
    Acceldata.AccelUVWdot = { _accelerations->GetUVWdot().Entry(1),_accelerations->GetUVWdot().Entry(2), _accelerations->GetUVWdot().Entry(3) };
    //retrieves the body axis acceleration in the ECI frame
    Acceldata.AccelUVWidot = { _accelerations->GetUVWidot().Entry(1), _accelerations->GetUVWidot().Entry(2), _accelerations->GetUVWidot().Entry(3) };
    //retrieves the weight applied on the body
    Acceldata.Weight = { _accelerations->GetWeight().Entry(1), _accelerations->GetWeight().Entry(2), _accelerations->GetWeight().Entry(3) };


    //////////// MassBalance

    ot::aircraft_data::MassBalance Massbaldata;

    Massbaldata.EmptyWeight = _massBalance->GetEmptyWeight();
    Massbaldata.Mass = _massBalance->GetMass();
    Massbaldata.PointMassWeight = _massBalance->GetTotalPointMassWeight();
    Massbaldata.Weight = _massBalance->GetWeight();
    //can take parameter int axis
    Massbaldata.DeltaXYZcg = { _massBalance->GetDeltaXYZcg().Entry(1), _massBalance->GetDeltaXYZcg().Entry(2), _massBalance->GetDeltaXYZcg().Entry(3) };
    //can take parameter int axis
    Massbaldata.XYZcg = { _massBalance->GetXYZcg().Entry(1), _massBalance->GetXYZcg().Entry(2), _massBalance->GetXYZcg().Entry(3) };
    Massbaldata.PointMassMoment = { _massBalance->GetPointMassMoment().Entry(1), _massBalance->GetPointMassMoment().Entry(2), _massBalance->GetPointMassMoment().Entry(3) };


    //////////// Aerodynamics

    ot::aircraft_data::Aerodynamics Aerodyndata;

    Aerodyndata.AlphaCLmax = _aerodynamics->GetAlphaCLMax();
    Aerodyndata.AlphaCLmin = _aerodynamics->GetAlphaCLMin();
    Aerodyndata.AlphaW = _aerodynamics->GetAlphaW();
    Aerodyndata.BI2vel = _aerodynamics->GetBI2Vel();
    Aerodyndata.CI2vel = _aerodynamics->GetCI2Vel();
    //gets square of the lift coeficient
    Aerodyndata.LiftCoefSq = _aerodynamics->GetClSquared();
    //gets aerodynamic vector
    Aerodyndata.AerodynForces = { _aerodynamics->GetForces().Entry(1),_aerodynamics->GetForces().Entry(2) ,_aerodynamics->GetForces().Entry(3) };
    Aerodyndata.HysteresisParm = _aerodynamics->GetHysteresisParm();
    //gets lift over drag ratio
    Aerodyndata.LoD = _aerodynamics->GetLoD();
    //gets aerodynamic moment vector about the CG - total or for given axis
    Aerodyndata.AerodynMoments = { _aerodynamics->GetMoments().Entry(1),_aerodynamics->GetMoments().Entry(2) ,_aerodynamics->GetMoments().Entry(3) };
    //gets aerodynamic moment vector about the moment reference center - total or for given axis
    Aerodyndata.AerodynMomentsMRC = { _aerodynamics->GetMomentsMRC().Entry(1),_aerodynamics->GetMomentsMRC().Entry(2) ,_aerodynamics->GetMomentsMRC().Entry(3) };
    Aerodyndata.StallWarn = _aerodynamics->GetStallWarn();
    //gets aerodynamic forces in the wind axes
    Aerodyndata.ForcesWindAxes = { _aerodynamics->GetvFw().Entry(1),_aerodynamics->GetvFw().Entry(2) ,_aerodynamics->GetvFw().Entry(3) };


    //////////// Inertial

    ot::aircraft_data::Inertial Inertialdata;

    JSBSim::FGColumnVector3 GravJ2 = _inertial->GetGravityJ2(_propagate->GetLocation());
    Inertialdata.GravityJ2 = { GravJ2.Entry(1),GravJ2.Entry(2) ,GravJ2.Entry(3) };
    Inertialdata.OmegaPlanet = { _inertial->GetOmegaPlanet().Entry(1),_inertial->GetOmegaPlanet().Entry(2) ,_inertial->GetOmegaPlanet().Entry(3) };
    Inertialdata.RefRadius = _inertial->GetRefRadius();
    Inertialdata.Semimajor = _inertial->GetSemimajor();
    Inertialdata.Semiminor = _inertial->GetSemiminor();
    Inertialdata.Omega = _inertial->omega();
    Inertialdata.SLgravity = _inertial->SLgravity();


    //////////// Aircraft


    ot::aircraft_data::Aircraft Aircraftdata;

    Aircraftdata.Cbar = _aircraft->Getcbar();
    Aircraftdata.HtailArea = _aircraft->GetHTailArea();
    Aircraftdata.HtailArm = _aircraft->GetHTailArm();
    Aircraftdata.LbarH = _aircraft->Getlbarh();
    Aircraftdata.Lbarv = _aircraft->Getlbarv();
    Aircraftdata.PitotAngle = _aircraft->GetPitotAngle();
    Aircraftdata.VbarH = _aircraft->Getvbarh();
    Aircraftdata.VbarV = _aircraft->Getvbarv();
    Aircraftdata.VtailArea = _aircraft->GetVTailArea();
    Aircraftdata.VtailArm = _aircraft->GetVTailArm();
    Aircraftdata.WingArea = _aircraft->GetWingArea();
    Aircraftdata.WingIncidence = _aircraft->GetWingIncidence();
    Aircraftdata.WingIncidenceDeg = _aircraft->GetWingIncidenceDeg();
    Aircraftdata.WingSpan = _aircraft->GetWingSpan();
    Aircraftdata.AircraftForces = { _aircraft->GetForces().Entry(1),_aircraft->GetForces().Entry(2) ,_aircraft->GetForces().Entry(3) };
    Aircraftdata.AircraftMoments = { _aircraft->GetMoments().Entry(1),_aircraft->GetMoments().Entry(2) ,_aircraft->GetMoments().Entry(3) };
    Aircraftdata.AircraftXYZep = { _aircraft->GetXYZep().Entry(1),_aircraft->GetXYZep().Entry(2) ,_aircraft->GetXYZep().Entry(3) };
    Aircraftdata.AircraftXYZrp = { _aircraft->GetXYZrp().Entry(1),_aircraft->GetXYZrp().Entry(2) ,_aircraft->GetXYZrp().Entry(3) };
    Aircraftdata.AircraftXYRvrp = { _aircraft->GetXYZvrp().Entry(1),_aircraft->GetXYZvrp().Entry(2) ,_aircraft->GetXYZvrp().Entry(3) };

    if (Aircraftdata.AircraftName.is_empty())
    {
        Aircraftdata.AircraftName = _aircraft->GetAircraftName().c_str();
    }


    //////////// Auxiliary

    ot::aircraft_data::Auxiliary Auxiliarydata;

    Auxiliarydata.DayOfYear = _auxiliary->GetDayOfYear();
    Auxiliarydata.AuxAdot = _auxiliary->Getadot();
    Auxiliarydata.AuxAlpha = _auxiliary->Getalpha();
    Auxiliarydata.AuxBdot = _auxiliary->Getbdot();
    Auxiliarydata.AuxBeta = _auxiliary->Getbeta();
    Auxiliarydata.CrossWind = _auxiliary->GetCrossWind();
    Auxiliarydata.DistanceRelativePos = _auxiliary->GetDistanceRelativePosition();
    Auxiliarydata.AuxGamma = _auxiliary->GetGamma();
    Auxiliarydata.GroundTrack = _auxiliary->GetGroundTrack();
    Auxiliarydata.HeadWind = _auxiliary->GetHeadWind();
    Auxiliarydata.HOverBCG = _auxiliary->GetHOverBCG();
    Auxiliarydata.HOverBMAC = _auxiliary->GetHOverBMAC();
    Auxiliarydata.hVRP = _auxiliary->GethVRP();
    Auxiliarydata.LatitudeRelatPos = _auxiliary->GetLatitudeRelativePosition();
    Auxiliarydata.LongtitudeRelatPos = _auxiliary->GetLongitudeRelativePosition();
    Auxiliarydata.Mach = _auxiliary->GetMach();
    Auxiliarydata.MachU = _auxiliary->GetMachU();
    Auxiliarydata.MagBeta = _auxiliary->GetMagBeta();
    Auxiliarydata.Nlf = _auxiliary->GetNlf();
    Auxiliarydata.Ny = _auxiliary->GetNy();
    Auxiliarydata.Nz = _auxiliary->GetNz();
    Auxiliarydata.Qbar = _auxiliary->Getqbar();
    Auxiliarydata.QbarUV = _auxiliary->GetqbarUV();
    Auxiliarydata.QbarUW = _auxiliary->GetqbarUW();
    Auxiliarydata.ReynoldsNum = _auxiliary->GetReynoldsNumber();
    Auxiliarydata.SecondsInDay = _auxiliary->GetSecondsInDay();
    Auxiliarydata.TAT_C = _auxiliary->GetTAT_C();
    Auxiliarydata.TotalPressure = _auxiliary->GetTotalPressure();
    Auxiliarydata.TotalTemp = _auxiliary->GetTotalTemperature();
    Auxiliarydata.VcalibratedFPS = _auxiliary->GetVcalibratedFPS();
    Auxiliarydata.VcalibratedKTS = _auxiliary->GetVcalibratedKTS();
    Auxiliarydata.VequivalentFPS = _auxiliary->GetVequivalentFPS();
    Auxiliarydata.VequivalentKTS = _auxiliary->GetVequivalentKTS();
    Auxiliarydata.Vground = _auxiliary->GetVground();
    Auxiliarydata.Vt = _auxiliary->GetVt();
    Auxiliarydata.VtrueFPS = _auxiliary->GetVtrueFPS();
    Auxiliarydata.VtrueKTS = _auxiliary->GetVtrueKTS();
    Auxiliarydata.AeroPQR = { _auxiliary->GetAeroPQR().Entry(1),_auxiliary->GetAeroPQR().Entry(2) ,_auxiliary->GetAeroPQR().Entry(3) };
    Auxiliarydata.EulerRates = { _auxiliary->GetEulerRates().Entry(1),_auxiliary->GetEulerRates().Entry(2) ,_auxiliary->GetEulerRates().Entry(3) };
    Auxiliarydata.LocVRP = { _auxiliary->GetLocationVRP().Entry(1),_auxiliary->GetLocationVRP().Entry(2) ,_auxiliary->GetLocationVRP().Entry(3) };
    Auxiliarydata.Ncg = { _auxiliary->GetNcg().Entry(1), _auxiliary->GetNcg().Entry(2), _auxiliary->GetNcg().Entry(3) };
    Auxiliarydata.Npilot = { _auxiliary->GetNpilot().Entry(1),_auxiliary->GetNpilot().Entry(2) ,_auxiliary->GetNpilot().Entry(3) };
    Auxiliarydata.Nwcg = { _auxiliary->GetNwcg().Entry(1),_auxiliary->GetNwcg().Entry(2) ,_auxiliary->GetNwcg().Entry(3) };
    Auxiliarydata.PilotAccel = { _auxiliary->GetPilotAccel().Entry(1),_auxiliary->GetPilotAccel().Entry(2) ,_auxiliary->GetPilotAccel().Entry(3) };

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
    int contact_point_count = _groundReactions->GetNumGearUnits();

    if (gearsonly)
    {
        int gearcount = 0;

        for (int i = 0; i < contact_point_count; i++)
        {
            gearcount += _groundReactions->GetGearUnit(i)->IsBogey();
        }
        return gearcount;
    }
    else
    {
        return contact_point_count;
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

double3 jsbsim_wrapper_impl::get_contact_point_pos(const uint idx)
{
    const uint n = _groundReactions->GetNumGearUnits();

    if (idx < n) {
        FGLGear* const gear = _groundReactions->GetGearUnit(idx);
        quat georot = geomob->get_rot();
        double3 pos = geomob->get_pos();
        float3 gearloc = { gear->GetLocationY(), -gear->GetLocationX(), gear->GetLocationZ() };
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
    const uint n = _groundReactions->GetNumGearUnits();

    if (id < n)
    {
        steer_type = _groundReactions->GetGearUnit(id)->GetSteerType();
        return steer_type;
    }

    return uint();
}


float3 jsbsim_wrapper_impl::get_wheel_axis_vel(uint wheel_id)
{
    const uint n = _groundReactions->GetNumGearUnits();

    if (wheel_id < n)
    {
        JSBSim::FGLGear* gear_unit = _groundReactions->GetGearUnit(wheel_id);
        float3 velocities;

        velocities.x = gear_unit->GetWheelVel(1);
        velocities.y = gear_unit->GetWheelVel(2);
        velocities.z = gear_unit->GetWheelVel(3);

        return velocities;
    }

    return float3();
}

void jsbsim_wrapper_impl::show_sketch(double3 pos)
{
    uint idgroup = sketch->create_group();
    sketch->set_xray_mode(true);

    sketch->set_position(pos);
    sketch->set_rotation(geomob->get_rot());

    uint canvasid = sketch->create_canvas(idgroup, { 0,0,0 });
    sketch->make_canvas_active(canvasid);

    float3 x_axis = { 1,0,0 };
    float3 y_axis = { 0,1,0 };
    float3 z_axis = { 0,0,1 };

    sketch->set_color({ 255,0,0 });
    float3 offset = { 0, 0, 0 };
    sketch->draw_line(offset, true);
    sketch->draw_line(offset + x_axis);

    sketch->set_color({ 0,255,0 });
    sketch->draw_line(offset, true);
    sketch->draw_line(offset + y_axis);

    sketch->set_color({ 0,0,255 });
    sketch->draw_line(offset, true);
    sketch->draw_line(offset + z_axis);

    sketch->delete_group(idgroup);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
