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


static ot::jsbsim_root* jsbroot(ot::eng_interface* eng)
{
    LOCAL_SINGLETON(ot::jsbsim_root) _root = new ot::jsbsim_root(eng);
    return _root.get();
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
jsbsim_wrapper_impl::jsbsim_wrapper_impl(ot::eng_interface* eng)
{
    _eng_ifc = eng;

    ot::jsbsim_root* root = jsbroot(eng);
    _jsbexec.reset(new JSBSim::FGFDMExec(root->get_gc().get()));

    _earth_radius = eng->get_earth_radius();

    DASSERT(_jsbexec.get() != 0);
    //_jsbexec->Setdt(integration_step);
    //_jsbexec->SetGroundCallback(ot::jsbsim_root::get().get_gc()); // set in the jsbsim_root!

    _jsbic = _jsbexec->GetIC().get();
    _inertial = _jsbexec->GetInertial().get();
    _atmosphere = _jsbexec->GetAtmosphere().get();
    _FCS = _jsbexec->GetFCS().get();
    _propulsion = _jsbexec->GetPropulsion().get();
    _massBalance = _jsbexec->GetMassBalance().get();
    _aircraft = _jsbexec->GetAircraft().get();
    _propagate = _jsbexec->GetPropagate().get();
    _auxiliary = _jsbexec->GetAuxiliary().get();
    _aerodynamics = _jsbexec->GetAerodynamics().get();
    _groundReactions = _jsbexec->GetGroundReactions().get();
    _props = _jsbexec->GetPropertyManager()->GetNode();

    auto top = _property_stack.add(1);
    top->node = _props;
    top->child_count = _props->nChildren();

    _jsbexec->Setdt(STEPs);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

jsbsim_wrapper_impl::~jsbsim_wrapper_impl()
{
    _jsbexec.reset();
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
            FGEngine* eng = _propulsion->GetEngine(i).get();
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

    const FGColumnVector3 accb = _propagate->GetTb2ec() * _jsbexec->GetAccelerations()->GetBodyAccel();
    _aircraft_data.acc_body = float3(accb(1) * F2M(), accb(2) * F2M(), accb(3) * F2M());

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
    float dist = _eng_ifc->elevation_over_terrain(
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
    auto& gnd = _aircraft_data.ground_reactions;
    gnd.BumpHeight = _groundReactions->GetBumpHeight();
    gnd.Bumpiness = _groundReactions->GetBumpiness();
    gnd.SteeringCmd = _groundReactions->GetDsCmd();
    gnd.GndreactMaximumForce = _groundReactions->GetMaximumForce();
    gnd.RollingFrictFactor = _groundReactions->GetRollingFFactor();
    gnd.SurfaceSolidStatus = _groundReactions->GetSolid();
    gnd.StaticFrictFactor = _groundReactions->GetStaticFFactor();
    gnd.WOW = _groundReactions->GetWOW();
    gnd.GndreactForces = { _groundReactions->GetForces().Entry(1), _groundReactions->GetForces().Entry(2) ,_groundReactions->GetForces().Entry(3) };
    gnd.GndreactMoments = { _groundReactions->GetMoments().Entry(1), _groundReactions->GetMoments().Entry(2) , _groundReactions->GetMoments().Entry(3) };

    uint NumCP = get_num_contact_points(false);
    _aircraft_data.Gears.realloc(NumCP);

    for (uint i = 0; i < NumCP; i++)
    {
        ot::aircraft_data::Gear& gear = _aircraft_data.Gears[i];
        JSBSim::FGLGear* wheel = _groundReactions->GetGearUnit(i).get();

        gear.SteerType = get_steer_type(i);
        gear.ContactPointPos = get_contact_point_pos(i, false);
        gear.AxisVelocities = get_wheel_axis_vel(i);
        gear.GearWOW = wheel->GetWOW();
        gear.GearAnglesToBody = { wheel->GetAnglesToBody().Entry(1),wheel->GetAnglesToBody().Entry(2) ,wheel->GetAnglesToBody().Entry(3) };
        gear.GearBodyForces = { wheel->GetBodyForces().Entry(1),wheel->GetBodyForces().Entry(2) ,wheel->GetBodyForces().Entry(3) };
        gear.GearBrakeGroup = wheel->GetBrakeGroup();
        gear.GearCompForce = wheel->GetCompForce();
        gear.GearCompLength = wheel->GetCompLen();
        gear.GearPos = wheel->GetGearUnitPos();
        gear.LocalGear = { wheel->GetLocalGear().Entry(1),wheel->GetLocalGear().Entry(2) ,wheel->GetLocalGear().Entry(3) };
        gear.GearMoments = { wheel->GetMoments().Entry(1),wheel->GetMoments().Entry(2) ,wheel->GetMoments().Entry(3) };
        gear.GearPitch = wheel->GetPitch();
        gear.GearYaw = wheel->GetYaw();
        gear.GearisRetractable = wheel->GetRetractable();
        gear.GearStaticFrictCoeff = wheel->GetstaticFCoeff();
        gear.GearSteerAngleDeg = wheel->GetSteerAngleDeg();
        gear.GearSteerNorm = wheel->GetSteerNorm();
        gear.GearTransformType = wheel->GetTransformType();
        gear.GearRollForce = wheel->GetWheelRollForce();
        gear.GearRollVel = wheel->GetWheelRollVel();
        gear.GearSideForce = wheel->GetWheelSideForce();
        gear.GearSideVel = wheel->GetWheelSideVel();
        gear.GearSlipAngle = wheel->GetWheelSlipAngle();
        gear.GearVelX = wheel->GetWheelVel(1);
        gear.GearVelY = wheel->GetWheelVel(2);
        gear.GearVelZ = wheel->GetWheelVel(3);
        gear.GearisBogey = wheel->IsBogey();

        if (gear.WheelName.is_empty())
        {
            gear.WheelName = wheel->GetName().c_str();
        }
    }

    //////////// FCS
    auto& fcs = _aircraft_data.fcs;
    fcs.LeftAileronPosRad = _FCS->GetDaLPos();
    fcs.RightAileronPosRad = _FCS->GetDaRPos();
    fcs.ElevatorPosRad = _FCS->GetDePos();
    fcs.FlapsPosRad = _FCS->GetDfPos();
    fcs.RudderPosRad = _FCS->GetDrPos();
    fcs.SpeedBrakePosRad = _FCS->GetDsbPos();
    fcs.SpoilerPosRad = _FCS->GetDspPos();
    fcs.RightBrakePos = _FCS->GetRBrake();
    fcs.LeftBrakePos = _FCS->GetLBrake();
    fcs.CenterBrakePos = _FCS->GetCBrake();
    // tailhook position  0 - up,  1 - down
    fcs.TailhookPos = _FCS->GetTailhookPos();
    //wing fold position 0 - unfolded, 1 - folded
    fcs.WingFoldPos = _FCS->GetWingFoldPos();
    fcs.TrimStatus = _FCS->GetTrimStatus();
    fcs.ChannelDeltaT = _FCS->GetChannelDeltaT();
    fcs.AileronCmd = _FCS->GetDaCmd();
    fcs.ElevatorCmd = _FCS->GetDeCmd();
    fcs.FlapsCmd = _FCS->GetDfCmd();
    fcs.RudderCmd = _FCS->GetDrCmd();
    fcs.SpeedbrakeCmd = _FCS->GetDsbCmd();
    fcs.SteeringCmd = _FCS->GetDsCmd();
    fcs.SpoilerCmd = _FCS->GetDspCmd();
    fcs.GearCmd = _FCS->GetGearCmd();
    fcs.PitchTrimCmd = _FCS->GetPitchTrimCmd();
    fcs.RollTrimCmd = _FCS->GetRollTrimCmd();
    fcs.YawTrimCmd = _FCS->GetYawTrimCmd();

    //////////// Propulsion & Tanks & Engines + EngineFCS
    auto& prop = _aircraft_data.propulsion;
    prop.GetActiveEngine = _propulsion->GetActiveEngine();
    prop.FuelFreezeStatus = _propulsion->GetFuelFreeze();
    //num of fuel tanks currently actively supplying fuel
    //prop.ActiveFuelTanks = _propulsion->GetnumSelectedFuelTanks();
    //num of fuel tanks currently actively supplying oxidizer
    //prop.ActiveOxiTanks = _propulsion->GetnumSelectedOxiTanks();
    prop.NumTanks = _propulsion->GetNumTanks();
    prop.NumEngines = _propulsion->GetNumEngines();
    prop.TanksWeight = _propulsion->GetTanksWeight();
    prop.PropForces = { _propulsion->GetForces().Entry(1), _propulsion->GetForces().Entry(2), _propulsion->GetForces().Entry(3) };
    prop.PropMoments = { _propulsion->GetMoments().Entry(1), _propulsion->GetMoments().Entry(2), _propulsion->GetMoments().Entry(3) };
    prop.TanksMoment = { _propulsion->GetTanksMoment().Entry(1), _propulsion->GetTanksMoment().Entry(2), _propulsion->GetTanksMoment().Entry(3) };

    //Tanks
    _aircraft_data.Tanks.realloc(prop.NumTanks);

    for (int i = 0; i < prop.NumTanks; i++)
    {
        ot::aircraft_data::Tank& tank = _aircraft_data.Tanks[i];
        JSBSim::FGTank* _tank = _propulsion->GetTank(i).get();

        tank.TankCapacityLbs = _tank->GetCapacity();
        tank.TankCapacityGal = _tank->GetCapacityGallons();
        tank.TankContentsLbs = _tank->GetContents();
        tank.TankContentsGal = _tank->GetContentsGallons();
        tank.TankDensity = _tank->GetDensity();
        tank.TankExternalFlow = _tank->GetExternalFlow();
        //fill level in percents 0-100
        tank.TankFillLvlPct = _tank->GetPctFull();
        tank.TankPriority = _tank->GetPriority();
        tank.TankSupplyStatus = _tank->GetSelected();
        tank.TankStandpipe = _tank->GetStandpipe();
        tank.TankTempDegF = _tank->GetTemperature();
        tank.TankTempDegC = _tank->GetTemperature_degC();
        //0-undefined, 1-fuel, 2-oxidizer
        tank.TankType = _tank->GetType();
        tank.TankXYZ = { _tank->GetXYZ().Entry(1), _tank->GetXYZ().Entry(2), _tank->GetXYZ().Entry(3) };
    }

    //Engines + EngineFCS
    _aircraft_data.Engines.realloc(prop.NumEngines);
    _aircraft_data.EnginesFCS.realloc(prop.NumEngines);

    for (int i = 0; i < prop.NumEngines; i++)
    {
        ot::aircraft_data::Engine& Enginedata = _aircraft_data.Engines[i];
        JSBSim::FGEngine* _engine = _propulsion->GetEngine(i).get();
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
    auto& propagate = _aircraft_data.propagate;
    propagate.AltitudeASL = _propagate->GetAltitudeASL();
    propagate.AltitudeASLm = _propagate->GetAltitudeASLmeters();
    //retrieves sine of vehicle Euler angle component (Phi - 1, Theta - 2 or Psi - 3)
    propagate.SinEulerPhi = _propagate->GetSinEuler(1);
    propagate.SinEulerTheta = _propagate->GetSinEuler(2);
    propagate.SinEulerPsi = _propagate->GetSinEuler(3);
    //retrieves cosine of vehicle Euler angle component (Phi - 1, Theta - 2 or Psi - 3)
    propagate.CosEulerPhi = _propagate->GetCosEuler(1);
    propagate.CosEulerTheta = _propagate->GetCosEuler(2);
    propagate.CosEulerPsi = _propagate->GetCosEuler(3);
    propagate.DistanceAGL = _propagate->GetDistanceAGL();
    propagate.DistanceAGLkm = _propagate->GetDistanceAGLKm();
    propagate.EarthPosAngle = _propagate->GetEarthPositionAngle();
    propagate.EarthPosAngleDeg = _propagate->GetEarthPositionAngleDeg();
    propagate.ECEFvel = { _propagate->GetECEFVelocity().Entry(1), _propagate->GetECEFVelocity().Entry(2) , _propagate->GetECEFVelocity().Entry(3) };
    propagate.Euler = { _propagate->GetEuler().Entry(1),_propagate->GetEuler().Entry(2) ,_propagate->GetEuler().Entry(3) };
    propagate.EulerDeg = { _propagate->GetEulerDeg().Entry(1),_propagate->GetEulerDeg().Entry(2) ,_propagate->GetEulerDeg().Entry(3) };
    propagate.GeodAlt = _propagate->GetGeodeticAltitude();
    propagate.GeodAltKm = _propagate->GetGeodeticAltitudeKm();
    propagate.GeodLatDeg = _propagate->GetGeodLatitudeDeg();
    propagate.GeodLatRad = _propagate->GetGeodLatitudeRad();
    propagate.CurrentAltRate = _propagate->Gethdot();
    propagate.InertialPos = { _propagate->GetInertialPosition().Entry(1), _propagate->GetInertialPosition().Entry(2), _propagate->GetInertialPosition().Entry(3) };
    propagate.InertialVel = { _propagate->GetInertialVelocity().Entry(1), _propagate->GetInertialVelocity().Entry(2), _propagate->GetInertialVelocity().Entry(3) };
    propagate.InertialVelMag = _propagate->GetInertialVelocityMagnitude();
    propagate.Latitude = _propagate->GetLatitude();
    propagate.LatitudeDeg = _propagate->GetLatitudeDeg();
    propagate.LocalTerrainRadius = _propagate->GetLocalTerrainRadius();
    propagate.PropagateLoc = { _propagate->GetLocation().Entry(1), _propagate->GetLocation().Entry(2), _propagate->GetLocation().Entry(3) };
    propagate.Longtitude = _propagate->GetLongitude();
    propagate.LongtitudeDeg = _propagate->GetLongitudeDeg();
    propagate.NEDvelMagn = _propagate->GetNEDVelocityMagnitude();
    propagate.PropagatePQR = { _propagate->GetPQR().Entry(1), _propagate->GetPQR().Entry(2), _propagate->GetPQR().Entry(3) };
    propagate.PropagatePQRi = { _propagate->GetPQRi().Entry(1), _propagate->GetPQRi().Entry(2), _propagate->GetPQRi().Entry(3) };
    propagate.PropagateRadius = _propagate->GetRadius();
    propagate.TerrainAngularVel = { _propagate->GetTerrainAngularVelocity().Entry(1),_propagate->GetTerrainAngularVelocity().Entry(2), _propagate->GetTerrainAngularVelocity().Entry(3) };
    propagate.TerrainElevation = _propagate->GetTerrainElevation();
    propagate.TerrainVel = { _propagate->GetTerrainVelocity().Entry(1), _propagate->GetTerrainVelocity().Entry(2), _propagate->GetTerrainVelocity().Entry(3) };
    propagate.PropagateUVW = { _propagate->GetUVW().Entry(1), _propagate->GetUVW().Entry(2), _propagate->GetUVW().Entry(3) };
    propagate.PropagateVel = { _propagate->GetVel().Entry(1), _propagate->GetVel().Entry(2), _propagate->GetVel().Entry(3) };


    //////////// atmosphere
    auto& atmo = _aircraft_data.atmosphere;
    atmo.AtmosDensity = _atmosphere->GetDensity();
    atmo.AbsoluteViscosity = _atmosphere->GetAbsoluteViscosity();
    atmo.DensityAltitude = _atmosphere->GetDensityAltitude();
    /// Returns the ratio of at-altitude density over the sea level value.
    atmo.DensityRatio = _atmosphere->GetDensityRatio();
    /// Returns the sea level density in slugs/ft^3
    atmo.DensitySL = _atmosphere->GetDensitySL();
    atmo.KinematicViscosity = _atmosphere->GetKinematicViscosity();
    //can take param double altitude - returns pressure at specified altitude in psf
    //altitude is in ft
    atmo.AtmosPressure = _atmosphere->GetPressure(propagate.AltitudeASL);
    atmo.PressureAltitude = _atmosphere->GetPressureAltitude();
    atmo.PressureRatio = _atmosphere->GetPressureRatio();
    atmo.PressureSL = _atmosphere->GetPressureSL();
    //can take param double altitude(ft?) - returns speed of sound ft/sec at given altitude in ft
    atmo.SoundSpeed = _atmosphere->GetSoundSpeed(propagate.AltitudeASL);
    /// Returns the ratio of at-altitude sound speed over the sea level value.
    atmo.SoundSpeedRatio = _atmosphere->GetSoundSpeedRatio();
    /// Returns the sea level speed of sound in ft/sec.
    atmo.SoundSpeedSL = _atmosphere->GetSoundSpeedSL();
    /// Returns the actual, modeled temperature at the current altitude in degrees Rankine.
    atmo.TemperatureRa = _atmosphere->GetTemperature(propagate.AltitudeASL);
    /// Returns the ratio at-current-altitude temperature as modeled over the sea level value
    atmo.TemperatureRatio = _atmosphere->GetTemperatureRatio(propagate.AltitudeASL);
    /// Returns the actual, modeled sea level temperature in degrees Rankine.
    atmo.TemperatureSLRa = _atmosphere->GetTemperatureSL();


    //////////// accelerations
    JSBSim::FGAccelerations* _accelerations = _jsbexec->GetAccelerations().get();

    auto& acc = _aircraft_data.accelerations;
    acc.GravAccelMagnitude = _accelerations->GetGravAccelMagnitude();

    //retrieves acceleration resulting from applied forces
    acc.BodyAccel = { _accelerations->GetBodyAccel().Entry(1), _accelerations->GetBodyAccel().Entry(2), _accelerations->GetBodyAccel().Entry(3) };
    //retrieves the total forces applied on the body
    acc.AccelForces = { _accelerations->GetForces().Entry(1), _accelerations->GetForces().Entry(2), _accelerations->GetForces().Entry(3) };
    //acc.GravAccel = { _accelerations->GetGravAccel().Entry(1), _accelerations->GetGravAccel().Entry(2), _accelerations->GetGravAccel().Entry(3) };
    //retrieves ground forces applied on the body
    acc.GroundForces = { _accelerations->GetGroundForces().Entry(1), _accelerations->GetGroundForces().Entry(2), _accelerations->GetGroundForces().Entry(3) };
    //retrieves ground moments applied on the body
    acc.GroundMoments = { _accelerations->GetGroundMoments().Entry(1), _accelerations->GetGroundMoments().Entry(2), _accelerations->GetGroundMoments().Entry(3) };
    //retrieves a component of the total moments applied on the body
    acc.AccelMoments = { _accelerations->GetMoments().Entry(1), _accelerations->GetMoments().Entry(2), _accelerations->GetMoments().Entry(3) };
    //retrieves the body axis angular acceleration vector
    acc.AccelPQRdot = { _accelerations->GetPQRdot().Entry(1),_accelerations->GetPQRdot().Entry(2), _accelerations->GetPQRdot().Entry(3) };
    //retrieves the body axis angular acceleration vector in ECI frame
    acc.AccelPQRidot = { _accelerations->GetPQRidot().Entry(1), _accelerations->GetPQRidot().Entry(2),_accelerations->GetPQRidot().Entry(3) };
    //retrieves the body axis acceleration
    acc.AccelUVWdot = { _accelerations->GetUVWdot().Entry(1),_accelerations->GetUVWdot().Entry(2), _accelerations->GetUVWdot().Entry(3) };
    //retrieves the body axis acceleration in the ECI frame
    acc.AccelUVWidot = { _accelerations->GetUVWidot().Entry(1), _accelerations->GetUVWidot().Entry(2), _accelerations->GetUVWidot().Entry(3) };
    //retrieves the weight applied on the body
    acc.Weight = { _accelerations->GetWeight().Entry(1), _accelerations->GetWeight().Entry(2), _accelerations->GetWeight().Entry(3) };


    //////////// MassBalance
    auto& mass = _aircraft_data.mass_balance;
    mass.EmptyWeight = _massBalance->GetEmptyWeight();
    mass.Mass = _massBalance->GetMass();
    mass.PointMassWeight = _massBalance->GetTotalPointMassWeight();
    mass.Weight = _massBalance->GetWeight();
    //can take parameter int axis
    mass.DeltaXYZcg = { _massBalance->GetDeltaXYZcg().Entry(1), _massBalance->GetDeltaXYZcg().Entry(2), _massBalance->GetDeltaXYZcg().Entry(3) };
    //can take parameter int axis
    mass.XYZcg = { _massBalance->GetXYZcg().Entry(1), _massBalance->GetXYZcg().Entry(2), _massBalance->GetXYZcg().Entry(3) };
    mass.PointMassMoment = { _massBalance->GetPointMassMoment().Entry(1), _massBalance->GetPointMassMoment().Entry(2), _massBalance->GetPointMassMoment().Entry(3) };


    //////////// Aerodynamics
    auto& aero = _aircraft_data.aerodynamics;
    aero.AlphaCLmax = _aerodynamics->GetAlphaCLMax();
    aero.AlphaCLmin = _aerodynamics->GetAlphaCLMin();
    aero.AlphaW = _aerodynamics->GetAlphaW();
    aero.BI2vel = _aerodynamics->GetBI2Vel();
    aero.CI2vel = _aerodynamics->GetCI2Vel();
    //gets square of the lift coeficient
    aero.LiftCoefSq = _aerodynamics->GetClSquared();
    //gets aerodynamic vector
    aero.AerodynForces = { _aerodynamics->GetForces().Entry(1),_aerodynamics->GetForces().Entry(2) ,_aerodynamics->GetForces().Entry(3) };
    aero.HysteresisParm = _aerodynamics->GetHysteresisParm();
    //gets lift over drag ratio
    aero.LoD = _aerodynamics->GetLoD();
    //gets aerodynamic moment vector about the CG - total or for given axis
    aero.AerodynMoments = { _aerodynamics->GetMoments().Entry(1),_aerodynamics->GetMoments().Entry(2) ,_aerodynamics->GetMoments().Entry(3) };
    //gets aerodynamic moment vector about the moment reference center - total or for given axis
    aero.AerodynMomentsMRC = { _aerodynamics->GetMomentsMRC().Entry(1),_aerodynamics->GetMomentsMRC().Entry(2) ,_aerodynamics->GetMomentsMRC().Entry(3) };
    aero.StallWarn = _aerodynamics->GetStallWarn();
    //gets aerodynamic forces in the wind axes
    aero.ForcesWindAxes = { _aerodynamics->GetvFw().Entry(1),_aerodynamics->GetvFw().Entry(2) ,_aerodynamics->GetvFw().Entry(3) };

/*
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
*/

    //////////// Aircraft
    auto& aircraft = _aircraft_data.aircraft;
    aircraft.Cbar = _aircraft->Getcbar();
    aircraft.HtailArea = _aircraft->GetHTailArea();
    aircraft.HtailArm = _aircraft->GetHTailArm();
    aircraft.LbarH = _aircraft->Getlbarh();
    aircraft.Lbarv = _aircraft->Getlbarv();
    //Aircraftdata.PitotAngle = _aircraft->GetPitotAngle();
    aircraft.VbarH = _aircraft->Getvbarh();
    aircraft.VbarV = _aircraft->Getvbarv();
    aircraft.VtailArea = _aircraft->GetVTailArea();
    aircraft.VtailArm = _aircraft->GetVTailArm();
    aircraft.WingArea = _aircraft->GetWingArea();
    aircraft.WingIncidence = _aircraft->GetWingIncidence();
    aircraft.WingIncidenceDeg = _aircraft->GetWingIncidenceDeg();
    aircraft.WingSpan = _aircraft->GetWingSpan();
    aircraft.AircraftForces = { _aircraft->GetForces().Entry(1),_aircraft->GetForces().Entry(2) ,_aircraft->GetForces().Entry(3) };
    aircraft.AircraftMoments = { _aircraft->GetMoments().Entry(1),_aircraft->GetMoments().Entry(2) ,_aircraft->GetMoments().Entry(3) };
    aircraft.AircraftXYZep = { _aircraft->GetXYZep().Entry(1),_aircraft->GetXYZep().Entry(2) ,_aircraft->GetXYZep().Entry(3) };
    aircraft.AircraftXYZrp = { _aircraft->GetXYZrp().Entry(1),_aircraft->GetXYZrp().Entry(2) ,_aircraft->GetXYZrp().Entry(3) };
    aircraft.AircraftXYRvrp = { _aircraft->GetXYZvrp().Entry(1),_aircraft->GetXYZvrp().Entry(2) ,_aircraft->GetXYZvrp().Entry(3) };

    if (aircraft.AircraftName.is_empty())
    {
        aircraft.AircraftName = _aircraft->GetAircraftName().c_str();
    }


    //////////// Auxiliary
    auto& aux = _aircraft_data.auxiliary;
    //aux.DayOfYear = _auxiliary->GetDayOfYear();
    aux.AuxAdot = _auxiliary->Getadot();
    aux.AuxAlpha = _auxiliary->Getalpha();
    aux.AuxBdot = _auxiliary->Getbdot();
    aux.AuxBeta = _auxiliary->Getbeta();
    //aux.CrossWind = _auxiliary->GetCrossWind();
    aux.DistanceRelativePos = _auxiliary->GetDistanceRelativePosition();
    aux.AuxGamma = _auxiliary->GetGamma();
    aux.GroundTrack = _auxiliary->GetGroundTrack();
    //aux.HeadWind = _auxiliary->GetHeadWind();
    aux.HOverBCG = _auxiliary->GetHOverBCG();
    aux.HOverBMAC = _auxiliary->GetHOverBMAC();
    //aux.hVRP = _auxiliary->GethVRP();
    aux.LatitudeRelatPos = _auxiliary->GetLatitudeRelativePosition();
    aux.LongtitudeRelatPos = _auxiliary->GetLongitudeRelativePosition();
    aux.Mach = _auxiliary->GetMach();
    aux.MachU = _auxiliary->GetMachU();
    aux.MagBeta = _auxiliary->GetMagBeta();
    aux.Nlf = _auxiliary->GetNlf();
    aux.Ny = _auxiliary->GetNy();
    aux.Nz = _auxiliary->GetNz();
    aux.Qbar = _auxiliary->Getqbar();
    aux.QbarUV = _auxiliary->GetqbarUV();
    aux.QbarUW = _auxiliary->GetqbarUW();
    aux.ReynoldsNum = _auxiliary->GetReynoldsNumber();
    //aux.SecondsInDay = _auxiliary->GetSecondsInDay();
    aux.TAT_C = _auxiliary->GetTAT_C();
    aux.TotalPressure = _auxiliary->GetTotalPressure();
    aux.TotalTemp = _auxiliary->GetTotalTemperature();
    aux.VcalibratedFPS = _auxiliary->GetVcalibratedFPS();
    aux.VcalibratedKTS = _auxiliary->GetVcalibratedKTS();
    aux.VequivalentFPS = _auxiliary->GetVequivalentFPS();
    aux.VequivalentKTS = _auxiliary->GetVequivalentKTS();
    aux.Vground = _auxiliary->GetVground();
    aux.Vt = _auxiliary->GetVt();
    aux.VtrueFPS = _auxiliary->GetVtrueFPS();
    aux.VtrueKTS = _auxiliary->GetVtrueKTS();
    aux.AeroPQR = { _auxiliary->GetAeroPQR().Entry(1),_auxiliary->GetAeroPQR().Entry(2) ,_auxiliary->GetAeroPQR().Entry(3) };
    aux.EulerRates = { _auxiliary->GetEulerRates().Entry(1),_auxiliary->GetEulerRates().Entry(2) ,_auxiliary->GetEulerRates().Entry(3) };
    aux.LocVRP = { _auxiliary->GetLocationVRP().Entry(1),_auxiliary->GetLocationVRP().Entry(2) ,_auxiliary->GetLocationVRP().Entry(3) };
    aux.Ncg = { _auxiliary->GetNcg().Entry(1), _auxiliary->GetNcg().Entry(2), _auxiliary->GetNcg().Entry(3) };
    aux.Npilot = { _auxiliary->GetNpilot().Entry(1),_auxiliary->GetNpilot().Entry(2) ,_auxiliary->GetNpilot().Entry(3) };
    aux.Nwcg = { _auxiliary->GetNwcg().Entry(1),_auxiliary->GetNwcg().Entry(2) ,_auxiliary->GetNwcg().Entry(3) };
    aux.PilotAccel = { _auxiliary->GetPilotAccel().Entry(1),_auxiliary->GetPilotAccel().Entry(2) ,_auxiliary->GetPilotAccel().Entry(3) };
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

void jsbsim_wrapper_impl::set_gear_brakes(const float3& brakes)
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


void jsbsim_wrapper_impl::root()
{
    //_property_stack.resize(1)[0].child_index = 0;
}

bool jsbsim_wrapper_impl::prop_first_child()
{
    auto* cur = _property_stack.last();
    if (cur->child_count == 0)
        return false;

    SGPropertyNode* node = cur->node->getChild(0);
    if (!node)
        return false;

    auto* next = _property_stack.add();
    next->node = node;
    next->child_index = 0;
    next->child_count = node->nChildren();

    cur = next - 1;
    cur->child_index = 0;

    return true;
}

bool jsbsim_wrapper_impl::prop_next_sibling()
{
    if (_property_stack.size() <= 1)
        return false;

    auto* cur = _property_stack.last();
    auto* par = cur - 1;

    uint nextid = par->child_index + 1;
    if (nextid >= par->child_count)
        return false; //no more

    SGPropertyNode* node = par->node->getChild(nextid);
    if (!node)
        return false;

    cur->node = node;
    cur->child_count = node->nChildren();
    par->child_index = nextid;

    return true;
}

bool jsbsim_wrapper_impl::prop_parent()
{
    if (_property_stack.size() <= 1)
        return false;

    _property_stack.pop();
    _property_stack.last()->child_index = 0;
    return true;
}

coid::token jsbsim_wrapper_impl::prop_name() const
{
    const std::string& name = _property_stack.last()->node->getNameString();
    return coid::token(name.c_str(), name.length());
}

jsbsim_prop_type jsbsim_wrapper_impl::prop_type() const
{
    return (jsbsim_prop_type)_property_stack.last()->node->getType();
}

int jsbsim_wrapper_impl::prop_index() const
{
    return _property_stack.last()->node->getIndex();
}

double jsbsim_wrapper_impl::prop_get_double_value() const
{
    return _property_stack.last()->node->getDoubleValue();
}

bool jsbsim_wrapper_impl::prop_get_bool_value() const
{
    return _property_stack.last()->node->getBoolValue();
}

bool jsbsim_wrapper_impl::prop_set_double_value(double value)
{
    return _property_stack.last()->node->setDoubleValue(value);
}

bool jsbsim_wrapper_impl::prop_set_bool_value(bool value)
{
    return _property_stack.last()->node->setBoolValue(value);
}

bool jsbsim_wrapper_impl::prop_add_child(const coid::token& name, int index)
{
    auto* cur = _property_stack.last();
    SGPropertyNode* node = cur->node->addChild(std::string(name.ptr(), name.len()), index);
    if (!node)
        return false;

    auto* next = _property_stack.add();
    cur = next - 1;
    next->node = node;
    next->child_index = 0;
    next->child_count = 0;

    cur->child_count = node->nChildren();
    cur->child_index = cur->child_count - 1;

    return true;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void jsbsim_wrapper_impl::set_gear(const bool down)
{
    _FCS->SetGearCmd(down ? 1 : 0);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//returned steer types: 0 - steerable, 1 - fix, 2 - caster
uint jsbsim_wrapper_impl::get_steer_type(uint wheel_id)
{
    int steer_type;
    const uint n = _groundReactions->GetNumGearUnits();

    if (wheel_id < n)
    {
        steer_type = _groundReactions->GetGearUnit(wheel_id)->GetSteerType();
        return steer_type;
    }

    return uint();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//If "gearsonly" is true, return only gear/bogey contact points, otherwise return all contact points ( "BOGEY" (on gears) and "STRUCTURE" (other contact points, e.g. wing tip, nose etc.) )
uint jsbsim_wrapper_impl::get_num_contact_points(bool gearsonly)
{
    int contact_point_count = _groundReactions->GetNumGearUnits();

    if (gearsonly)
    {
        int gearcount = 0;

        for (int i = 0; i < contact_point_count; i++)
        {
            if (_groundReactions->GetGearUnit(i)->IsBogey())
            {
                gearcount++;
            }
        }
        return gearcount;
    }
    else
    {
        return contact_point_count;
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Get model position of contact points in meters
//The position is relative to VRP point
//If "gearsonly" is true, return only gear/bogey contact points, otherwise return all contact points ( "BOGEY" (on gears) and "STRUCTURE" (other contact points, e.g. wing tip, nose etc.) )
float3 jsbsim_wrapper_impl::get_contact_point_pos(const uint idx, bool gearsonly)
{
    const uint n = _groundReactions->GetNumGearUnits();

    if (idx >= n)
    {
        return float3();
    }

    FGLGear* const gear = _groundReactions->GetGearUnit(idx).get();

    if (!gear)
    {
        return float3();
    }

    if (gearsonly && !gear->IsBogey())
    {
        return float3();
    }

    //VRP point should now be the 3D model 0,0,0
    auto& aircraft = _aircraft_data.aircraft;
    float3 vrp_point = {aircraft.AircraftXYRvrp.y, -aircraft.AircraftXYRvrp.x , aircraft.AircraftXYRvrp.z };
    vrp_point *= 0.0254f;

    // GetLocation returns position in inch
    float3 gearloc = { gear->GetLocationY(), -gear->GetLocationX(), gear->GetLocationZ() };
    //from inch to meters
    gearloc *= 0.0254f;

    return (gearloc - vrp_point);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

float3 jsbsim_wrapper_impl::get_wheel_axis_vel(uint wheel_id)
{
    const uint n = _groundReactions->GetNumGearUnits();

    if (wheel_id < n)
    {
        JSBSim::FGLGear* gear_unit = _groundReactions->GetGearUnit(wheel_id).get();

        float3 velocities;
        velocities.x = gear_unit->GetWheelVel(1);
        velocities.y = gear_unit->GetWheelVel(2);
        velocities.z = gear_unit->GetWheelVel(3);

        return velocities;
    }

    return float3();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
