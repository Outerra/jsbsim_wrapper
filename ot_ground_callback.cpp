#include "ot_ground_callback.h"
#include "ot_eng_interface.h"

#include <math/FGLocation.h>
#include <math/FGColumnVector3.h>

#include <ot/glm/glm_ext.h>

using namespace JSBSim;

static double M2F() { return 3.2808399; }
static double F2M() { return 1.0 / M2F(); }
static double KG2SLUG() { return 0.06852178; }
static double SLUG2KG() { return 1.0 / KG2SLUG(); }
static double KGMM2SLUGFF() { return KG2SLUG() * M2F() * M2F(); } // for conversion of moment of inertia from kg*m*m to slug*ft*ft
static double SLUGFF2KGMM() { return 1.0 / KGMM2SLUGFF(); }

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

ot::ground_callback::ground_callback(ot::eng_interface* eng)
    : _earth_radius(eng->get_earth_radius())
    , _water_radius(eng->get_earth_radius())
    , _eng(eng)
{}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

ot::ground_callback::~ground_callback() {}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

double ot::ground_callback::GetAltitude(const FGLocation& l) const
{
    return l.GetRadius() - GetTerrainGeoCentRadius(0, FGLocation());
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

double ot::ground_callback::GetAGLevel(
    double t,
    double maxdist,
    const FGLocation& l,
    FGLocation& c,
    FGColumnVector3& n,
    FGColumnVector3& v,
    FGColumnVector3& w,
    FGColumnVector3& ground_position,
    double& ground_mass_inverse,
    FGMatrix33& ground_j_inverse) const
{
    double3 pos(l.Entry(1), l.Entry(2), l.Entry(3));

    pos *= F2M();
    float mmaxdist = float(maxdist * F2M());

    v = w = FGColumnVector3(0, 0, 0);

    float3 normal;
    double3 surface;
    float3 vel;
    float3 rot_vel;
    double3 ground_pos;
    double3x3 ground_j_inv;

    float dist = _eng->elevation_over_terrain(
        pos,
        mmaxdist,
        &normal,
        &surface,
        &vel,
        &rot_vel,
        &ground_pos,
        &ground_mass_inverse,
        &ground_j_inv);

    if (dist < mmaxdist) {
        surface *= M2F();
        vel *= M2F();
        ground_pos *= M2F();
        ground_mass_inverse *= SLUG2KG(); // convert to slugs. mass is inverted so 1.0/KG2SLUG must be used as multiplier. Therefore we use SLUG2KG.
        ground_j_inv *= SLUGFF2KGMM(); // convert to slug * ft *ft

        c = FGColumnVector3(surface.x, surface.y, surface.z);
        n = FGColumnVector3(normal.x, normal.y, normal.z);
        v = FGColumnVector3(vel.x, vel.y, vel.z);
        w = FGColumnVector3(rot_vel.x, rot_vel.y, rot_vel.z);
        ground_position = FGColumnVector3(ground_pos.x, ground_pos.y, ground_pos.z);
        ground_j_inverse = FGMatrix33(ground_j_inv[0][0], ground_j_inv[0][1], ground_j_inv[0][2],
            ground_j_inv[1][0], ground_j_inv[1][1], ground_j_inv[1][2],
            ground_j_inv[2][0], ground_j_inv[2][1], ground_j_inv[2][2]
        );

        return dist * M2F();
    }
    else {
        c.SetLatitude(l.GetLatitude());
        c.SetLongitude(l.GetLongitude());
        c.SetRadius(GetTerrainGeoCentRadius(0, FGLocation()));
        //TODO n=FGColumnVector3(normal.x, normal.y, normal.z);

        //_eng->write_log("elevation_over_terrain failed!");

        return maxdist;//GetAltitude(l);
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

double ot::ground_callback::GetTerrainGeoCentRadius(
    double t, const FGLocation& location) const
{
    return _earth_radius * M2F();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

double ot::ground_callback::GetSeaLevelRadius(const FGLocation& location) const
{
    return _earth_radius * M2F();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
