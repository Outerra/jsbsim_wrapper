#include "ot_ground_callback.h"
#include "ot_eng_interface.h"

#include <math/FGLocation.h>
#include <math/FGColumnVector3.h>

#include <ot/glm/glm_ext.h>

using namespace JSBSim;

static double M2F() { return 3.2808399; }
static double F2M() { return 1.0/M2F(); }

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

ot::ground_callback::ground_callback(ot::eng_interface *eng)
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
	FGColumnVector3& w) const
{
	double3 pos(l.Entry(1), l.Entry(2), l.Entry(3));

	pos *= F2M();
    float mmaxdist = float(maxdist * F2M());

	v = w = FGColumnVector3(0,0,0);

	float3 normal;
	double3 surface;
    float3 vel;
    float3 rot_vel;

    float dist = _eng->elevation_over_terrain(
        pos,
        mmaxdist,
        &normal,
        &surface,
        &vel,
        &rot_vel);

	if(dist < mmaxdist) {
		surface *= M2F();

		c = FGColumnVector3(surface.x, surface.y, surface.z);
		n = FGColumnVector3(normal.x, normal.y, normal.z);
        v = FGColumnVector3(vel.x, vel.y, vel.z);
        w = FGColumnVector3(rot_vel.x, rot_vel.y, rot_vel.z);

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
