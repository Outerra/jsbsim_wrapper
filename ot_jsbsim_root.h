#pragma once
#ifndef __JSBSIM_WRAPPER_ROOT_H__
#define __JSBSIM_WRAPPER_ROOT_H__

#include <comm/singleton.h>
#include <comm/ref.h>

#include "math/FGLocation.h"

namespace JSBSim {
    class FGFDMExec;
    class FGPropertyManager;
}

namespace ot {
    
class ground_callback;
class eng_interface;

class jsbsim_root
    : public policy_intrusive_base
{
public:
    static jsbsim_root* _inst;      //< singleton instance

protected:

    JSBSim::FGGroundCallback_ptr _gc;   //< ground callback for collision detection
	ref<JSBSim::FGFDMExec> _jsbexec;    //< root FGFDMExec
    uint _counter;                      //< FGFDMExec instance/ID counter
    eng_interface *_eng;

public:

    jsbsim_root(ot::eng_interface *eng);
    ~jsbsim_root();

    bool is_initialized() const { return _eng != 0; }

    JSBSim::FGPropertyManager* get_pm() const;
    uint* get_counter() { DASSERT(!_jsbexec.is_empty()); return &_counter; }
    double get_earth_radius() const;
    eng_interface* get_eng() const { DASSERT(_eng != 0); return _eng; }
    ground_callback* get_gc() const;
};

} // end of namaspace JSBSim

#endif // __JSBSIM_WRAPPER_ROOT_H__
