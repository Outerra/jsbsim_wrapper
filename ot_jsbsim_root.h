#pragma once
#ifndef __JSBSIM_WRAPPER_ROOT_H__
#define __JSBSIM_WRAPPER_ROOT_H__

#include <comm/singleton.h>
#include <comm/ref.h>
#include <fstream>

#include "math/FGLocation.h"

namespace JSBSim {
class FGFDMExec;
class FGPropertyManager;
class FGGroundCallback;
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

    std::shared_ptr<ot::ground_callback> _gc;
    //std::shared_ptr<JSBSim::FGFDMExec> _jsbexec;    //< root FGFDMExec
    uint _counter = 0;                              //< FGFDMExec instance/ID counter
    eng_interface* _eng = 0;

    std::ofstream _cout_buf;
    std::ofstream _cerr_buf;
    std::streambuf* _cout_buf_backup = nullptr;
    std::streambuf* _cerr_buf_backup = nullptr;
public:

    jsbsim_root(ot::eng_interface* eng);
    ~jsbsim_root();

    bool is_initialized() const { return _eng != 0; }

    uint* get_counter() { return &_counter; }

    eng_interface* get_eng() const { DASSERT(_eng != 0); return _eng; }
    std::shared_ptr<JSBSim::FGGroundCallback> get_gc() const;
};

} // end of namaspace JSBSim

#endif // __JSBSIM_WRAPPER_ROOT_H__
