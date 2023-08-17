#include "ot_jsbsim_root.h"
#include "ot_ground_callback.h"
#include "ot_eng_interface.h"

#include <FGFDMExec.h>

ot::jsbsim_root* ot::jsbsim_root::_inst = 0;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

ot::jsbsim_root::jsbsim_root(ot::eng_interface* eng)
    : _gc(new ground_callback(eng))
    , _jsbexec(new JSBSim::FGFDMExec(_gc))
    , _counter(0)
    , _eng(eng)
{
    ot::jsbsim_root::_inst = this;
    _cout_buf.open("jsbsim_cout.log", std::fstream::trunc);
    _cerr_buf.open("jsbsim_cerr.log", std::fstream::trunc);

    if (_cout_buf.is_open() && _cerr_buf.is_open()) {
        _cout_buf_backup = std::cout.rdbuf();
        _cerr_buf_backup = std::cerr.rdbuf();

        std::cout.rdbuf(_cout_buf.rdbuf());
        std::cerr.rdbuf(_cerr_buf.rdbuf());

        JSBSim::FGFDMExec::RedirectStdOutput(&_cout_buf, &_cerr_buf);
    }
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

ot::jsbsim_root::~jsbsim_root()
{
    ot::jsbsim_root::_inst = 0;
    _jsbexec.release();
    _gc = 0;
    _eng = 0;
    JSBSim::FGLocation::SetGroundCallback(0);

    if (_cout_buf.is_open()) {
        _cout_buf.close();
    }

    if (_cerr_buf.is_open()) {
        _cerr_buf.close();
    }

    if (_cout_buf_backup)
        std::cout.rdbuf(_cout_buf_backup);
    if (_cerr_buf_backup)
        std::cerr.rdbuf(_cerr_buf_backup);

}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

JSBSim::FGPropertyManager* ot::jsbsim_root::get_pm() const
{
    DASSERT(!_jsbexec.is_empty());
    return _jsbexec->GetPropertyManager();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

double ot::jsbsim_root::get_earth_radius() const
{
    DASSERT(_eng != 0);
    return _eng->get_earth_radius();
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

ot::ground_callback* ot::jsbsim_root::get_gc() const
{
    DASSERT(_gc.valid());
    return static_cast<ground_callback*>(_gc.ptr());
}