#include "jsbsim_wrapper_impl.h"
//#include "ot_jsbsim_root.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

extern "C"
__declspec(dllexport) jsbsim_wrapper* jsbsim_create_wrapper(ot::eng_interface* eng)
{
    return new jsbsim_wrapper_impl(eng);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
