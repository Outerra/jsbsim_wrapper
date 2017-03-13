#include "jsbsim_wrapper_impl.h"
#include "jsbsim_wrapper_api.h"
#include "ot_jsbsim_root.h"

#include <stdio.h>

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

static FILE *rout=0, *rerr=0;

extern "C" void JSBSIM_WRAPPER_API jsbsim_create_wrapper(
    iref<JSBSim::jsbsim_wrapper> &jsb, ot::eng_interface *eng)
{
    if(!rout) {
        rout = freopen("cout.log", "w", stdout);
        rerr = freopen("cerr.log", "w", stderr);
    }

	jsb.create(new JSBSim::jsbsim_wrapper_impl(eng));
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
