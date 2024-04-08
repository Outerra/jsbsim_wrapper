#include "jsbsim_wrapper_impl.h"
#include "jsbsim_wrapper_api.h"
#include "ot_jsbsim_root.h"

#include <iostream>

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

extern "C" void JSBSIM_WRAPPER_API jsbsim_create_wrapper(iref<JSBSim::jsbsim_wrapper>& jsb, ot::eng_interface* eng, uint object_id)
{
    jsb.create(new JSBSim::jsbsim_wrapper_impl(eng, object_id));

    std::cout << "JSBSIM Wrapper created!" << std::endl;
}


/*JSBSim::jsbsim_wrapper::jsbsim_wrapper() {
    if (!jsbsim_wrapper::_rout) {
        jsbsim_wrapper::_rout = freopen("jsbsim_cout.log", "w", stdout);
        jsbsim_wrapper::_rerr = freopen("jsbsim_cerr.log", "w", stderr);
    }
}

JSBSim::jsbsim_wrapper::~jsbsim_wrapper() {

}
*/

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
