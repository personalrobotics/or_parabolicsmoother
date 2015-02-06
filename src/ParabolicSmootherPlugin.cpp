#include <boost/make_shared.hpp>
#include <openrave/plugin.h>
#include "ParabolicSmoother.h"

using namespace OpenRAVE;
using or_parabolicsmoother::ParabolicSmoother;

InterfaceBasePtr CreateInterfaceValidated(
        InterfaceType type, std::string const &interfacename,
        std::istream &sinput, EnvironmentBasePtr penv)
{
    if (type == PT_Planner && interfacename == "parabolicsmootherworking") {
        return boost::make_shared<ParabolicSmoother>(penv);
    } else {
        return InterfaceBasePtr();
    }
}

void GetPluginAttributesValidated(PLUGININFO &info)
{
    info.interfacenames[PT_Planner].push_back("ParabolicSmootherWorking");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
}
