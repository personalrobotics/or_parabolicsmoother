#include <boost/make_shared.hpp>
#include <openrave/plugin.h>
#include "ParabolicSmoother.h"

using namespace OpenRAVE;
using namespace or_parabolicsmoother;

static std::string const kInterfaceName = "ParabolicSmootherWorking";

InterfaceBasePtr CreateInterfaceValidated(
        InterfaceType type, std::string const &interfacename,
        std::istream &sinput, EnvironmentBasePtr penv)
{
    if (type == PT_Planner && interfacename == kInterfaceName) {
        return boost::make_shared<ParabolicSmoother>(penv);
    } else {
        return InterfaceBasePtr();
    }
}

void GetPluginAttributesValidated(PLUGININFO &info)
{
    info.interfacenames[PT_Planner].push_back(kInterfaceName);
}

RAVE_PLUGIN_API void DestroyPlugin()
{
}
