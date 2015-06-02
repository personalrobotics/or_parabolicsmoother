#ifndef PARABOLICSMOOTHER_H_
#define PARABOLICSMOOTHER_H_
#include <openrave/openrave.h>
#include "DynamicPath.h"
#include "ParabolicSmootherParameters.h"

namespace or_parabolicsmoother {

const int DEFAULT_MAX_ITERATIONS = 25;

class ORFeasibilityChecker : public ParabolicRamp::FeasibilityCheckerBase {
public:
    ORFeasibilityChecker(
        OpenRAVE::EnvironmentBasePtr const &env,
        OpenRAVE::PlannerBase::PlannerParametersConstPtr const &params
    );

    virtual bool ConfigFeasible(ParabolicRamp::Vector const &x);
    virtual bool SegmentFeasible(ParabolicRamp::Vector const &a,
                                 ParabolicRamp::Vector const &b);

private:
    OpenRAVE::EnvironmentBasePtr env_;
    OpenRAVE::PlannerBase::PlannerParametersConstPtr params_;
};


class ParabolicSmoother : public OpenRAVE::PlannerBase {
public:
    ParabolicSmoother(OpenRAVE::EnvironmentBasePtr penv);

    virtual bool InitPlan(OpenRAVE::RobotBasePtr robot,
                          PlannerParametersConstPtr params);
    virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream &input);

    virtual OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr ptraj);

    virtual PlannerParametersConstPtr GetParameters () const;

private:
    ParabolicSmootherParametersPtr parameters_;
};


}

#endif
