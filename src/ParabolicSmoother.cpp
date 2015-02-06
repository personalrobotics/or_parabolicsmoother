#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <openrave/planningutils.h>
#include "ParabolicSmoother.h"

using boost::make_shared;
using OpenRAVE::EnvironmentBasePtr;
using OpenRAVE::RobotBasePtr;
using OpenRAVE::TrajectoryBasePtr;

namespace {

template <typename Tout, typename Tin>
static std::vector<Tout> Convert(std::vector<Tin> const &vin)
{
    std::vector<Tout> vout(vin.size());

    for (size_t i = 0; i < vin.size(); ++i) {
        vout[i] = vin[i];
    }

    return vout;
}

void ConvertWaypoint(TrajectoryBasePtr const &traj,
                     ParabolicRamp::ParabolicRampND const &ramp_nd,
                     double deltatime)
{
    OpenRAVE::ConfigurationSpecification const cspec
        = traj->GetConfigurationSpecification();
    std::vector<OpenRAVE::dReal> waypoint(cspec.GetDOF());
    size_t const num_dof = ramp_nd.ramps.size();

    for (size_t idof = 0; idof < ramp_nd.ramps.size(); ++idof) {
        ParabolicRamp::ParabolicRamp1D const &ramp = ramp_nd.ramps[idof];
        double const deltatime = ramp.EndTime();

        // TODO: Does each segment start at zero?
        waypoint[idof] = ramp.Evaluate(deltatime);
        waypoint[idof + num_dof] = ramp.Derivative(deltatime);
        waypoint[2 * num_dof] = deltatime;
    }

    traj->Insert(traj->GetNumWaypoints(), waypoint, false);
}

OpenRAVE::dReal FixLimit(OpenRAVE::dReal const &x,
                         OpenRAVE::dReal const &x_min,
                         OpenRAVE::dReal const &x_max,
                         OpenRAVE::dReal const &tolerance)
{
    if (x < x_min) {
        if (x < x_min - tolerance) {
            // TODO: error
            return x_min;
        } else {
            return x_min;
        }
    } else if (x > x_max) {
        if (x > x_max + tolerance) {
            // TODO: error;
            return x_max;
        } else {
            return x_max;
        }
    } else {
        return x;
    }
}


}

namespace or_parabolicsmoother {


/*
 * ORFeasibilityChecker 
 */
ORFeasibilityChecker::ORFeasibilityChecker(
        OpenRAVE::EnvironmentBasePtr const &env,
        OpenRAVE::PlannerBase::PlannerParametersConstPtr const &params)
    : env_(env)
    , params_(params)
{
}

bool ORFeasibilityChecker::ConfigFeasible(ParabolicRamp::Vector const &x)
{
    std::vector<OpenRAVE::dReal> const or_x = Convert<OpenRAVE::dReal>(x);
    std::vector<OpenRAVE::dReal> const empty;

    return params_->CheckPathAllConstraints(
        x, x, empty, empty, 0, OpenRAVE::IT_OpenStart
    );
}

bool ORFeasibilityChecker::SegmentFeasible(ParabolicRamp::Vector const &a,
                                           ParabolicRamp::Vector const &b)
{
    std::vector<OpenRAVE::dReal> const or_a = Convert<OpenRAVE::dReal>(a);
    std::vector<OpenRAVE::dReal> const or_b = Convert<OpenRAVE::dReal>(b);
    std::vector<OpenRAVE::dReal> const empty;

    return params_->CheckPathAllConstraints(
        a, b, empty, empty, 0, OpenRAVE::IT_OpenStart
    );
}

/*
 * ParabolicSmoother
 */
ParabolicSmoother::ParabolicSmoother(EnvironmentBasePtr penv)
    : OpenRAVE::PlannerBase(penv)
{
}

bool ParabolicSmoother::InitPlan(RobotBasePtr robot,
                                 PlannerParametersConstPtr params)
{
    parameters_ = params;
    return true;
}

bool ParabolicSmoother::InitPlan(RobotBasePtr robot,
                                 std::istream &input)
{
    PlannerParametersPtr const params = make_shared<PlannerParameters>();
    input >> *params;
    return InitPlan(robot, params);
}

OpenRAVE::PlannerStatus ParabolicSmoother::PlanPath(TrajectoryBasePtr traj)
{
    using OpenRAVE::ConfigurationSpecification;
    using OpenRAVE::KinBodyPtr;

    EnvironmentBasePtr const env = GetEnv();
    ConfigurationSpecification const pos_cspec
        = parameters_->_configurationspecification;
    ConfigurationSpecification const vel_cspec
        = pos_cspec.ConvertToVelocitySpecification();
    size_t const num_dof = pos_cspec.GetDOF();

    ParabolicRamp::DynamicPath dynamic_path;
    RAVELOG_DEBUG("Setting velocity and acceleration limits.\n");
    dynamic_path.Init(
        Convert<double>(parameters_->_vConfigVelocityLimit),
        Convert<double>(parameters_->_vConfigAccelerationLimit)
    );
    RAVELOG_DEBUG("Setting joint limits.\n");
    dynamic_path.SetJointLimits(
        Convert<double>(parameters_->_vConfigLowerLimit),
        Convert<double>(parameters_->_vConfigUpperLimit)
    );
    
    // Copy milestones into the DynamicPath. This assumes that the input
    // trajectory is piecewise linear and stops at each waypoint.
    // TODO: What about velocities?
    for (size_t iwaypoint = 0; iwaypoint < traj->GetNumWaypoints(); ++iwaypoint) {
        RAVELOG_DEBUG("Inserting waypoint %d.\n", iwaypoint);

        std::vector<OpenRAVE::dReal> waypoint;
        traj->GetWaypoint(iwaypoint, waypoint, pos_cspec);

        // Fix small joint limit violations.
        for (size_t idof = 0; idof < num_dof; ++idof) {
            waypoint[idof] = FixLimit(
                waypoint[idof],
                parameters_->_vConfigLowerLimit[idof],
                parameters_->_vConfigUpperLimit[idof],
                parameters_->_vConfigResolution[idof]
            );
        }

        dynamic_path.Append(
            Convert<double>(waypoint)
        );
    }

    // Choose the most conservative resolution because RampFeasibilityChecker
    // does not support per-DOF resolutions.
    RAVELOG_DEBUG("Creating collision checker.\n");

    OpenRAVE::dReal const tolerance = *std::min_element(
        parameters_->_vConfigResolution.begin(),
        parameters_->_vConfigResolution.end()
    );

    ORFeasibilityChecker base_checker(env, parameters_);
    ParabolicRamp::RampFeasibilityChecker ramp_checker(&base_checker, tolerance);

    // Shortcut.
    // TODO: Split this into multiple iterations so we can call callbacks.
    RAVELOG_DEBUG("Shortcutting for %d iterations.\n", parameters_->_nMaxIterations);
    //dynamic_path.Shortcut(parameters_->_nMaxIterations, ramp_checker);
    dynamic_path.Shortcut(500, ramp_checker);

    // Convert back to an OpenRAVE trajectory.
    OpenRAVE::ConfigurationSpecification output_cspec = pos_cspec + vel_cspec;
    output_cspec.AddDeltaTimeGroup();

    RAVELOG_DEBUG("Creating output trajectory.\n");
    OpenRAVE::planningutils::ConvertTrajectorySpecification(traj, output_cspec);

    RAVELOG_DEBUG("Converting ramp %d.\n", 0);
    BOOST_ASSERT(!dynamic_path.ramps.empty());
    ConvertWaypoint(traj, dynamic_path.ramps[0], 0.);

    for (size_t iramp = 0; iramp < dynamic_path.ramps.size(); ++iramp) {
        ParabolicRamp::ParabolicRampND const &ramp_nd = dynamic_path.ramps[iramp];

        RAVELOG_DEBUG("Converting ramp %d (duration: %f).\n", iramp, ramp_nd.endTime);
        ConvertWaypoint(traj, ramp_nd, ramp_nd.endTime);
    }

    return (dynamic_path.IsValid()) ? OpenRAVE::PS_HasSolution
                                    : OpenRAVE::PS_Failed;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr
    ParabolicSmoother::GetParameters () const
{
    return parameters_;
}

}
