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
                     ParabolicRamp::DynamicPath const &dynamic_path,
                     double t, double dt)
{
    size_t const num_dof = dynamic_path.velMax.size();

    OpenRAVE::ConfigurationSpecification const cspec
        = traj->GetConfigurationSpecification();

    ParabolicRamp::Vector q;
    dynamic_path.Evaluate(t, q);
    BOOST_ASSERT(q.size() == num_dof);

    ParabolicRamp::Vector qd;
    dynamic_path.Derivative(t, qd);
    BOOST_ASSERT(qd.size() == num_dof);

    std::vector<OpenRAVE::dReal> waypoint(cspec.GetDOF());
    for (size_t idof = 0; idof < num_dof; ++idof) {
        waypoint[idof] = q[idof];
        waypoint[idof + num_dof] = qd[idof];
    }

    waypoint[2 * num_dof] = dt;

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

    return !params_->CheckPathAllConstraints(
        x, x, empty, empty, 0, OpenRAVE::IT_OpenStart
    );
}

bool ORFeasibilityChecker::SegmentFeasible(ParabolicRamp::Vector const &a,
                                           ParabolicRamp::Vector const &b)
{
    std::vector<OpenRAVE::dReal> const or_a = Convert<OpenRAVE::dReal>(a);
    std::vector<OpenRAVE::dReal> const or_b = Convert<OpenRAVE::dReal>(b);
    std::vector<OpenRAVE::dReal> const empty;

    return !params_->CheckPathAllConstraints(
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
    ConfigurationSpecification pos_cspec
        = parameters_->_configurationspecification;

    // TODO: How do we do this properly?
    BOOST_FOREACH (ConfigurationSpecification::Group &group,
                   pos_cspec._vgroups) {
        group.interpolation = "quadratic";
    }

    ConfigurationSpecification vel_cspec
        = pos_cspec.ConvertToVelocitySpecification();

    size_t num_dof = pos_cspec.GetDOF();
    RAVELOG_DEBUG("Detected %d DOFs.\n", num_dof);

    ParabolicRamp::DynamicPath dynamic_path;
    RAVELOG_DEBUG("Setting velocity and acceleration limits.\n");
    dynamic_path.Init(
        Convert<double>(parameters_->_vConfigVelocityLimit),
        Convert<double>(parameters_->_vConfigAccelerationLimit)
    );
    BOOST_ASSERT(dynamic_path.velMax.size() == num_dof);
    BOOST_ASSERT(dynamic_path.accMax.size() == num_dof);

#if 0
    RAVELOG_DEBUG("Setting joint limits.\n");
    dynamic_path.SetJointLimits(
        Convert<double>(parameters_->_vConfigLowerLimit),
        Convert<double>(parameters_->_vConfigUpperLimit)
    );
    BOOST_ASSERT(dynamic_path.xMin.size() == num_dof);
    BOOST_ASSERT(dynamic_path.xMax.size() == num_dof);
#endif
    
    // Copy milestones into the DynamicPath. This assumes that the input
    // trajectory is piecewise linear and stops at each waypoint.
    // TODO: What about velocities?
    std::vector<ParabolicRamp::Vector> milestones(traj->GetNumWaypoints());

    for (size_t iwaypoint = 0; iwaypoint < traj->GetNumWaypoints(); ++iwaypoint) {
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

        BOOST_ASSERT(waypoint.size() == num_dof);
        milestones[iwaypoint] = Convert<double>(waypoint);
    }

    RAVELOG_DEBUG("Setting %d milestones.\n", milestones.size());
    dynamic_path.SetMilestones(milestones);

    if (!dynamic_path.IsValid()) {
        throw OpenRAVE::openrave_exception(
            "Converted DynamicPath is not valid.",
            OpenRAVE::ORE_Failed
        );
    }

    // Choose the most conservative resolution because RampFeasibilityChecker
    // does not support per-DOF resolutions.
    RAVELOG_DEBUG("Creating collision checker.\n");

    OpenRAVE::dReal const tolerance = *std::min_element(
        parameters_->_vConfigResolution.begin(),
        parameters_->_vConfigResolution.end()
    );
    BOOST_ASSERT(tolerance > 0.);

    ORFeasibilityChecker base_checker(env, parameters_);
    ParabolicRamp::RampFeasibilityChecker ramp_checker(&base_checker, tolerance);

    // Shortcut.
    // TODO: Split this into multiple iterations so we can call callbacks.
    RAVELOG_DEBUG("Shortcutting for %d iterations.\n", parameters_->_nMaxIterations);

    //dynamic_path.Shortcut(parameters_->_nMaxIterations, ramp_checker);
    dynamic_path.Shortcut(10, ramp_checker);

    // Clear the trajectory to write in the output.
    traj->Remove(0, traj->GetNumWaypoints());
    BOOST_ASSERT(traj->GetNumWaypoints() == 0);

    // Convert back to an OpenRAVE trajectory.
    OpenRAVE::ConfigurationSpecification output_cspec = pos_cspec + vel_cspec;
    output_cspec.AddDeltaTimeGroup();

    RAVELOG_DEBUG("Creating output trajectory (duration: %f).\n", dynamic_path.GetTotalTime());
    OpenRAVE::planningutils::ConvertTrajectorySpecification(traj, output_cspec);

    ConvertWaypoint(traj, dynamic_path, 0., 0.);

    //double t = dynamic_path.ramps[0].endTime;
    size_t hack_n = 10000;
    double t = dynamic_path.GetTotalTime() / hack_n;

    //for (size_t iramp = 0; iramp < dynamic_path.ramps.size(); ++iramp) {
    for (size_t iramp = 0; iramp < hack_n; ++iramp) {
        //double const dt = dynamic_path.ramps[iramp].endTime;
        double const dt = dynamic_path.GetTotalTime() / hack_n; 

        RAVELOG_DEBUG("Converting ramp %d (duration: %f).\n", iramp, dt);
        ConvertWaypoint(traj, dynamic_path, t, dt);

        t += dt;
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
