#ifndef PARABOLICSMOOTHERPARAMETERS_H_
#define PARABOLICSMOOTHERPARAMETERS_H_
#include <openrave/planner.h>

namespace or_parabolicsmoother {

class ParabolicSmootherParameters
    : public OpenRAVE::PlannerBase::PlannerParameters {
public:
    ParabolicSmootherParameters()
        : is_processing_(false)
        , do_blend_(true)
        , blend_radius_(0.5)
        , blend_iterations_(4)
        , do_shortcut_(true)
        , time_limit_(3.0)
        , use_velocity_(true)
    {
        _vXMLParameters.push_back("do_blend");
        _vXMLParameters.push_back("do_shortcut");
        _vXMLParameters.push_back("time_limit");
        _vXMLParameters.push_back("blend_radius");
        _vXMLParameters.push_back("blend_iterations");
        _vXMLParameters.push_back("use_velocity");
    }

    virtual void copy(boost::shared_ptr<PlannerParameters const> r)
    {
        boost::shared_ptr<ParabolicSmootherParameters const> p
            = boost::dynamic_pointer_cast<ParabolicSmootherParameters const>(r);
        if (p) {
            is_processing_ = p->is_processing_;
            do_blend_ = p->do_blend_;
            blend_radius_ = p->blend_radius_;
            blend_iterations_ = p->blend_iterations_;
            do_shortcut_ = p->do_shortcut_;
            time_limit_ = p->time_limit_;
            use_velocity_ = p->use_velocity_;
        }

        PlannerParameters::copy(r);
    }


    bool is_processing_;

    bool do_blend_;
    double blend_radius_;
    int blend_iterations_;

    bool do_shortcut_;

    double time_limit_;

    bool use_velocity_;

protected:
    virtual bool serialize(std::ostream &stream) const
    {
        if (!PlannerParameters::serialize(stream)) {
            return false;
        }

        stream
            << "<do_blend>" << do_blend_ << "</do_blend>\n"
            << "<do_shortcut>" << do_shortcut_ << "</do_shortcut>\n"
            << "<time_limit>" << time_limit_ << "</time_limit>\n"
            << "<blend_radius>" << blend_radius_ << "</blend_radius>\n"
            << "<blend_iterations>" << blend_iterations_ << "</blend_iterations>\n"
            << "<use_velocity>" << use_velocity_ << "</use_velocity>\n";

        return !!stream;
    }

    ProcessElement startElement(
        std::string const &name,
        std::list<std::pair<std::string, std::string> > const &atts)
    {
        if (is_processing_) {
            return PE_Ignore;
        }

        switch (PlannerParameters::startElement(name, atts)) {
            case PE_Pass:
                break;
            case PE_Support:
                return PE_Support;
            case PE_Ignore:
                return PE_Ignore;
        }

        is_processing_ =
             name == "do_blend"
          || name == "blend_radius"
          || name == "blend_iterations"
          || name == "do_shortcut"
          || name == "time_limit"
          || name == "use_velocity";

        return is_processing_ ? PE_Support : PE_Pass;
    }

    virtual bool endElement(std::string const &name)
    {
        using boost::format;
        using boost::str;

        if (is_processing_) {
            if (name == "do_blend") {
                _ss >> do_blend_;
            } else if (name == "blend_radius") {
                _ss >> blend_radius_;
            } else if (name == "blend_iterations") {
                _ss >> blend_iterations_;
            } else if (name == "do_shortcut") {
                _ss >> do_shortcut_;
            } else if (name == "time_limit") {
                _ss >> time_limit_;
            } else if (name == "use_velocity") {
                _ss >> use_velocity_;
            } else {
                RAVELOG_WARN(str(format("unknown tag %s\n") % name));
            }
            is_processing_ = false;
            return false;
        }

        return PlannerParameters::endElement(name);
    }
};

typedef boost::shared_ptr<ParabolicSmootherParameters> ParabolicSmootherParametersPtr;
typedef boost::shared_ptr<ParabolicSmootherParameters const> ParabolicSmootherParametersConstPtr;

} // namespace or_parabolicsmoother

#endif
