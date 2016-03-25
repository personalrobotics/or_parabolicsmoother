from copy import deepcopy
from prpy.planning.retimer import OpenRAVERetimer
from prpy.planning.base import PlanningMethod


class HauserParabolicSmoother(OpenRAVERetimer):
    """
    A prpy wrapper for the `or_parabolicsmoother` plugin.
    """
    def __init__(self, do_blend=True, do_shortcut=True, blend_radius=0.5,
                 blend_iterations=0, timelimit=3., **kwargs):
        super(HauserParabolicSmoother, self).__init__(
                'HauserParabolicSmoother', **kwargs)

        self.default_options.update({
            'do_blend': int(do_blend),
            'do_shortcut': int(do_shortcut),
            'blend_radius': float(blend_radius),
            'blend_iterations': int(blend_iterations),
            'time_limit': float(timelimit),
        })

    @PlanningMethod
    def RetimeTrajectory(self, robot, path, options=None, **kw_args):
        # Copy the user-specified before passing them.
        new_options = deepcopy(options) if options else dict()
        if 'timelimit' in kw_args:
            new_options['time_limit'] = kw_args['timelimit']

        # Call the abstract retimer implementation.
        return super(HauserParabolicSmoother, self).RetimeTrajectory(
            robot, path, options=new_options, **kw_args)
