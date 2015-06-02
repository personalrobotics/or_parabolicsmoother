# or_parabolicsmoother #
## An OpenRAVE Plugin for Parabolic Smoothing ##

This is an OpenRAVE wrapper to a fast smoothing algorithm for robot manipulator
trajectories under geometric constraints and bounded acceleration.

It is based on code provided with the paper:
  * K. Hauser and V. Ng-Thow-Hing, *Fast Smoothing of Manipulator Trajectories using Optimal Bounded-Acceleration Shortcuts by*, ICRA 2010.

## Parameters

This algorithm takes the following parameters (with the specified default values):

- `<do_shortcut>1</do_shortcut>` If non-zero, run shortcutting iterations. 
- `<do_blend>1</do_blend>`: If non-zero, perform blending.
- `<blend_radius>0.5</blend_radius>` Largest blend radius to attempt, in seconds.
- `<blend_iterations>5</blend_iterations>` Number of blend radii to search over.
  Each iteration halfs the blend radius.
