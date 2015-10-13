^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package or_parabolicsmoother
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2015-10-12)
------------------
* Added parameters (`#8 <https://github.com/personalrobotics/or_parabolicsmoother/issues/8>`_, `#9 <https://github.com/personalrobotics/or_parabolicsmoother/issues/9>`_)
* Added basic support for trajectories that define waypoint velocities (`#17 <https://github.com/personalrobotics/or_parabolicsmoother/issues/17>`_)
* Added a check to terminate smoothing when the trajectory contains two waypoints (`#12 <https://github.com/personalrobotics/or_parabolicsmoother/issues/12>`_)
* Fixed a bug that duplicated the first waypoint (`#14 <https://github.com/personalrobotics/or_parabolicsmoother/issues/14>`_)
* Fixed a bug where last ramp would be shortcutted forever due to numerical precision errors (`#10 <https://github.com/personalrobotics/or_parabolicsmoother/issues/10>`_)
* Fixed missing snap on sampling out-of-bounds past end of traj (`#15 <https://github.com/personalrobotics/or_parabolicsmoother/issues/15>`_)
* Contributors: Michael Koval, Pras Velagapudi, Shushman

1.0.0 (2015-05-01)
------------------
* Fixed a bug that caused all shortcutting to fail.
* Implemented waypoint blending.
* Renamed the smoother to HauserParabolicSmoother.
* Changed format specifier for `size_t` printfs to C99 spec `%zu`.
* Added default max_iterations and loading from planningparameters.
* Configured logging to go through OpenRAVE.
* Create README.md
* Implemented the entire plugin.
* Added CMakeLists.txt.
* Removed mc_test.cpp.
* Adding missing #include.
* Moved headers into include.
* Restructured into Catkin package layout.
* Imported ParabolicSmoother.
* Contributors: Michael Koval, Pras, Pras Velagapudi
