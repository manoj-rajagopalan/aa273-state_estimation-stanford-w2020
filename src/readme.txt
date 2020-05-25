Simulation environment for 2D robot traversing through environment of obstacles.

Top level function:
-------------------
run_sim.m

Usage:
    run_sim()
    run_sim(1) % To see more debugging in the range finder plot.


Parts:
------
polygon.m:       Class for manipulating polygons used.
                 Includes a plot member function.

range_finder.m:  Class for calculations involved in a simulated range finder.
                 Includes a plot member function.

robot.m:         Class for kinematic state transition of the 2-wheel differential drive robot.
                 Includes a plot member function.
                 Currently doesn't have odometer, but that could be added if needed.

init_polygons.m, init_plot_scene.m, plot_scene.m:  These are utility functions for packing calls
                                                   to the above, in the interest of reducing clutter.

Filters to be added:
--------------------
For filters that each of us need to add, please write a separate function / class for your filters.
Then call them in your copy of "run_sim.m" at the bottom where there are comments about "filters TBD".
