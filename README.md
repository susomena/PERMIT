PERMIT - A SUMO Simulator for Platooning Maneuvers in Mixed Traffic Scenarios
======

This is the open-source repository for PERMIT, a platooning simulator based on the microscopic traffic simulator SUMO and its platooning extension Plexe.

This simulator allows to simulate platooning maneuvers and driving in mixed traffic scenarios where automated and non-automated vehicles coexist. The different platooning maneuvers implemented in PERMIT are the join maneuver, the merge maneuver, the leave maneuver and the split maneuver.

PERMIT can be used with an existent Plexe scenario without the need of making any change to that scenario. To use PERMIT just run it using the following command:

	python simulator.py -c plexe_scenario.sumo.cfg

In order to use PERMIT, Python and Plexe-SUMO are needed. The code of Plexe-SUMO can be found in [this repository](https://github.com/susomena/plexe-sumo).

If you use PERMIT for your research please cite our work:

>J. Mena and J. Gozalvez, "PERMIT - A sumo simulator for platooning maneuvers in mixed traffic scenarios," in *Proc. of the 21st International Conference on Intelligent Transportation Systems (IEEE ITSC 2018)*, 4-7 November, 2018, Maui, HI, USA, pp. 3445-3450.

The code for PERMIT takes as a starting point the Plexe Python demos that can be found in [this repository](https://github.com/michele-segata/plexe-python-demo). PERMIT is released under a LGPL 3.0 open-source license. A copy of the LGPL 3.0 license can be found in this repository.
