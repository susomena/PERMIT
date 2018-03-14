# -*- coding: utf-8 -*-
#
# Copyright (c) 2018 Jesús Mena-Oreja <jmena@umh.es>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see http://www.gnu.org/licenses/.
#

import os
import sys
import argparse
from formatter import CustomFormatter
import platooning
import ccparams as cc
import utils

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please, declare environment variable 'SUMO_HOME'")

import sumolib
import traci

parser = argparse.ArgumentParser(description="Platooning simulator for adding autonomous mobility on SUMO scenarios",
                                 formatter_class=CustomFormatter)
arg_group = parser.add_argument_group('Simulator parameters')
arg_group.add_argument("-c", "--configuration-file", type=str, help="config file of the SUMO scenario", metavar="FILE",
                       required=True)
arg_group.add_argument("-D", "--demo", action="store_true", help="run SUMO indefinitely")
arg_group.add_argument("-g", "--gui", action="store_true", help="run simulation with SUMO GUI")
arg_group.add_argument("-m", "--max-step", default=0, type=int, help="max simulation step for SUMO")
arg_group.add_argument("-ef", "--edge-filter", type=str,
                       help="list of edges where platooning is allowed (empty list means it is allowed everywhere)",
                       nargs="*", metavar=("EDGE 1", "EDGE 2"))
arg_group.add_argument("-vf", "--vtype-filter", type=str,
                       help="list of CACC enabled vehicle types (empty list means every vehicle type is CACC enabled)",
                       nargs="*", metavar=("VTYPE 1", "VTYPE 2"))
arg_group.add_argument("-md", "--max-distance", default=100.0, type=float, help="maximum distance for platoon merge")
arg_group.add_argument("-cs", "--cacc-spacing", default=5.0, type=float, help="intra-platoon spacing")
arg_group.add_argument("-pl", "--platoon-length", default=8, type=int, help="max platoon length in vehicles")
args = parser.parse_args()


def main():
    sumo_binary = "sumo-gui" if args.gui else "sumo"
    utils.start_sumo(sumo_binary, args.configuration_file, False)
    edge_filter, vtype_filter = utils.validate_params(args.edge_filter, args.vtype_filter)
    step = 0

    platoons = []

    while utils.running(args.demo, step, args.max_step):
        traci.simulationStep()

        vehicles = utils.retrieve_vehicles(edge_filter)
        cacc_vehicles = utils.filter_cacc_vehicles(vehicles, vtype_filter)
        simulation_vehicles = traci.vehicle.getIDList()

        for vehicle in cacc_vehicles:
            if not platooning.in_platoon(platoons, vehicle):
                platoons.append(platooning.Platoon([vehicle], cacc_spacing=args.cacc_spacing))

        for platoon in platoons:
            # Remove platoons with vehicles that have left the simulation
            if not platoon.all_members_are_in(simulation_vehicles):
                platoons.pop(platoons.index(platoon))
                continue

            platoon.look_for_splits()

            if platoon.leader_wants_to_leave(edge_filter):
                platoon.leader_leave()

        while True:
            merges = platooning.look_for_merges(platoons, max_distance=args.max_distance,
                                                max_platoon_length=args.platoon_length, edge_filter=edge_filter)

            for i in range(len(merges)):
                if merges[i] != -1:
                    platoons.append(platooning.merge_platoons(platoons[i], platoons[merges[i]]))
                    platoons = [platoon for platoon in platoons if platoon not in {platoons[i], platoons[merges[i]]}]

            if all([x == -1 for x in merges]):
                break

        for platoon in platoons:
            platoon.update_desired_speed()
            platoon.communicate()
            platoon.maneuver()

        step += 1

    traci.close()


if __name__ == "__main__":
    main()
