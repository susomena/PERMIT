# -*- coding: utf-8 -*-
#
# Copyright (c) 2017 Michele Segata <segata@ccs-labs.org>
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
# Original code obtained from Michele Segata's original GitHub repository:
# https://github.com/michele-segata/plexe-python-demo
#

import sys
import os
import ccparams as cc
import random
import platooning
import math
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib
import traci


# constants for lane change mode
DEFAULT_LC = 0b1001010101
DEFAULT_NOTRACI_LC = 0b1010101010
FIX_LC = 0b0000000000


def set_par(vid, par, value):
    """
    Shorthand for the setParameter method
    :param vid: vehicle id
    :param par: parameter name
    :param value: numeric or string value for the parameter
    :type vid: str
    :type par: str
    """
    traci.vehicle.setParameter(vid, "carFollowModel.%s" % par, str(value))


def get_par(vid, par):
    """
    Shorthand for the getParameter method
    :param vid: vehicle id
    :param par: parameter name
    :type vid: str
    :type par: str
    :return: the parameter value
    """
    return traci.vehicle.getParameter(vid, "carFollowModel.%s" % par)


def change_lane(vid, lane):
    """
    Let a vehicle change lane without respecting any safety distance
    :param vid: vehicle id
    :param lane: lane index
    :type vid: str
    :type lane: int
    """
    traci.vehicle.setLaneChangeMode(vid, FIX_LC)
    traci.vehicle.changeLane(vid, lane, 1000000)


def add_vehicle(vid, position, lane, speed, cacc_spacing, real_engine=False):
    """
    Adds a vehicle to the simulation
    :param vid: vehicle id to be set
    :param position: position of the vehicle
    :param lane: lane
    :param speed: starting speed
    :param cacc_spacing: spacing to be set for the CACC
    :param real_engine: use the realistic engine model or the first order lag
    model
    :type vid: str
    :type position: float
    :type lane: int
    :type speed: float
    :type cacc_spacing: float
    :type real_engine: bool
    """
    traci.vehicle.add(vid, "platoon_route",
                      pos=position, speed=speed, lane=lane,
                      typeID="vtypeauto")
    set_par(vid, cc.CC_PAR_CACC_C1, 0.5)
    set_par(vid, cc.CC_PAR_CACC_XI, 2)
    set_par(vid, cc.CC_PAR_CACC_OMEGA_N, 1)
    set_par(vid, cc.PAR_CACC_SPACING, cacc_spacing)
    set_par(vid, cc.PAR_CC_DESIRED_SPEED, speed)
    if real_engine:
        set_par(vid, cc.CC_PAR_VEHICLE_ENGINE_MODEL,
                cc.CC_ENGINE_MODEL_REALISTIC)
        set_par(vid, cc.CC_PAR_VEHICLES_FILE, "vehicles.xml")
        set_par(vid, cc.CC_PAR_VEHICLE_MODEL, "alfa-147")
    traci.vehicle.setColor(vid, (random.uniform(0, 255),
                                 random.uniform(0, 255),
                                 random.uniform(0, 255), 0))


def get_distance(v1, v2):
    """
    Returns the distance between two vehicles, removing the length of the first
    vehicle
    :param v1: id of first vehicle
    :param v2: id of the second vehicle
    :type v1: str
    :type v2: str
    :return: distance between v1 and v2
    :rtype: float
    """
    v_data = get_par(v1, cc.PAR_SPEED_AND_ACCELERATION)
    (v, a, u, x1, y1, t) = cc.unpack(v_data)
    v_data = get_par(v2, cc.PAR_SPEED_AND_ACCELERATION)
    (v, a, u, x2, y2, t) = cc.unpack(v_data)
    # Since euclidean distance doesn't give information of which vehicle is in
    # first position (no sign) we need to identify the first vehicle
    first_vehicle = platooning.first_of([v1, v2])

    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2) - traci.vehicle.getLength(first_vehicle)


def communicate(topology):
    """
    Performs data transfer between vehicles, i.e., fetching data from
    leading and front vehicles to feed the CACC algorithm
    :param topology: a dictionary pointing each vehicle id to its front
    vehicle and platoon leader. Each entry of the dictionary is a dictionary
    which includes the keys "leader" and "front"
    :type topology: dict[str, dict[str, str]]
    """
    for vid, links in topology.iteritems():
        # get data about platoon leader
        leader_data = get_par(links["leader"], cc.PAR_SPEED_AND_ACCELERATION)
        (l_v, l_a, l_u, l_x, l_y, l_t) = cc.unpack(leader_data)
        leader_data = cc.pack(l_v, l_u, l_x, l_y, l_t)
        # get data about front vehicle
        front_data = get_par(links["front"], cc.PAR_SPEED_AND_ACCELERATION)
        (f_v, f_a, f_u, f_x, f_y, f_t) = cc.unpack(front_data)
        front_data = cc.pack(f_v, f_u, f_x, f_y, f_t)
        # pass leader and front vehicle data to CACC
        set_par(vid, cc.PAR_LEADER_SPEED_AND_ACCELERATION, leader_data)
        set_par(vid, cc.PAR_PRECEDING_SPEED_AND_ACCELERATION, front_data)
        # compute GPS distance and pass it to the fake CACC
        f_d = get_distance(vid, links["front"])
        set_par(vid, cc.PAR_LEADER_FAKE_DATA, cc.pack(l_v, l_u))
        set_par(vid, cc.PAR_FRONT_FAKE_DATA, cc.pack(f_v, f_u, f_d))


def start_sumo(sumo_binary, config_file, already_running):
    """
    Starts or restarts sumo with the given configuration file
    :param sumo_binary: SUMO binary to be called from command line (sumo or
    sumo-gui)
    :param config_file: sumo configuration file
    :param already_running: if set to true then the command simply reloads
    the given config file, otherwise sumo is started from scratch
    :type sumo_binary: str
    :type config_file: str
    :type already_running: bool
    """
    arguments = ["-c"]
    sumo_cmd = [sumolib.checkBinary(sumo_binary)]
    arguments.append(config_file)
    if already_running:
        traci.load(arguments)
    else:
        sumo_cmd.extend(arguments)
        traci.start(sumo_cmd)


def running(demo_mode, step, max_step):
    """
    Returns whether the demo should continue to run or not. If demo_mode is
    set to true, the demo should run indefinitely, so the function returns
    true. Otherwise, the function returns true only if step <= max_step
    :param demo_mode: true if running in demo mode
    :param step: current simulation step
    :param max_step: maximum simulation step
    :type demo_mode: bool
    :type step: int
    :type max_step: int
    :return: true if the simulation should continue
    :rtype: bool
    """
    if demo_mode:
        return True
    elif max_step == 0:  # if max_steps is 0 run simulation until all vehicles have arrived
        return traci.simulation.getMinExpectedNumber() > 0
    else:
        return step <= max_step


def validate_params(edge_filter, vtype_filter):
    """
    Checks if edge-filter and vtype-filter command line arguments are valid and
    in case of empty lists turns them into the full list of edges and vTypes
    :param edge_filter: list of edges where platooning is allowed (empty list
    means it is allowed everywhere)
    :param vtype_filter: list of vehicle types CACC enabled (empty list means
    every vehicle type is CACC enabled
    :type edge_filter: list[str]
    :type vtype_filter: list[str]
    :return: list of selected edges and list of selected vTypes
    :rtype: (list[str], list[str])
    """
    edges = traci.edge.getIDList()
    vtypes = traci.vehicletype.getIDList()

    if edge_filter is None:
        edge_filter = edges
    else:
        for edge in edge_filter:
            assert edge in edges, "%s not found in SUMO network" % edge

    if vtype_filter is None:
        vtype_filter = vtypes
    else:
        for vtype in vtype_filter:
            assert vtype in vtypes, "%s is not a vType of the simulation" % vtype

    return edge_filter, vtype_filter


def retrieve_vehicles(edge_filter):
    """
    Returns a list of the vehicles present on the selected edges
    :param edge_filter: list of edges where platooning is allowed
    :type edge_filter: list[str]
    :return: list of the vehicles present on the selected edges
    :rtype: list[str]
    """
    return [vehicle for edge in edge_filter for vehicle in traci.edge.getLastStepVehicleIDs(edge)]


def filter_cacc_vehicles(vehicles, vtype_filter):
    """
    Returns a list of the CACC enabled vehicles in the vehicles list
    :param vehicles: list of the simulation vehicles
    :param vtype_filter: list of vehicle types CACC enabled
    :type vehicles: list[str]
    :return: list of vehicles CACC enabled present in the list vehicles
    :rtype: list[str]
    """
    return [vehicle for vehicle in vehicles if traci.vehicle.getTypeID(vehicle) in vtype_filter]
