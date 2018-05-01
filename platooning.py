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
import ccparams as cc
import utils
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci


class Platoon:

    """

    Class for the management of a platoon. This simulator considers that every
    CACC enabled vehicle is already a platoon composed by just one vehicle.

    """

    # Maneuver states
    IDLE = 0
    GOING_TO_POSITION = 1
    CHECK_LANE = 2
    CLOSING_GAP = 3
    LEAVING = 4
    OPENING_GAP = 5
    WAITING = 6

    def __init__(self, vehicles, desired_gap, safe_gap, merging=False, lane_change=False):
        """
        Constructor of the Platoon class. Every platoon is initialized with a
        ordered list of vehicles. The first vehicle is the platoon leader and
        uses an ACC controller, while the followers use a CACC controller. If
        the vehicles list is composed by just one vehicle then a platoon of one
        vehicle is created.
        :param vehicles: list of vehicles that compose the platoon
        :param desired_gap: distance between vehicles in a platoon
        :param safe_gap: safety distance between vehicles for lane changes
        :param merging: whether the new platoon is in a maneuver or not
        :type vehicles: list[str]
        :type desired_gap: float
        :type safe_gap: float
        :type merging: bool
        """
        self._topology = dict()
        self._states = [self.IDLE]
        if len(vehicles) > 1:
            traci.vehicle.setLaneChangeMode(vehicles[0], utils.FIX_LC)
        utils.set_par(vehicles[0], cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
        traci.vehicle.setSpeedFactor(vehicles[0], 1)

        lanes = [traci.vehicle.getLaneIndex(vehicles[0])]
        lane_ids = [traci.vehicle.getLaneID(vehicles[0])]

        for i in range(1, len(vehicles)):
            traci.vehicle.setLaneChangeMode(vehicles[i], utils.FIX_LC)
            self._topology[vehicles[i]] = {"leader": vehicles[0], "front": vehicles[i - 1]}
            self._states.append(self.WAITING)
            utils.set_par(vehicles[i], cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
            min_gap = traci.vehicle.getMinGap(vehicles[i])
            # The CACC controller tries to keep (desired_gap + min_gap)
            # meters as intra-platoon spacing
            utils.set_par(vehicles[i], cc.PAR_CACC_SPACING, desired_gap - min_gap)
            traci.vehicle.setSpeedFactor(vehicles[i], 1.05)

            lanes.append(traci.vehicle.getLaneIndex(vehicles[i]))
            lane_ids.append(traci.vehicle.getLaneID(vehicles[i]))

        self._members = vehicles
        self._desired_gap = desired_gap
        self._safe_gap = safe_gap
        self._merging = merging
        self._splitting = False
        self._vehicles_splitting = [False] * len(vehicles)

        leader_max_speed = traci.vehicle.getAllowedSpeed(vehicles[0])
        lane = traci.vehicle.getLaneID(vehicles[0])
        road_max_speed = traci.lane.getMaxSpeed(lane)
        # If the platoon is fast enough merge all vehicles to the leftmost
        # lane, else merge to the rightmost lane
        self._desired_lane = max(lanes) if leader_max_speed > 0.9 * road_max_speed else min(lanes)
        self._desired_lane_id = lane_ids[lanes.index(self._desired_lane)]

        if lane_change:
            for vehicle in self._members:
                if traci.vehicle.getLaneIndex != self._desired_lane:
                    utils.change_lane(vehicle, self._desired_lane)

    def __contains__(self, vehicle):
        return vehicle in self._members

    def update_desired_speed_and_lane(self):
        """
        The ACC and CACC of Plexe SUMO seem to limit the desired speed
        automatically to 13.9m/s, so this function should be called at every
        time step to ensure that the maximum speed is updated after every edge
        change. This function also changes the desired lane when a platoon
        composed by one single vehicle has performed a manual lane change
        """
        for vehicle in self._members:
            # getAllowedSpeed() also includes the speed factor
            lane_max_speed = traci.vehicle.getAllowedSpeed(vehicle)
            vehicle_max_speed = traci.vehicle.getMaxSpeed(vehicle)
            desired_speed = min(lane_max_speed, vehicle_max_speed)
            utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, desired_speed)

        if len(self._members) == 1:
            self._desired_lane = traci.vehicle.getLaneIndex(self._members[0])
            self._desired_lane_id = traci.vehicle.getLaneID(self._members[0])

    def communicate(self):
        """
        Performs data transfer between vehicles, i.e., fetching data from
        leading and front vehicles to feed the CACC algorithm. This function is
        an adaptation of the original that can be found in the Michele Segata's
        GitHub repository and in the utils.py file
        """
        for vid, links in self._topology.iteritems():
            # get data about platoon leader
            leader_data = utils.get_par(links["leader"], cc.PAR_SPEED_AND_ACCELERATION)
            (l_v, l_a, l_u, l_x, l_y, l_t) = cc.unpack(leader_data)
            leader_data = cc.pack(l_v, l_u, l_x, l_y, l_t)
            # get data about front vehicle
            front_data = utils.get_par(links["front"], cc.PAR_SPEED_AND_ACCELERATION)
            (f_v, f_a, f_u, f_x, f_y, f_t) = cc.unpack(front_data)
            front_data = cc.pack(f_v, f_u, f_x, f_y, f_t)
            # pass leader and front vehicle data to CACC
            if (l_a < 0 and f_a < 0) or (l_a > 0 and f_a > 0):
                utils.set_par(vid, cc.PAR_LEADER_SPEED_AND_ACCELERATION, leader_data)
            else:
                utils.set_par(vid, cc.PAR_LEADER_SPEED_AND_ACCELERATION, front_data)
            utils.set_par(vid, cc.PAR_PRECEDING_SPEED_AND_ACCELERATION, front_data)
            # compute GPS distance and pass it to the fake CACC
            f_d = gap_between_vehicles(vid, links["front"])[0]
            utils.set_par(vid, cc.PAR_LEADER_FAKE_DATA, cc.pack(l_v, l_u))
            utils.set_par(vid, cc.PAR_FRONT_FAKE_DATA, cc.pack(f_v, f_u, f_d))

    def leader_wants_to_leave(self, edge_filter):
        """
        Checks if the platoon leader is arriving to an edge where platooning is
        not allowed
        :param edge_filter: the list of edges where platooning is allowed
        :type edge_filter: list[str]
        :return: True if the Platoon leader is arriving to an edge where
        platooning is not allowed, False otherwise
        """
        edge = traci.vehicle.getRoadID(self._members[0])
        # If vehicle is in an internal edge don't check for leaving. If a leave
        # is coming it should have been noted before
        if edge.startswith(':'):
            return False

        route = traci.vehicle.getRoute(self._members[0])

        if edge not in route or route.index(edge) == len(route) - 1:
            return True

        next_edge = route[route.index(edge) + 1]

        if next_edge not in edge_filter:
            # TODO Don't hardcode the distance and make it speed dependant
            if traci.vehicle.getDrivingDistance(self._members[0], next_edge, 0) < 750.0:
                return True

        return False

    def remove_vehicle(self, index):
        """
        Creates a new platoon with all the vehicles of the platoon except for
        the vehicle in the position index
        :param index: the position of the vehicle to be removed
        :type index: int
        """
        traci.vehicle.setLaneChangeMode(self._members[index], utils.DEFAULT_LC)
        if len(self._members) != 1:
            self._members.pop(index)

        self.__init__(self._members, self._desired_gap, self._safe_gap)

    def leader_leave(self):
        """
        Creates a new platoon with all the vehicles of the platoon except for
        the former leader
        """
        self.remove_vehicle(0)

    def look_for_splits(self):
        """
        Checks if any vehicle of the platoon wants to leave and prepares the
        platoon for the maneuver in that case
        """
        for i in range(len(self._members)):
            edge = traci.vehicle.getRoadID(self._members[i])
            # If vehicle is in an internal edge don't check for splits. If a
            # split is coming it should have been noted before
            if edge.startswith(':'):
                continue

            route = traci.vehicle.getRoute(self._members[i])

            if route.index(edge) == len(route) - 1:
                continue

            next_edge = route[route.index(edge) + 1]
            platoon_route = traci.vehicle.getRoute(self._members[0])

            if edge not in platoon_route or platoon_route.index(edge) + 1 >= len(platoon_route):
                continue

            next_platoon_edge = platoon_route[platoon_route.index(edge) + 1]

            if next_edge != next_platoon_edge:
                # TODO Don't hardcode the distance and make it speed dependant
                if traci.vehicle.getDrivingDistance(self._members[i], next_edge, 0) < 750.0 \
                        and not self._vehicles_splitting[i]:
                    self._splitting = True
                    self._vehicles_splitting[i] = True
                    self._states[i] = self.LEAVING
                    if i < len(self._states) - 1:
                        self._states[i + 1] = self.OPENING_GAP

    def merge(self):
        """
        Executes the merge maneuver finite state machine of every vehicle of
        the platoon
        """
        if it_is_safe_to_change_lane(self._members[0], self._desired_lane_id, self._safe_gap - 1):
            utils.change_lane(self._members[0], self._desired_lane)

        for i in range(1, len(self._members)):
            if self._states[i] is self.WAITING:
                # The gap-making stage is done in a sequential manner to avoid
                # huge decelerations and possible dangerous situations as is
                # done in:
                # E. Semsar-kazerooni, J. Ploeg, "Interaction protocols for
                # cooperative merging and lane reduction scenarios," IEEE
                # Conference on Intelligent Transportation Systems (ITSC), Las
                # Palmas, Spain, 2015, pp. 1964-1970
                lane = traci.vehicle.getLaneIndex(self._members[i])
                lane_front = traci.vehicle.getLaneIndex(self._members[i - 1])
                if lane != lane_front:
                    utils.set_par(self._members[i], cc.PAR_ACTIVE_CONTROLLER, cc.FAKED_CACC)

                if self._states[i - 2] is not self.WAITING and self._states[i - 2] is not self.GOING_TO_POSITION \
                        or i == 1:
                    self._topology[self._members[i]]["leader"] = self._members[0]
                    self._states[i] = self.GOING_TO_POSITION
                else:
                    self._topology[self._members[i]]["leader"] = self._members[i - 1]

            if self._states[i] is self.GOING_TO_POSITION:
                lane = traci.vehicle.getLaneIndex(self._members[i])
                lane_front = traci.vehicle.getLaneIndex(self._members[i - 1])
                if lane != lane_front:
                    utils.set_par(self._members[i], cc.PAR_ACTIVE_CONTROLLER, cc.FAKED_CACC)
                else:
                    utils.set_par(self._members[i], cc.PAR_ACTIVE_CONTROLLER, cc.CACC)

                front_distance = gap_between_vehicles(self._members[i], self._members[i - 1])[0]
                no_need_to_open_gap = already_in_lane(self._members[i], self._desired_lane)
                no_need_to_open_gap = no_need_to_open_gap and already_in_lane(self._members[i - 1], self._desired_lane)
                if self._safe_gap - 1 < front_distance < self._safe_gap + 1 \
                        and self._states[i - 1] is not self.GOING_TO_POSITION or no_need_to_open_gap:
                    self._states[i] = self.CHECK_LANE
                else:
                    utils.set_par(self._members[i], cc.PAR_CACC_SPACING, self._safe_gap)

            if self._states[i] is self.CHECK_LANE:
                lane = traci.vehicle.getLaneIndex(self._members[i])
                lane_front = traci.vehicle.getLaneIndex(self._members[i - 1])
                if lane != lane_front:
                    utils.set_par(self._members[i], cc.PAR_ACTIVE_CONTROLLER, cc.FAKED_CACC)
                else:
                    utils.set_par(self._members[i], cc.PAR_ACTIVE_CONTROLLER, cc.CACC)

                if it_is_safe_to_change_lane(self._members[i], self._desired_lane_id, self._safe_gap - 1)\
                        or already_in_lane(self._members[i], self._desired_lane):
                    utils.change_lane(self._members[i], self._desired_lane)
                    utils.set_par(self._members[i], cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                    min_gap = traci.vehicle.getMinGap(self._members[i])
                    # The CACC controller tries to keep (cacc_spacing + min_gap)
                    # meters as intra-platoon spacing
                    utils.set_par(self._members[i], cc.PAR_CACC_SPACING, self._desired_gap - min_gap)
                    self._states[i] = self.CLOSING_GAP

            if self._states[i] is self.CLOSING_GAP:
                front_distance = gap_between_vehicles(self._members[i], self._members[i - 1])[0]
                if self._desired_gap - 1 < front_distance < self._desired_gap + 1:
                    self._states[i] = self.IDLE

        if self.all_members_are_idle():
            self._merging = False

    def split(self):
        """
        Executes the split maneuver finite state machine of every vehicle of
        the platoon
        """
        for i in range(1, len(self._members)):
            if self._states[i] is self.LEAVING:
                front_distance = gap_between_vehicles(self._members[i], self._members[i - 1])[0]
                if self._safe_gap - 1 < front_distance < self._safe_gap + 1:
                    self._states[i] = self.CHECK_LANE
                else:
                    utils.set_par(self._members[i], cc.PAR_ACTIVE_CONTROLLER, cc.FAKED_CACC)
                    utils.set_par(self._members[i], cc.PAR_CACC_SPACING, self._safe_gap)

            if self._states[i] is self.CHECK_LANE:
                desired_lane = self._desired_lane - 1 if self._desired_lane > 1 else 0
                desired_lane_id = self._desired_lane_id[:-1] + str(desired_lane)
                if it_is_safe_to_change_lane(self._members[i], desired_lane_id, self._safe_gap - 1):
                    utils.change_lane(self._members[i], desired_lane)
                    utils.set_par(self._members[i], cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                    traci.vehicle.setSpeedFactor(self._members[i], 1.0)

                    self._states[i] = self.IDLE

            if self._states[i] is self.OPENING_GAP:
                front_distance = gap_between_vehicles(self._members[i], self._members[i - 1])[0]
                if self._safe_gap - 1 < front_distance < self._safe_gap + 1 \
                        and self._states[i - 1] is self.CHECK_LANE or self._states[i - 1] is self.IDLE:
                    self._states[i] = self.WAITING
                else:
                    utils.set_par(self._members[i], cc.PAR_CACC_SPACING, self._safe_gap)

            if self._states[i] is self.WAITING:
                front_vehicle_index = self._members.index(self._topology[self._members[i]]["front"])
                if self._states[front_vehicle_index] is self.IDLE:
                    self._topology[self._members[i]]["front"] = self._members[front_vehicle_index - 1]
                    if not self._vehicles_splitting[front_vehicle_index - 1]:
                        utils.set_par(self._members[i], cc.PAR_CACC_SPACING, self._desired_gap)
                        self._states[i] = self.IDLE

        if self.all_members_are_idle():
            self._splitting = False
            remaining_platoon = [self._members[i] for i in range(len(self._members)) if not self._vehicles_splitting[i]]
            self.__init__(remaining_platoon, self._desired_gap, True)

    def all_members_are_idle(self):
        """
        Checks whether all platoon members are idle or not
        :return: True if all the platoon members state is IDLE, False otherwise
        :rtype: bool
        """
        for state in self._states:
            if state is not self.IDLE:
                return False

        return True

    def maneuver(self):
        """
        Checks and executes the maneuver of the current time step. This
        function gives splitting priority over merging
        """
        if self._splitting:
            self.split()
        elif self._merging:
            self.merge()

    def get_desired_gap(self):
        """
        Returns the desired gap of this platoon
        :return: the desired gap of this platoon
        :rtype: float
        """
        return self._desired_gap

    def get_safe_gap(self):
        """
        Returns the safe gap of this platoon
        :return: the safe gap of this platoon
        :rtype: float
        """
        return self._safe_gap

    def get_leader(self):
        """
        Returns the ID of the platoon leader
        :return: the ID of the platoon leader
        :rtype: str
        """
        return self._members[0]

    def get_members(self):
        """
        Return a list of the IDs of the platoon members
        :return: a list of the IDs of the platoon members
        :rtype: list[str]
        """
        return self._members

    def length(self):
        """
        Return the length of the platoon in vehicles
        :return: length of the platoon in vehicles
        :rtype: int
        """
        return len(self._members)

    def distance_to(self, platoon):
        """
        Computes the distance between the platoon and another platoon as the
        distance between their leaders
        :param platoon: the platoon with respect to which the distance is
        computed
        :type platoon: Platoon
        :return: distance between the two platoons. The returned value can be
        negative (and invalid) if the platoon with respect to which the
        distance is computed is placed behind the self platoon
        :rtype: float
        """
        return distance_between_vehicles(self.get_leader(), platoon.get_leader())[0]

    def is_splitting(self):
        """
        Returns whether this platoon is in a split maneuver or not
        :return: True if the platoon is in a split maneuver, False if not
        :rtype: bool
        """
        return self._splitting

    def get_edges_list(self):
        """
        Returns a list containing the edges where all the platoon members are
        placed
        :return: list of edges of the platoon members
        :rtype: list[str]
        """
        return [traci.vehicle.getRoadID(vehicle) for vehicle in self._members]

    def get_routes_list(self):
        """
        Returns a list containing the routes of all the platoon members. The
        routes are lists of edges.
        :return: a list of the routes of all the platoon members
        :rtype: list[list[str]]
        """
        return [traci.vehicle.getRoute(vehicle) for vehicle in self._members]

    def all_members_are_in(self, vehicle_list):
        """
        Checks whether all the vehicles of the platoon are present in a list of
        vehicles or not
        :param vehicle_list: the list to be checked
        :type vehicle_list: list[str]
        :return: True if all the vehicles of the platoon are present in the
        list, False otherwise
        :rtype: bool
        """
        for vehicle in self._members:
            if vehicle not in vehicle_list:
                return False

        return True

    def get_desired_lane(self):
        """
        Returns the platoon's desired lane index
        :return: the index of the platoon's desired lane
        :rtype: int
        """
        return self._desired_lane

    def get_desired_lane_id(self):
        """
        Returns the platoon's desired lane id
        :return: the id of the platoon's desired lane
        :rtype: str
        """
        return self._desired_lane_id

    def length_sum(self):
        """
        Returns the sum of the lengths of all the platoon members
        :return: the sum of the lengths of all the platoon members
        :rtype: float
        """
        length = 0

        for vehicle in self._members:
            length += traci.vehicle.getLength(vehicle)

        return length

    def index_of(self, vehicle):
        """
        Returns the position of the vehicle in the platoon or -1 if the vehicle
        is not a member of the platoon
        :param vehicle: id of the vehicle
        :type vehicle: str
        :return: position of the vehicle in the platoon or -1 if the vehicle is
        not a member of the platoon
        :rtype: int
        """
        for i in range(len(self._members)):
            if vehicle == self._members[i]:
                return i

        return -1

    def get_head_position(self):
        """
        Returns the position of the front bumper of the platoon leader
        :return: position of the front bumper of the platoon leader
        :rtype: float
        """
        return traci.vehicle.getLanePosition(self._members[0])

    def get_tail_position(self):
        """
        Returns the position of the back bumper of the platoon's last member
        :return: position of the back bumper of the platoon's last member
        :rtype: float
        """
        pos = traci.vehicle.getLanePosition(self._members[-1])
        l = traci.vehicle.getLength(self._members[-1])
        return pos - l


def in_platoon(platoons, vehicle):
    """
    Checks if a vehicle is a member of any platoon in a list of platoons
    :param platoons: list of platoons
    :param vehicle: vehicle to check if it is a member of a platoon of the list
    :type platoons: list[Platoon]
    :type vehicle: str
    :return: True if the vehicle is a member of a platoon of the list, False if
    not.
    :rtype: bool
    """
    for platoon in platoons:
        if vehicle in platoon:
            return True

    return False


def distance_between_vehicles(v1, v2, recursion=False):
    """
    Computes the distance between two vehicles (from v1 to v2)
    :param v1: ID of the first vehicle
    :param v2: ID of the second vehicle
    :param recursion: whether this function is called from itself or not
    :type v1: str
    :type v2: str
    :type recursion: bool
    :return: tuple with the distance between the two vehicles (from v1 to v2)
    and whether this distance is valid or not
    :rtype: (float, bool)
    """
    edge = traci.vehicle.getRoadID(v2)
    pos = traci.vehicle.getLanePosition(v2)
    lane = traci.vehicle.getLaneIndex(v2)
    if lane < 0:
        return -1, False

    distance = traci.vehicle.getDrivingDistance(v1, edge, pos, lane)

    # Sometimes both vehicles will be unreachable for each other, in this case
    # the computed distance in both ways will be negative and invalid
    if recursion and distance < 0:
        return -1, False

    # Since getDrivingDistance can return a negative and invalid distance when
    # computing the distance to a vehicle that is placed behind, we compute the
    # distance in both directions to use the valid distance (which will be the
    # positive and valid one) and multiply the distance to a rear vehicle by -1
    sign = -1 if recursion else 1
    return (distance * sign, True) if distance >= 0 else distance_between_vehicles(v2, v1, recursion=True)


def gap_between_vehicles(v1, v2):
    """
    Computes the gap between two vehicles
    :param v1: ID of the first vehicle
    :param v2: ID of the second vehicle
    :type v1: str
    :type v2: str
    :return: tuple with the gap between the two vehicles and whether this gap
    is valid or not
    :rtype: (float, bool)
    """
    distance, valid_distance = distance_between_vehicles(v1, v2)
    length = traci.vehicle.getLength(v2) if distance > 0 else traci.vehicle.getLength(v1)

    return abs(distance) - length, valid_distance


def first_of(vehicle_list):
    """
    Returns the ID of the first vehicle of the list
    :param vehicle_list: list of vehicles where to look for the first vehicle
    :type vehicle_list: list[str]
    :return: ID of the first vehicle of the list
    :rtype: str
    """
    distances = [0]  # Distance between the first vehicle and the first vehicle is zero
    """:type: list[float]"""

    for i in range(1, len(vehicle_list)):
        distances.append(distance_between_vehicles(vehicle_list[i], vehicle_list[0])[0])

    return vehicle_list[distances.index(min(distances))]


def sort_vehicle_list(vehicle_list):
    """
    Returns the vehicle list used as argument sorted by distance. Vehicle
    sorting by distance based on the accepted answer in:
    https://stackoverflow.com/questions/6618515/sorting-list-based-on-values-from-another-list
    :param vehicle_list: the vehicle list to be sorted
    :type vehicle_list: list[str]
    :return: sorted vehicle list
    :rtype: list[str]
    """
    distances = [0]  # Distance between the first vehicle and the first vehicle is zero
    """:type: list[float]"""

    for i in range(1, len(vehicle_list)):
        distances.append(distance_between_vehicles(vehicle_list[i], vehicle_list[0])[0])

    return [vehicle for _, vehicle in sorted(zip(distances, vehicle_list))]


def merge_platoons(p1, p2, lane_change=False):
    """
    Creates a new platoon from the two that merge and starts its merge
    maneuver, which is managed by the new Platoon object
    :param p1: one of the platoons that merge
    :param p2: the other platoon that merge
    :param lane_change: whether one of the platoons has to change its lane at
    the beginning of the maneuver
    :type p1: Platoon
    :type p2: Platoon
    :type lane_change: bool
    :return: the new platoon
    :rtype: Platoon
    """
    vehicle_list = []
    vehicle_list.extend(p1.get_members())
    vehicle_list.extend(p2.get_members())

    return Platoon(sort_vehicle_list(vehicle_list), desired_gap=p1.get_desired_gap(), safe_gap=p1.get_safe_gap(),
                   merging=True, lane_change=lane_change)


def all_edges_in_all_routes(edges, routes):
    """
    Checks if all the edges contained in the edges list are contained in all
    the routes of the routes list. The routes of the routes list are list of
    edges
    :param edges: list of edges to check
    :param routes: list of routes to check
    :type edges: list[str]
    :type routes: list[list[str]]
    :return: True if all edges of the edges list are contained in the routes
    list
    :rtype: bool
    """
    for route in routes:
        for edge in edges:
            if edge not in route:
                return False

    return True


def look_for_merges(platoons, max_distance, max_platoon_length, edge_filter, max_relative_speed):
    """
    Checks for the nearest platoon that can merge with each platoon. The merge
    candidate platoons are those that are closer than a maximum distance and
    placed on edges that are in the route of the platoon looking for a merge.
    Those candidates are discarded if there are other vehicles that represent
    an obstacle for the merge maneuver. This function also checks if it is
    necessary to perform a lane change at the beginning of the merge maneuver.
    :param platoons: list of platoons of the simulation
    :param max_distance: maximum distance to check for platoons that can merge
    :param max_platoon_length: maximum platoon length allowed in vehicles
    :param edge_filter: list of edges where platooning is allowed
    :param max_relative_speed: maximum relative speed allowed for merging (m/s)
    :type platoons: list[Platoon]
    :type max_distance: float
    :type max_platoon_length: int
    :type edge_filter: list[str]
    :type max_relative_speed: float
    :return: an list of indexes of platoons to merge, with -1 meaning that
    there is no merge for a platoon
    :rtype: list[int]
    """
    merges = [-1] * len(platoons)
    lane_changes = [False] * len(platoons)

    for i in range(len(platoons) - 1):
        if platoons[i].is_splitting():
            continue

        if platoons[i].leader_wants_to_leave(edge_filter) and platoons[i].length() == 1:
            continue

        min_distance = max_distance
        index = -1
        l1 = platoons[i].length()
        speed_1 = traci.vehicle.getSpeed(platoons[i].get_leader())
        max_speed_1 = traci.vehicle.getMaxSpeed(platoons[i].get_leader())
        allowed_speed_1 = traci.vehicle.getAllowedSpeed(platoons[i].get_leader())
        lane_1 = platoons[i].get_desired_lane()
        lane_id_1 = platoons[i].get_desired_lane_id()
        road_max_speed = traci.lane.getMaxSpeed(lane_id_1)
        routes = platoons[i].get_routes_list()
        edges_list = platoons[i].get_edges_list()

        for j in range(i + 1, len(platoons)):
            l2 = platoons[j].length()
            if platoons[j].is_splitting() or l1 + l2 > max_platoon_length:
                continue

            if platoons[j].leader_wants_to_leave(edge_filter) and platoons[j].length() == 1:
                continue

            distance = abs(platoons[i].distance_to(platoons[j]))
            if distance >= min_distance:
                continue

            speed_2 = traci.vehicle.getSpeed(platoons[j].get_leader())
            max_speed_2 = traci.vehicle.getMaxSpeed(platoons[j].get_leader())
            allowed_speed_2 = traci.vehicle.getAllowedSpeed(platoons[j].get_leader())
            if max_speed_1 < allowed_speed_2 or max_speed_2 < allowed_speed_1:
                continue

            if abs(speed_1 - speed_2) > max_relative_speed:
                continue

            neighbor_edges_list = platoons[j].get_edges_list()
            neighbor_routes = platoons[j].get_routes_list()

            if not all_edges_in_all_routes(neighbor_edges_list, routes) \
                    or not all_edges_in_all_routes(edges_list, neighbor_routes):
                continue

            lane_2 = platoons[j].get_desired_lane()
            lanes = [lane_1, lane_2]
            lane_id_2 = platoons[j].get_desired_lane_id()
            lane_ids = [lane_id_1, lane_id_2]
            desired_lane = max(lanes) if max_speed_1 > 0.9 * road_max_speed else min(lanes)
            desired_lane_id = lane_ids[lanes.index(desired_lane)]
            non_desired_lane_id = lane_id_1 if desired_lane == lane_2 else lane_id_2
            vehicles_in_lane = traci.lane.getLastStepVehicleIDs(desired_lane_id)

            max_pos = traci.vehicle.getLanePosition(first_of(platoons[i].get_members() + platoons[j].get_members()))
            l = platoons[i].length_sum() + platoons[j].length_sum() + (l1 + l2 + 1) * platoons[i].get_desired_gap()
            min_pos = min(max_pos - l, platoons[i].get_tail_position() - platoons[i].get_desired_gap(),
                          platoons[j].get_tail_position() - platoons[i].get_desired_gap())

            obstacle_vehicle = False
            for vehicle in vehicles_in_lane:
                pos = traci.vehicle.getLanePosition(vehicle)
                if min_pos < pos < max_pos and vehicle not in platoons[i] and vehicle not in platoons[j]:
                    obstacle_vehicle = True

            if obstacle_vehicle:
                continue

            change_lane = False
            overlapping = platoons[i].get_head_position() > platoons[j].get_tail_position()
            overlapping = overlapping and platoons[j].get_head_position() > platoons[i].get_tail_position()
            if not overlapping:
                p1 = traci.vehicle.getLanePosition(platoons[i].get_leader())
                p2 = traci.vehicle.getLanePosition(platoons[j].get_leader())

                vehicles_in_lane = traci.lane.getLastStepVehicleIDs(non_desired_lane_id)
                for vehicle in vehicles_in_lane:
                    pos = traci.vehicle.getLanePosition(vehicle)
                    # If the two platoons are not place one next to the other,
                    # one of them has to catch up the other. If there is a
                    # vehicle in the same lane than the platoon that has to
                    # change its lane it is safer to perform the lane change
                    # at the beginning of the merge maneuver (there are no
                    # obstacles for this lane change since it has been checked
                    # before)
                    if min(p1, p2) < pos < max(p1, p2) and vehicle not in platoons[i] and vehicle not in platoons[j]:
                        change_lane = True

            min_distance = distance
            index = j

            lane_changes[i] = True if change_lane else False

        if index != -1:
            merges[i] = index

    return merges, lane_changes


def it_is_safe_to_change_lane(vehicle, lane_id, safe_gap):
    """
    Checks whether it is safe or not for a vehicle to change to a lane
    :param vehicle: ID of the vehicle that wants to change lane
    :param lane_id: ID of the destination lane
    :param safe_gap: minimum gap to consider that the lane change is safe
    :type vehicle: str
    :type lane_id: str
    :type safe_gap: float
    :return: True if it is safe to change lane, False otherwise
    :rtype: bool
    """
    vehicles_in_lane = traci.lane.getLastStepVehicleIDs(lane_id)

    for v in vehicles_in_lane:
        gap, valid_gap = gap_between_vehicles(vehicle, v)
        if gap < safe_gap and valid_gap:
            return False

    return True


def already_in_lane(vehicle, lane):
    """
    Checks whether a vehicle is already in a lane
    :param vehicle: ID of the vehicle to check
    :param lane: index of the lane to check
    :type vehicle: str
    :type lane: int
    :return: True if the vehicle is already in the lane, False otherwise
    :rtype: bool
    """
    return True if traci.vehicle.getLaneIndex(vehicle) is lane else False
