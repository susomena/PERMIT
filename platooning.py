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

import ccparams as cc
import utils


class Platoon:

    """

    Class for the management of a platoon. This simulator considers that every
    CACC enabled vehicle is already a platoon composed by just one vehicle.

    """

    def __init__(self, vehicles):
        """
        Constructor of the Platoon class. Every platoon is initialized with a
        ordered list of vehicles. The first vehicle is the platoon leader and
        uses an ACC controller, while the followers use a CACC controller. If
        the vehicles list is composed by just one vehicle then a platoon of one
        vehicle is created.
        :param vehicles: list of vehicles that compose the platoon
        """
        self._topology = dict()
        utils.set_par(vehicles[0], cc.PAR_ACTIVE_CONTROLLER, cc.ACC)

        for i in range(1, len(vehicles)):
            self._topology[vehicles[i]] = {"leader": vehicles[0], "front": vehicles[i - 1]}
            utils.set_par(vehicles[i], cc.PAR_ACTIVE_CONTROLLER, cc.CACC)

        self._members = vehicles

    def __contains__(self, vehicle):
        return vehicle in self._members

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
            utils.set_par(vid, cc.PAR_LEADER_SPEED_AND_ACCELERATION, leader_data)
            utils.set_par(vid, cc.PAR_PRECEDING_SPEED_AND_ACCELERATION, front_data)
            # compute GPS distance and pass it to the fake CACC
            f_d = utils.get_distance(vid, links["front"])
            utils.set_par(vid, cc.PAR_LEADER_FAKE_DATA, cc.pack(l_v, l_u))
            utils.set_par(vid, cc.PAR_FRONT_FAKE_DATA, cc.pack(f_v, f_u, f_d))


def in_platoon(platoons, vehicle):
    """
    Checks if a vehicle is a member of any platoon in a list of platoons
    :param platoons: list of platoons
    :param vehicle: vehicle to check if it is a member of a platoon of the list
    :return: True if the vehicle is a member of a platoon of the list, False if
    not.
    """
    for platoon in platoons:
        if vehicle in platoon:
            return True

    return False
