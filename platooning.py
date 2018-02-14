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


class Platoon:

    """

    Class for the management of a platoon. This simulator considers that every
    CACC enabled vehicle is already a platoon composed by just one vehicle.

    """

    def __init__(self, vehicle):
        """
        Constructor of the Platoon class. Every platoon is initialized with a
        first vehicle. This is because this simulator considers that every CACC
        enabled vehicle is already a platoon composed by just one vehicle
        :param vehicle: the first vehicle that composes the platoon
        """
        self._topology = {"leader": vehicle}
        self._members = [vehicle]

    def __contains__(self, vehicle):
        return vehicle in self._members


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
