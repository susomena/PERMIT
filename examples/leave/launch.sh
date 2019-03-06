#!/bin/sh

netconvert -n leave.nod.xml -e leave.edg.xml -t leave.typ.xml -x leave.con.xml -o leave.net.xml
python ../../simulator.py -c leave.sumo.cfg -g -ef 1to2 2to3 -vf type1 type2 -pl 8
