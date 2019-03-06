#!/bin/sh

netconvert -n merge.nod.xml -e merge.edg.xml -t merge.typ.xml -o merge.net.xml
python ../../simulator.py -c merge.sumo.cfg -g -ef 1to2 2to3 -vf type1 type2 -pl 8
