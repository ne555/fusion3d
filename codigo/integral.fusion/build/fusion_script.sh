#!/bin/bash

mkdir --parents result
echo > result/times

scan_dir=~/test/fusion_3d/database
conf_dir=~/test/fusion_3d/codigo/result.registration

#ejecución
##bunny
echo "bunny" | tee --append result/times
{ time ./fusion ${scan_dir}/bunny/data/ ${conf_dir}/bun.conf/result result/bun; } 2>> result/times

#armadillo
echo "armadillo" | tee --append result/times
for K in {ArmadilloBack.conf,ArmadilloOnHeadMultiple.conf,ArmadilloOnHeadMultipleOffset.conf,ArmadilloStand.conf,ArmadilloStandFlip.conf}; do
	{ time ./fusion ${scan_dir}/Armadillo_scans/ ${conf_dir}/${K}/result result/${K%.conf}; } 2>> result/times
done

##dragon
#dragonSideRight.conf, dragonStandRight.conf, dragonUpRight.conf
echo "dragon" | tee --append result/times
{ time ./fusion ${scan_dir}/dragon/dragon_side/ ${conf_dir}/dragonSideRight.conf/result result/dragon_side; } 2>> result/times
{ time ./fusion ${scan_dir}/dragon/dragon_stand/ ${conf_dir}/dragonStandRight.conf/result result/dragon_stand; } 2>> result/times
{ time ./fusion ${scan_dir}/dragon/dragon_up/ ${conf_dir}/dragonUpRight.conf/result result/dragon_up; } 2>> result/times

##drill
echo "drill" | tee --append result/times
{ time ./fusion ${scan_dir}/drill/data/ ${conf_dir}/drill_1.6mm_cyb.conf/result result/drill; } 2>> result/times

##happy
#happyBackRight.conf, happySideRight.conf, happyStandRight.conf
echo "happy" | tee --append result/times
{ time ./fusion ${scan_dir}/happy/happy_back/ ${conf_dir}/happyBackRight.conf/result result/happy_back; } 2>> result/times
{ time ./fusion ${scan_dir}/happy/happy_side/ ${conf_dir}/happySideRight.conf/result result/happy_side; } 2>> result/times
{ time ./fusion ${scan_dir}/happy/happy_stand/ ${conf_dir}/happyStandRight.conf/result result/happy_stand; } 2>> result/times
