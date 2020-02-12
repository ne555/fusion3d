#!/bin/bash

#sin corrección de bucle

mkdir --parents result/ArmadilloBack.conf result/ArmadilloOnHeadMultiple.conf result/ArmadilloOnHeadMultipleOffset.conf result/ArmadilloStand.conf result/ArmadilloStandFlip.conf result/bun.conf result/dragonSideRight.conf result/dragonStandRight.conf result/dragonUpRight.conf result/drill_1.6mm_cyb.conf result/happyBackRight.conf result/happySideRight.conf result/happyStandRight.conf

#ejecución


##bunny
echo "bunny"
time ./registrar database/bunny/data/ bun.conf sin_correcion_de_bucle

#armadillo
echo "armadillo"
for K in {ArmadilloBack.conf,ArmadilloOnHeadMultiple.conf,ArmadilloOnHeadMultipleOffset.conf,ArmadilloStand.conf,ArmadilloStandFlip.conf}; do
	time ./registrar database/Armadillo_scans/ "$K" sin_correcion_de_bucle
done

##dragon
#dragonSideRight.conf, dragonStandRight.conf, dragonUpRight.conf
echo "dragon"
time ./registrar database/dragon/dragon_side/ dragonSideRight.conf sin_correcion_de_bucle
time ./registrar database/dragon/dragon_stand/ dragonStandRight.conf sin_correcion_de_bucle
time ./registrar database/dragon/dragon_up/ dragonUpRight.conf sin_correcion_de_bucle

##drill
echo "drill"
time ./registrar database/drill/data/ drill_1.6mm_cyb.conf sin_correcion_de_bucle

##happy
#happyBackRight.conf, happySideRight.conf, happyStandRight.conf
echo "happy"
time ./registrar database/happy/happy_back/ happyBackRight.conf sin_correcion_de_bucle
time ./registrar database/happy/happy_side/ happySideRight.conf sin_correcion_de_bucle
time ./registrar database/happy/happy_stand/ happyStandRight.conf sin_correcion_de_bucle
