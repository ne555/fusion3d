#!/bin/bash

for K in ../result.registration/*.conf; do
	conf=$(basename $K)
	echo $conf
	./build/fitness $(ls -v ../result.registration/${conf}/*.ply) 2>/dev/null 1>fitness.log/${conf}
done
