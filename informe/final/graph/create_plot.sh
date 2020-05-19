#!/bin/bash

for K in *.gp; do
	echo "$K"
	gnuplot "$K"
done
