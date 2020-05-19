#/* vim: set filetype=gnuplot:*/

set terminal latex rotate
set output 'fitness.tex'
#set terminal dumb ansirgb
set xlabel 'captura'
set ylabel 'solapamiento'
set yrange [0:1]
unset key

plot 'data/dragon_fitness' using 2:xtic(1) with histogram
