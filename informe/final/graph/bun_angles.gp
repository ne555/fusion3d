#/* vim: set filetype=gnuplot:*/

set terminal tikz monochrome
set output 'bun_angles.tex'
#set terminal dumb ansirgb
#set terminal png
#set output 'foo.png'

set xlabel 'ángulo de giro (grados)'
set ylabel 'frecuencia'

unset key

binwidth = 2
binstart = 15
load 'hist.fct'
plot 'data/bun_angles.graph' i 0 @hist
