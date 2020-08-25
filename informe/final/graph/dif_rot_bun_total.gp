#/* vim: set filetype=gnuplot:*/

set terminal tikz monochrome
set output 'dif_rot_bun_total.tex'
#set terminal dumb ansirgb
set xlabel 'captura'
set ylabel 'ángulo (grados)'
unset key

plot 'data/bun_dif_total.angles' using 2:xtic(1) with histogram
