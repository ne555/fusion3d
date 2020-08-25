#/* vim: set filetype=gnuplot:*/

set terminal tikz
set output 'dif_rot_bun.tex'
#set terminal dumb ansirgb
set xlabel 'captura'
set ylabel 'ángulo (grados)'
unset key

plot 'data/bun.angles' using (abs($2-$3)):xtic(1) with histogram
