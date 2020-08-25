#/* vim: set filetype=gnuplot:*/

set terminal tikz monochrome
set output 'dif_rot_happy_sac.tex'
#set terminal dumb ansirgb
set xlabel 'captura'
set ylabel 'ángulo (grados)'
unset key

plot 'data/happy.angles' using (abs($3-$4)):xtic(1) with histogram
