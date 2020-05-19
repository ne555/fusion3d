#/* vim: set filetype=gnuplot:*/

set terminal latex rotate
set output 'dif_rot_happy.tex'
#set terminal dumb ansirgb
set xlabel 'captura'
set ylabel 'ángulo (grados)'
unset key

plot 'data/happy.angles' using (abs($2-$4)):xtic(1) with histogram, \
	 'data/happy.angles' using (abs($3-$4)):xtic(1) with histogram fs pattern 1
