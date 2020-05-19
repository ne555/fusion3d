#/* vim: set filetype=gnuplot:*/
set terminal latex rotate
set output 'dif_rot_happy.tex'
#set terminal dumb ansirgb
set xlabel 'captura'
set ylabel 'ángulo (grados)'
unset key

plot 'data/dif_rot_happy' using 2:xtic(1) with histogram, \
	 'data/dif_rot_happy' using 3 with histogram fs pattern 1
