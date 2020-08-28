#/* vim: set filetype=gnuplot:*/

set terminal tikz monochrome
#set terminal png
#set output 'foo.png'
set output 'registration_order.tex'
#set terminal dumb ansirgb
set xlabel 'puntos'
set ylabel 'tiempo'
set format x "$10^{%L}$"
set format y "$10^{%L}$"
set logscale xy
unset key

f(x) = a*x**2 + b*x + c
fit [10e3:100e3] f(x) 'data/registration.times' using 1:2 via a, b, c

plot [10e3:100e3] 'data/registration.times' pt 7 notitle, f(x) dt 1 title "O(n^2)"
