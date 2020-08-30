#/* vim: set filetype=gnuplot:*/

set terminal tikz monochrome
set output 'bun_translation.tex'
#set terminal dumb ansirgb
#set terminal png
#set output 'foo.png'

set xlabel 'x'
set ylabel 'z'

#set format x "$10^{%L}$"
#set format y "$10^{%L}$"

unset key

plot 'data/bun.translation.2p' using 1:3 with points pt 7
