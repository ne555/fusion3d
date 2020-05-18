#!/usr/bin/env python

import re

factor = 350/576

with open('cluster_tikz.tex') as entrada:
    lines = entrada.readlines()
    for l in lines:
        to_fix = re.match(r'^(.*?)([\d.+-]+pt)(.*)$', l)
        while to_fix:
            n = float(re.match(r'[\d.+-]+', to_fix[2])[0])
            n = n*factor
            print(to_fix[1], n, 'pt', end='', sep='')
            siguiente = re.match(r'^(.*?)([\d.+-]+pt)(.*)$', to_fix[3])
            if not siguiente:
                print(to_fix[3])
                break
            to_fix = siguiente
        else:
            print(l, end='')
