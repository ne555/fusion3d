1;

hf = figure();
x = load('bun_angles.graph');
nbins = (max(x) - min(x)) / 2;
hist(x, nbins, 1);
colormap(white());

xlabel('ángulo de giro (grados)')
ylabel('frecuencia')
%set(hf, "papersize", [100, 100]);
%print (hf, "bun_angle", "-dsvg");
%print (hf, "ángulo", "-dpdflatexstandalone");
print (hf, "bun_angle", "-S350,200",  "-dtikz");
%print (hf, "bun_angle", "-dpdf");
