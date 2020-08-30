1;

x = load('data/bun_angles.graph');
x = x(x<90);
%nbins = (max(x) - min(x)) / 2;
nbins = 50;
[nn, xx] = hist(x, nbins, 1);
result = [xx' nn'];
save 'data/bun_angles.hist' result
