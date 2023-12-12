alt=344;
origin = [mean(N), mean(W), alt];
[xEast,yNorth] = latlon2local(N,W,alt,origin);
figure
plot(xEast,yNorth)
std(xEast)
std(yNorth)