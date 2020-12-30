R = 300; %m 
resolution = 1; %m
c = 3e8; %m/s

% TODO : Find the Bsweep of chirp for 1 m resolution

Bsweep = c/(2*resolution);

% TODO : Calculate the chirp time based on the Radar's Max Range
Tchirp = 5.5*2*R/c;

% TODO : define the frequency shifts 
fb = [0, 1.1, 13, 24]*1e6;

calculated_range = (c*fb*Tchirp)/(2*Bsweep);
% Display the calculated range
disp(calculated_range);

