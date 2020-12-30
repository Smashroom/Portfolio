% Doppler Velocity Calculation
c = 3*10^8;         %speed of light
frequency = 77e9;   %frequency in Hz

% TODO : Calculate the wavelength
lambda = c/frequency;

fprintf("Wavelength is: %1f\n", lambda);
% TODO : Define the doppler shifts in Hz using the information from above 
f_doppler = [3 -4.5 11 -3]*1e3; %Hz

% TODO : Calculate the velocity of the targets  fd = 2*vr/lambda
vr = f_doppler*lambda/2;


% TODO: Display results
disp(vr);
