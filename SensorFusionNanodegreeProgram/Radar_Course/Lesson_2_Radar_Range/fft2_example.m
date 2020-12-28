% 2-D Transform
% The 2-D Fourier transform is useful for processing 2-D signals and other 2-D data such as images.
% Create and plot 2-D data with repeated blocks.

P = peaks(20);
X = repmat(P,[5 10]);
imagesc(X)

% TODO : Compute the 2-D Fourier transform of the data.  
% Shift the zero-frequency component to the center of the output, and 
% plot the resulting 100-by-200 matrix, which is the same size as X.

Y = fft2(X);

% Shift the zero-frequency component to the center of the output
Y_shifted= fftshift(Y);

% Take the absolute value
Y_shifted_abs = abs(Y_shifted);

%imagesc(Y_shifted_abs)