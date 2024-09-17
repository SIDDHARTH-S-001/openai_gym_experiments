function [f, amp] = fft_code(data, tsamp)

Fs = 1/tsamp;           % Sample frequency
T = tsamp;              % Sample Time
L = length(data);       % Data Length

x = data;               % Data

NFFT = 2^nextpow2(L);   % Next power of 2 from length of x
Y = fft(x, NFFT)/L;     % Perform FFT
f = Fs/2*linspace(0, 1, NFFT/2+1);  % Generate frequency vector

% Plot single-sided amplitude spectrum
amp = 2*abs(Y(1:NFFT/2+1));
loglog(f, amp)
title('Single-Sided Amplitude Spectrum of data(t)')
xlabel('Frequency (Hz)')
ylabel('Data(f)')
