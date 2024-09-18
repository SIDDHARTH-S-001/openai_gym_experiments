s = serial('port_number', 'BaudRate', 115200);
fopen(s);
sample_rate = 1000; % Hz
wait_time = 3*60; % secs (converting minutes to secs)
num_readings = sample_rate * wait_time;
for i = 1:num_readings
    realGyroData(i) = fscanf(s, '%d');
end

fclose(s);

% plot the readings to get an idea (ensure sensor was fixed on spot the whole time)
% mean of readings gives the static bias (convert to degrees/s aka dps)
% subtract the mean (static bias) to get unbiased data (still contains noise)
% run a FFT on the unbiased data to get the frequency content of just the noise.
% to find rate-noise density, use Alan Density of the signal.