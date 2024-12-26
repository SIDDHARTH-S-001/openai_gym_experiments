%% Reading data and storing it in excel file.
% Specify the serial port and baud rate
serialPort = 'COM10'; % Replace with your serial port
baudRate = 115200; % Set baud rate (match your device)

% Create a serial object
s = serialport(serialPort, baudRate);

% Set the timeout for serial communication
s.Timeout = 1; % 1 second timeout for reading

% Initialize variables
data = []; % Array to store the received data
startTime = tic; % Start the timer

disp('Reading data from serial port...');

% Read data for 3 seconds
while toc(startTime) < 180
    if s.NumBytesAvailable > 0
        % Read a line of data
        line = readline(s);
        % Convert the line to numeric and store it
        value = str2double(line);
        if ~isnan(value) % Check if conversion was successful
            data = [data; value];
        end
    end
end

disp('Finished reading data.');

% Close the serial port
clear s;

% Save data to an Excel file
fileName = 'mpu_data_gz_5Hz_LPF.xlsx';
writematrix(data, fileName);

disp(['Data saved to Excel file: ', fileName]);

% Display the list of values
disp('List of values:');
disp(data);

%% Processing
mean_gz = mean(data);
data_gz_unbiased = data - mean_gz; % should be 0 ideally.
% run a FFT to find the frequency response of the unbiased data.
% timestamps = (0:length(data)-1) * sampling_interval; % if interval is known
% Fs = 1 / mean(diff(timestamps)); % Calculate sampling frequency
Fs = length(data_gz_unbiased)/180;

% Time vector
N = length(data_gz_unbiased); % Number of samples
t = (0:N-1) / Fs; % Time vector

% Perform FFT
Y = fft(data_gz_unbiased); % Compute the FFT
Y_mag = abs(Y); % Magnitude of FFT
Y_phase = angle(Y); % Phase of FFT

% Frequency vector
f = (0:N-1)*(Fs/N); % Frequency vector
f = f(1:floor(N/2)); % Retain positive frequencies only
Y_mag = Y_mag(1:floor(N/2)); % Magnitude for positive frequencies

% Plot the frequency response
figure;
plot(f, Y_mag);
xlabel('Frequency (Hz)');
ylabel('Amplitude');
title('Frequency Response of Gyroscope Data');
grid on;

% Optional: Display dominant frequency
[~, maxIdx] = max(Y_mag); % Find the index of the max amplitude
dominantFrequency = f(maxIdx); % Get the corresponding frequency
disp(['Dominant Frequency: ', num2str(dominantFrequency), ' Hz']); % 0.022222 Hz

%% Low Pass Filter

% Define the cutoff frequency and sampling frequency
Fc = 5; % Cutoff frequency (Hz), adjust based on FFT observation (obtained from datasheet).
Fs = length(data_gz_unbiased)/180;

% Design a low-pass filter using a Butterworth filter
% [b, a] = butter(4, Fc/(Fs/2)); % 4th-order Butterworth filter
[b, a] = butter(6, 5/(Fs/2)); % 6th-order Butterworth filter with 5 Hz cutoff

% Apply the filter to your data
filtered_data = filtfilt(b, a, data_gz_unbiased);

% Plot the original and filtered signals for comparison
figure;
subplot(2, 1, 1);
plot(data_gz_unbiased);
title('Original Gyroscope Data');
xlabel('Samples');
ylabel('Amplitude');

subplot(2, 1, 2);
plot(filtered_data);
title('Filtered Gyroscope Data');
xlabel('Samples');
ylabel('Amplitude');

%% Digital Low Pass Filter

Fs = length(data_gz_unbiased)/180; % Sampling frequency (Hz), based on your earlier calculation

% Specify cutoff frequency
Fc = 5; % Desired cutoff frequency (Hz)

% Normalize the cutoff frequency
Wn = Fc / (Fs / 2); % Normalized cutoff frequency (0 to 1)

% Design a digital low-pass filter
% Example 1: FIR Filter (Finite Impulse Response)
fir_order = 50; % Filter order for FIR
b = fir1(fir_order, Wn, 'low'); % FIR filter coefficients

% Example 2: IIR Filter (Infinite Impulse Response) - Butterworth
iir_order = 4; % Filter order for IIR
[b_iir, a_iir] = butter(iir_order, Wn, 'low'); % IIR filter coefficients

% Apply the filters to the gyroscope data
filtered_data_fir = filter(b, 1, data_gz_unbiased); % FIR filter application
filtered_data_iir = filter(b_iir, a_iir, data_gz_unbiased); % IIR filter application

% Plot the original and filtered data
figure;
subplot(3, 1, 1);
plot(data_gz_unbiased);
title('Original Gyroscope Data');
xlabel('Samples');
ylabel('Amplitude');

subplot(3, 1, 2);
plot(filtered_data_fir);
title('Filtered Data (FIR)');
xlabel('Samples');
ylabel('Amplitude');

subplot(3, 1, 3);
plot(filtered_data_iir);
title('Filtered Data (IIR)');
xlabel('Samples');
ylabel('Amplitude');

% Compare the FFT of original and filtered signals
Y_orig = abs(fft(data_gz_unbiased));
Y_fir = abs(fft(filtered_data_fir));
Y_iir = abs(fft(filtered_data_iir));
frequencies = (0:length(Y_orig)-1) * Fs / length(Y_orig);

figure;
plot(frequencies(1:floor(end/2)), Y_orig(1:floor(end/2)), 'k', 'DisplayName', 'Original');
hold on;
plot(frequencies(1:floor(end/2)), Y_fir(1:floor(end/2)), 'b', 'DisplayName', 'FIR Filtered');
plot(frequencies(1:floor(end/2)), Y_iir(1:floor(end/2)), 'r', 'DisplayName', 'IIR Filtered');
title('Frequency Response of Original and Filtered Data');
xlabel('Frequency (Hz)');
ylabel('Magnitude');
legend;
grid on;

%% Computing Rate Noise Density

% Sampling frequency
Fs = length(data_gz_unbiased)/180; % Sampling frequency (Hz) based on earlier calculation

% Time interval between samples
dt = 1 / Fs;

% Number of samples and total duration
N = length(data_gz_unbiased); % Total number of samples
T_total = N * dt; % Total duration (seconds)

% Time cluster sizes (tau) for Allan Variance calculation
max_clusters = floor(N / 2); % Maximum number of clusters
taus = dt * (1:max_clusters); % Cluster times (seconds)

% Preallocate Allan Variance array
allan_variance = zeros(size(taus));

% Compute Allan Variance
for k = 1:length(taus)
    m = floor(N / (2 * k)); % Number of clusters for this tau
    clusters = reshape(data_gz_unbiased(1:m*2*k), [2*k, m]); % Divide data into clusters
    cluster_means = mean(clusters); % Compute cluster means
    allan_variance(k) = 0.5 * mean(diff(cluster_means).^2); % Allan Variance
end

% Allan Deviation (square root of Allan Variance)
allan_deviation = sqrt(allan_variance);

% Plot Allan Deviation
figure;
loglog(taus, allan_deviation, 'b-', 'LineWidth', 1.5);
grid on;
title('Allan Deviation');
xlabel('Cluster Time (\tau) [s]');
ylabel('Allan Deviation');

% Rate Noise Density Estimation
% Identify the region where the Allan Deviation curve is flat (white noise)
% Rate Noise Density is derived from the Allan Deviation in this region
% (Standard deviation divided by sqrt(tau))
% Select tau where the slope is -0.5 (white noise region)
white_noise_region = taus(allan_deviation == min(allan_deviation)); % Adjust this based on visual observation
rate_noise_density = min(allan_deviation) / sqrt(white_noise_region);

% Display Rate Noise Density
disp(['Rate Noise Density: ', num2str(rate_noise_density), ' rad/s/√Hz']);
% Rate Noise Density: 0.00026122 rad/s/√Hz


