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
fileName = 'mpu_data_gz.xlsx';
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
disp(['Dominant Frequency: ', num2str(dominantFrequency), ' Hz']);





