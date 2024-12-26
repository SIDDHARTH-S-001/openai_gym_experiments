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
while toc(startTime) < 3
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



