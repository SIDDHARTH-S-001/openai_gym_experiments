% port at which your arduino is connected

port = 'COM10';

% model of your arduino board

board = 'mega2560';

% creating arduino object with servo library

arduino_board = arduino(port, board, 'Libraries', 'Servo');

% creating servo motor object

servo_motor = servo(arduino_board, 'D9');

% loop to rotate servo motor from 0 to 180
%% 
for angle = 0:0.1:1 % from 0 to 1 with a step size of 0..1

   writePosition(servo_motor, angle);

   current_position = readPosition(servo_motor);

   current_position = current_position * 180;   

   % print current position of servo motor

   fprintf('Current position is %d\n', current_position);   

   % small delay is required so that servo can be positioned at the

   % angle told to it.

   pause(0.5);

end

% bring back motor to 0 angle position

writePosition(servo_motor, 0);
