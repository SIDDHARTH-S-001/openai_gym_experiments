s = serial('port_number', 'BaudRate', 115200);
fopen(s);

for i = 1:17100
    realGyroData(i) = fscanf(s, '%d');
end

fclose(s);