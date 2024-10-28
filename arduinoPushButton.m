
a = arduino('COM3', 'Uno');

while true

    readDigitalPin(arduino, 'D9');
end
 
pause(0.01); % Short pause to reduce CPU usage

