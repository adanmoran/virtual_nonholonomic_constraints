

%% Create the serial object
serialPort = 'COM5';
serialObject = serial(serialPort, 'BaudRate', 250000);
fopen(serialObject);

Tstop = 30;
Ts = 0.002;

%% Collect data
n = Tstop/(Ts*8);
time = zeros(1,n);
switchState = zeros(1,n);
switchStateBot = zeros(1,n);
E0 = zeros(1,n);
psi = zeros(1,n);
dpsi = zeros(1,n);
alpha = zeros(1,n);
dalpha = zeros(1,n);
alpha_ref = zeros(1,n);
E = zeros(1,n);

i = 1;
while i <= n
    time(i) = (i-1)*Ts*8;

    headerFound = false;
    while ~headerFound
        header1 = fread(serialObject,1,'uint8');
        %error checking
        if header1 ~= 250
            switchState(i) = NaN;
            switchStateBot(i) = NaN;
            psi(i) = NaN;
            dpsi(i) = NaN;
            alpha(i) = NaN;
            dalpha(i) = NaN;
            alpha_ref(i) = NaN;
            while header1 ~= 250
               header1 = fread(serialObject,1,'uint8');
            end
            i = i + 1;
            time(i) = (i-1)*Ts*8;
        end
        
        header2 = fread(serialObject,1,'uint8');
        if header2 == 206
            headerFound = true;
        end
    end
    
    switchState(i) = fread(serialObject,1,'uint8');
    switchStateBot(i) = fread(serialObject,1,'uint8');
    E0(i) = fread(serialObject,1,'float32');
    psi(i) = fread(serialObject,1,'float32');
    dpsi(i) = fread(serialObject,1,'float32');
    alpha(i) = fread(serialObject,1,'float32');
    dalpha(i) = fread(serialObject,1,'float32');
    alpha_ref(i) = fread(serialObject,1,'float32');
    E(i) = fread(serialObject,1,'float32');
    
    i = i + 1;
end

hold on
plot(time, psi, 'r')
plot(time, dpsi, 'b')
plot(time, alpha, 'y')
plot(time, dalpha, 'g')
plot(time, switchState, 'k')
hold off

%% Clean up the serial object
fclose(serialObject);
clear serialObject serialPort h1 h2 h3 h4 h5 Tstop Ts i n w scrsz header1 header2 headerFound;
