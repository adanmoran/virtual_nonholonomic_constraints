% Create the serial object
serialPort = 'COM5';
serialObject = serial(serialPort, 'BaudRate', 250000);
fopen(serialObject);

Ts = 0.002;

% SUGAR parameters
Rt = 0.148;
Rl = 0.145;
lt = 0.073;
ll = 0.083;
rt = Rt - lt;
rl = Rl - ll;
Mt = 0.2112;
Ml = 0.1979;
Jt = 0.00075;
Jl = 0.00129;
g = 9.8;
xg = 0;
zg = 0.50;

switchState = 0;
switchStateBot = 0;
psi = 0;
dpsi = 0;
alpha = 0;
dalpha = 0;
alpha_ref = 0;
psi_past = zeros(1,80);
dpsi_past = zeros(1,80);

xh = xg + Rt*sin(psi);
zh = zg - Rt*cos(psi);
xf = xh + Rl*sin(psi+alpha);
zf = zh - Rl*cos(psi+alpha);

E0 = -g*(Ml*(Rt + ll) + Mt*lt);
t = linspace(0, Ts*300, 300);
Ed = 0;
E = zeros(1,300);

fcount = 0;
fdraw = 3;

% make figure
figure('Color', 'white', 'Position', [160 90 1280 720]);
figureHandle = gcf;

subplot(2, 3, [1 2])

hh1 = plot([xg xh; xh xg], [zg zh; zh zf], '.-', 'MarkerSize', 25, 'LineWidth', 5);
xlabel('x (m)');
ylabel('z (m)');
axis([-1 1 0 1]);
axis equal

subplot(2,3,3)
hh2 = plot(psi_past, dpsi_past, 'Marker', '.', 'MarkerSize', 5, 'Color', 'b');
hh2.XDataSource = 'psi_past';
hh2.YDataSource = 'dpsi_past';
xlabel('\psi');
ylabel('d\psi');

axis([-pi, pi, -20 20]);

subplot(2,3,[4 5 6])
hold on
hh3 = plot(t, E, 'Marker', '.', 'MarkerSize', 5, 'Color', 'b');
hh3.XDataSource = 't';
hh3.YDataSource = 'E';
hh4 = line([t(1) t(end)], [0 0], 'Color', 'r');
hold off

linkdata on

while isgraphics(figureHandle)
    headerFound = false;
    while ~headerFound
        header1 = fread(serialObject,1,'uint8');
        %error checking
        if header1 ~= 250
            while header1 ~= 250
               header1 = fread(serialObject,1,'uint8');
            end
        end
        
        header2 = fread(serialObject,1,'uint8');
        if header2 == 206
            headerFound = true;
        end
    end
    
    switchState = fread(serialObject,1,'uint8');
    switchStateBot = fread(serialObject,1,'uint8');
    Ed = fread(serialObject,1,'float32');
    psi = fread(serialObject,1,'float32');
    dpsi = fread(serialObject,1,'float32');
    alpha = fread(serialObject,1,'float32');
    dalpha = fread(serialObject,1,'float32');
    alpha_ref = fread(serialObject,1,'float32');
    
    E = circshift(E, [0 1]);
    E(end) = fread(serialObject,1,'float32');
    
    psi = atan2(sin(psi), cos(psi));
    
    psi_past = circshift(psi_past, [0 1]);
    psi_past(1) = psi;
    
    dpsi_past = circshift(dpsi_past, [0 1]);
    dpsi_past(1) = dpsi;
    
    t_last = t(end);
    t = circshift(t, [0 -1]);
    t(end) = t_last + Ts;
    
    fcount = fcount + 1;
    if fcount > fdraw
        fcount = 0;
        xh = xg + Rt*sin(psi);
        zh = zg - Rt*cos(psi);
        xf = xh + Rl*sin(psi+alpha);
        zf = zh - Rl*cos(psi+alpha);


        set(hh1(1), 'XData', [xg, xh], 'YData', [zg, zh])
        set(hh1(2), 'XData', [xh, xf], 'YData', [zh, zf])
        hh3 = plot(t, E, 'Marker', '.', 'MarkerSize', 5, 'Color', 'b');
        hh4 = line([t(1) t(end)], [Ed Ed]);
        axis([t(1) t(end) 0 3]);
        refreshdata(hh2);

        drawnow;
    end
end

fclose(serialObject);
