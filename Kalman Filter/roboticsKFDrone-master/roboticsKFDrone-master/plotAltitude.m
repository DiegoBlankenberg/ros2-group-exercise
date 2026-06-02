%% plot_altitude.m
% Plot barometer altitude and estimated altitude from ArduPilot BIN log
% Requires pymavlink installed: pip install pymavlink --break-system-packages

%% Load the log file
filename = 'solar.bin';  % <-- change this to your .bin file path
mlog = py.pymavlink.mavutil.mavlink_connection(filename);

%% Extract BARO data
mlog.rewind();
baro_time = [];
baro_alt  = [];
while true
    msg = mlog.recv_match(pyargs('type','BARO','blocking',false));
    if isempty(msg) || isa(msg, 'py.NoneType')
        break;
    end
    d = msg.to_dict();
    baro_time(end+1) = double(d{'TimeUS'});
    baro_alt(end+1)  = double(d{'Alt'});
end

%% Extract POS data (EKF estimated position)
mlog.rewind();
pos_time = [];
pos_alt  = [];
pos_rel  = [];
while true
    msg = mlog.recv_match(pyargs('type','POS','blocking',false));
    if isempty(msg) || isa(msg, 'py.NoneType')
        break;
    end
    d = msg.to_dict();
    pos_time(end+1) = double(d{'TimeUS'});
    pos_alt(end+1)  = double(d{'Alt'});
    pos_rel(end+1)  = double(d{'RelHomeAlt'});
end

%% Extract GPS data
mlog.rewind();
gps_time = [];
gps_alt  = [];
while true
    msg = mlog.recv_match(pyargs('type','GPS','blocking',false));
    if isempty(msg) || isa(msg, 'py.NoneType')
        break;
    end
    d = msg.to_dict();
    gps_time(end+1) = double(d{'TimeUS'});
    gps_alt(end+1)  = double(d{'Alt'});
end

%% Convert timestamps from microseconds to seconds (relative to start)
t0 = min([baro_time(1), pos_time(1), gps_time(1)]);
baro_t_sec = (baro_time - t0) / 1e6;
pos_t_sec  = (pos_time - t0) / 1e6;
gps_t_sec  = (gps_time - t0) / 1e6;

%% Plot
figure;

subplot(3,1,1);
plot(baro_t_sec, baro_alt, 'b', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Barometer Altitude');
grid on;

subplot(3,1,2);
plot(gps_t_sec, gps_alt, 'g', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Altitude (m)');
title('GPS Altitude');
grid on;

subplot(3,1,3);
plot(pos_t_sec, pos_rel, 'r', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Altitude (m)');
title('EKF Estimated Altitude (Relative to Home)');
grid on;

sgtitle('ArduPilot Flight Altitude Data');