% list_ardupilot_message_types.m
% List all unique message types in an ArduPilot .bin log using pymavlink
% Requires pymavlink installed and accessible from MATLAB

filename = 'solar.bin';
mlog = py.pymavlink.mavutil.mavlink_connection(filename);

msg_types = {};
mlog.rewind();
while true
    msg = mlog.recv_match(pyargs('blocking', false));
    if isempty(msg) || isa(msg, 'py.NoneType')
        break;
    end
    msg_type = char(msg.get_type());
    if ~ismember(msg_type, msg_types)
        msg_types{end+1} = msg_type; %#ok<AGROW>
    end
end

msg_types = sort(msg_types);
disp('Unique message types in the log:');
disp(msg_types');

% Save to a text file
fid = fopen('ardupilot_message_types.txt', 'w');
for i = 1:length(msg_types)
    fprintf(fid, '%s\n', msg_types{i});
end
fclose(fid);

disp('Message types saved to ardupilot_message_types.txt');
