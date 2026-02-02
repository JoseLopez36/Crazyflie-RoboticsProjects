clear all
close all
clc

% Parameters
bParsePose = true;
bParseSetpoint = true;
time_cutoff_ratio = 0.77;

% Load the rosbag
rosbag_path = fullfile('..', 'experiments', 'rosbag_success', 'test_rosbag_0.db3');

if ~isfile(rosbag_path)
    warning('Rosbag file not found at: %s', rosbag_path);
end

try
    bag = ros2bagreader(rosbag_path);
catch ME
    error('Failed to load rosbag: %s. \nMake sure you have the ROS Toolbox installed.', ME.message);
end

% Topics
topics_pose = ["/cf_0/pose", "/cf_1/pose"];
topics_setpoint = ["/cf_0/control/setpoint", "/cf_1/control/setpoint"];
names = {'cf_0', 'cf_1'};

% Data containers
data_pose = containers.Map;
data_setpoint = containers.Map;

% Parse Pose Data
if bParsePose
    for i = 1:length(topics_pose)
        topic = topics_pose(i);
        fprintf('Parsing topic: %s\n', topic);
        try
            bag_sel = select(bag, "Topic", topic);
            if bag_sel.NumMessages > 0
                d = ParsePose(bag_sel);
                % Apply time cutoff
                if ~isempty(d.time)
                    start_t = d.time(1);
                    end_t = d.time(end);
                    cutoff_t = start_t + (end_t - start_t) * time_cutoff_ratio;
                    idx = d.time <= cutoff_t;
                    d.time = d.time(idx);
                    d.x = d.x(idx);
                    d.y = d.y(idx);
                    d.z = d.z(idx);
                end
                data_pose(topic) = d;
            else
                warning('No messages found for topic: %s', topic);
            end
        catch ME
            warning('Error parsing topic %s: %s', topic, ME.message);
        end
    end
end

% Parse Setpoint Data
if bParseSetpoint
    for i = 1:length(topics_setpoint)
        topic = topics_setpoint(i);
        fprintf('Parsing topic: %s\n', topic);
        try
            bag_sel = select(bag, "Topic", topic);
            if bag_sel.NumMessages > 0
                d = ParsePoint(bag_sel);
                % Apply time cutoff
                if ~isempty(d.time)
                    start_t = d.time(1);
                    end_t = d.time(end);
                    cutoff_t = start_t + (end_t - start_t) * time_cutoff_ratio;
                    idx = d.time <= cutoff_t;
                    d.time = d.time(idx);
                    d.x = d.x(idx);
                    d.y = d.y(idx);
                    d.z = d.z(idx);
                end
                data_setpoint(topic) = d;
            else
                warning('No messages found for topic: %s', topic);
            end
        catch ME
            warning('Error parsing topic %s: %s', topic, ME.message);
        end
    end
end

% Calculate Metrics
fprintf('\n================================ METRICS ================================\n');
fprintf('%-10s %-15s %-15s %-15s %-15s\n', 'Robot', 'Duration [s]', 'Distance [m]', 'Avg Speed [m/s]', 'RMSE [m]');

for i = 1:length(topics_pose)
    t_pose = topics_pose(i);
    t_setpoint = topics_setpoint(i);
    name = names{i};
    
    duration = NaN;
    distance = NaN;
    avg_speed = NaN;
    rmse = NaN;
    
    if isKey(data_pose, t_pose)
        d_p = data_pose(t_pose);
        
        % Duration
        if length(d_p.time) > 1
            duration = d_p.time(end) - d_p.time(1);
            
            % Distance
            dx = diff(d_p.x);
            dy = diff(d_p.y);
            dz = diff(d_p.z);
            dist_segments = sqrt(dx.^2 + dy.^2 + dz.^2);
            distance = sum(dist_segments);
            
            % Avg Speed
            if duration > 0
                avg_speed = distance / duration;
            end
        end
        
        % RMSE (assume aligned indices)
        if isKey(data_setpoint, t_setpoint)
            d_s = data_setpoint(t_setpoint);
            
            % Use minimum length to avoid index mismatch
            n = min(length(d_p.x), length(d_s.x));
            
            if n > 0
                % Truncate to common length
                p_x = d_p.x(1:n);
                p_y = d_p.y(1:n);
                p_z = d_p.z(1:n);
                
                s_x = d_s.x(1:n);
                s_y = d_s.y(1:n);
                s_z = d_s.z(1:n);
                
                err_x = p_x - s_x;
                err_y = p_y - s_y;
                err_z = p_z - s_z;
                
                sq_err = err_x.^2 + err_y.^2 + err_z.^2;
                rmse = sqrt(mean(sq_err));
            end
        end
    end
    
    fprintf('%-10s %-15.2f %-15.2f %-15.2f %-15.4f\n', name, duration, distance, avg_speed, rmse);
end
fprintf('=========================================================================\n');

%% Helper Functions
function out = ParsePose(bag_selection)
% Reads geometry_msgs/PoseStamped messages
msgs = readMessages(bag_selection);
N = length(msgs);

out.time = zeros(N, 1);
out.x = zeros(N, 1);
out.y = zeros(N, 1);
out.z = zeros(N, 1);

for k = 1:N
    msg = msgs{k};
    out.time(k) = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec)*1e-9;
    out.x(k) = msg.pose.position.x;
    out.y(k) = msg.pose.position.y;
    out.z(k) = msg.pose.position.z;
end
end

function out = ParsePoint(bag_selection)
% Reads geometry_msgs/PointStamped messages
msgs = readMessages(bag_selection);
N = length(msgs);

out.time = zeros(N, 1);
out.x = zeros(N, 1);
out.y = zeros(N, 1);
out.z = zeros(N, 1);

for k = 1:N
    msg = msgs{k};
    out.time(k) = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec)*1e-9;
    out.x(k) = msg.point.x;
    out.y(k) = msg.point.y;
    out.z(k) = msg.point.z;
end
end
