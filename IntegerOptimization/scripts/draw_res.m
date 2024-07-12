clear all
clc
close all

figure(1)
hold on
axis equal
xlabel("x axis (m)");
ylabel("y axis (m)");

res_x = 2000;
res_y = 2000;

num_agent = 7;
line_size = 4;
nodes = [];
for i = 1:num_agent
    agent_path = importfile("../agent"+int2str(i)+"path.csv");
    [m,n] = size(agent_path);
    pos = [];
    for i = 1:m
        pos = [pos; agent_path{i,2}, agent_path{i,3}];
        if i ~= 1
            nodes = [nodes;agent_path{i,2}, agent_path{i,3}];
        end
    end
    plot(pos(:,1), pos(:,2),'LineWidth',line_size)
    hold on
end

[m,n] = size(nodes);
for i = 1:m
    xCenter = nodes(i,1); % Wherever...
    yCenter = nodes(i,2); % Wherever...
    xLeft = xCenter - res_x/2;
    yBottom = yCenter - res_y/2;
    rectangle('Position', [xLeft, yBottom, res_x, res_y], 'EdgeColor', 'k', 'LineWidth', 1);
end

