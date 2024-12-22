unzip("bag_pose.zip");
folderPath = fullfile(pwd,"bag_pose");
bagReader = ros2bagreader(folderPath);
baginfo = ros2("bag","info",folderPath)
msgs = readMessages(bagReader);
bagReader.AvailableTopics
disp(msgs{1}.pose.pose.position)
numMessages = length(msgs);
x_positions = zeros(numMessages, 1);

y_positions = zeros(numMessages, 1);
for i = 1:numMessages
    x_positions(i) = msgs{i}.pose.pose.position.x;
    y_positions(i) = msgs{i}.pose.pose.position.y;
end

plot(x_positions, y_positions, '-o', 'LineWidth', 1.5, 'MarkerSize', 5);
xlabel('x [m]');
ylabel('y [m]');
title('fra2mo pose');
grid on;

highlight_x = [6.5, -1.6, 6, 0, 0];
highlight_y = [-1.4, -2.5, 4, 3, 0];

hold on
scatter(highlight_x, highlight_y, 100, 'r', 'filled');