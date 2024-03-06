% Open the AMC file for reading
% Define file names
fileNames = {'../mocapPlayer-starter/131_04-dance.amc', 
    '../mocapPlayer-starter/131_04-dance-be-N20.amc', 
    '../mocapPlayer-starter/131_04-dance-bq-N20.amc'};

% Initialize cell array to store joint angles
jointAnglesCell = cell(1, numel(fileNames));

% Define frame range
startFrame = 200;
endFrame = 500;

% Loop through each file
for i = 1:numel(fileNames)
    % Open the AMC file for reading
    fileID = fopen(fileNames{i}, 'r');
    % Initialize variables to store joint angles for current file
    jointAngles = {};
    
    % Read the file line by line
    frameCount = 0;
    tline = fgetl(fileID);
    while ischar(tline)
        % Check if the line contains joint angle data for "root"
        if startsWith(tline, 'root')
            frameCount = frameCount + 1;
            if frameCount >= startFrame && frameCount <= endFrame
                % Extract z rotation angle
                angles = sscanf(tline, '%*s %f %f %f %f %f %f');
                % Append the angle to the variable
                jointAngles = [jointAngles; angles(6)];
            end
        end
        % Read the next line
        tline = fgetl(fileID);
    end
    
    % Close the file
    fclose(fileID);
    
    % Store joint angles in cell array
    jointAnglesCell{i} = jointAngles;
end
% Plot joint angles
figure;
hold on;
for i = 1:numel(fileNames)
    % Calculate the number of frames in the selected range
    numFrames = endFrame - startFrame + 1;
    % Create x-axis values representing frames 200 to 500
    xValues = startFrame:endFrame;
    % Plot the joint angles against the correct x-axis values
    plot(xValues, cell2mat(jointAnglesCell{i}(:)), 'LineWidth', 1.5);
end
hold off;

xlabel('Frame');
ylabel('Z Rotation Angle (degrees)');
title('Z Rotation Angle of "root" Joint (Frames 200-500)');
legend('Input', 'Bezier Euler', 'Bezier Quaternion');
