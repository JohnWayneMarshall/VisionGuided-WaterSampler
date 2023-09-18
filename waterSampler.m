% Clear all variables and close all figures
clear all;
close all;

% Initialize variables
amount = 0;
distConv = 0.01863118; % Conversion factor for distance
cm = 5; % Distance in centimeters

% Establish connection to Arduino
a = arduino();

% Define the number of steps in a revolution for the stepper motor
stepsPerRevolution = 48;  
% Initialize the stepper motor on Arduino using pins D8, D9, D10, D11
thisStepper = StepperRevA(a, stepsPerRevolution, 'D8', 'D9', 'D10', 'D11');

% Initialize the servo motor on pin D3 of Arduino
s = servo(a, 'D3', 'MinPulseDuration', 130*10^-5, 'MaxPulseDuration', 170*10^-5);
pause(3);  
writePosition(s, .1);  % Set the servo position
pause(2);
clear s;  % Clear the servo object

% Setup video input
videoInputNumber=1;
% Initialize video input (might need changes based on camera specs)
vidObj=videoinput('winvideo',videoInputNumber, 'YUY2_640x480');
preview(vidObj);
pause(3);

% Begin water sampling loop until the desired depth is reached
while(cm > 1)

    % Capture and display the current frame
    image=getsnapshot(vidObj);
    image=(ycbcr2rgb(image));
    imshow(image);

    % Extract individual RGB channels
    g_channel=image(:,:,2);
    r_channel=image(:,:,1);
    b_channel=image(:,:,3);

    % Calculate the ratio between the RGB channels
    gr_ratio=double(g_channel)./double(r_channel);
    gb_ratio=double(g_channel)./double(b_channel);
    rb_ratio=double(r_channel)./double(b_channel);

    % Remove NaN values
    gr_ratio(isnan(gr_ratio))=0;
    gb_ratio(isnan(gb_ratio))=0;
    rb_ratio(isnan(rb_ratio))=0;

    % Create a binary mask for the water detection based on RGB ratios
    white_bin=((g_channel<10 & b_channel<10 & r_channel<10));
    blackAndWhite=bwareaopen(white_bin,50);
    waterMask=blackAndWhite >= 100;
    waterMask=imfill(blackAndWhite,'holes');
    green_overlay=imoverlay(image,waterMask, [0,1,0]);

    % Update the binary mask using the new RGB ratios
    white_bin=((gr_ratio>1 & gb_ratio>1 & r_channel<120)|(g_channel<10 & b_channel<10 & r_channel<10));
    blackAndWhite2=bwareaopen(white_bin,50);
    waterMask2=blackAndWhite >= 100;
    waterMask2=imfill(blackAndWhite2,'holes');
    green_overlay2=imoverlay(green_overlay,waterMask2, [0,0,1]);
    imshow(green_overlay2);

    % Calculate the boundaries of the detected water
    boundaries = bwboundaries(waterMask2);
    b1 = boundaries{1};
    b2 = boundaries{2};
    
    % Calculate the shortest distance between the two detected objects
    x1 = b1(:, 2);
    y1 = b1(:, 1);
    x2 = b2(:, 2);
    y2 = b2(:, 1);
    distances = pdist2([x1,y1], [x2,y2]);
    minDistance = min(distances(:));
    [index1, index2] = find(distances == minDistance);

    % Convert the minimum distance using the conversion ratio
    cm = minDistance * distConv;

    % Move the stepper motor based on the detected distance
    if(cm>2)
        MoveCounterClockWise(thisStepper,100,100);
    else
        MoveCounterClockWise(thisStepper,100,10);
    end

end

% Notify the user that the pipette has reached the desired position
disp("The pipette has reached 1 cm above the sample");

pause(1);

% Reinitialize the servo for another operation
s = servo(a, 'D3', 'MinPulseDuration', 130*10^-5, 'MaxPulseDuration', 170*10^-5);
pause(2);

% Begin the loop to move the pipette over the cup
while(cm < 4.5)
    image=getsnapshot(vidObj);
    image=(ycbcr2rgb(image));
    imshow(image);
    g_channel=image(:,:,2);
    r_channel=image(:,:,1);
    b_channel=image(:,:,3);
    gr_ratio=double(g_channel)./double(r_channel);
    gb_ratio=double(g_channel)./double(b_channel);
    rb_ratio=double(r_channel)./double(b_channel);
    gr_ratio(isnan(gr_ratio))=0;
    gb_ratio(isnan(gb_ratio))=0;
    rb_ratio(isnan(rb_ratio))=0;
    white_bin=((g_channel<25 & b_channel<25 & r_channel<25));
    blackAndWhite=bwareaopen(white_bin,50);
    waterMask=blackAndWhite >= 100;
    waterMask=imfill(blackAndWhite,'holes');
    green_overlay=imoverlay(image,waterMask, [0,1,0]);
    white_bin=((gr_ratio>1 & gb_ratio>1 & r_channel<120)|(g_channel<25 & b_channel<25 & r_channel<25));
    blackAndWhite2=bwareaopen(white_bin,50);
    waterMask2=blackAndWhite >= 100;
    waterMask2=imfill(blackAndWhite2,'holes');
    green_overlay2=imoverlay(green_overlay,waterMask2, [0,0,1]);
    imshow(green_overlay2);
    boundaries = bwboundaries(waterMask2);
    b1 = boundaries{1};
    b2 = boundaries{2};
    x1 = b1(:, 2);
    y1 = b1(:, 1);
    x2 = b2(:, 2);
    y2 = b2(:, 1);
    distances = pdist2([x1,y1], [x2,y2]);
    minDistance = min(distances(:));
    cm = minDistance * distConv;
    MoveClockWise(thisStepper,100,100);
end

% Notify the user that the pipette is in position
disp("The pipette is over the top of the cup");

% Clear all variables and reinitialize the Arduino and servo
clear all;
a = arduino();
s = servo(a, 'D3', 'MinPulseDuration', 130*10^-5, 'MaxPulseDuration', 170*10^-5);
