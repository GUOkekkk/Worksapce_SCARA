function [] = workspace_SCARA(l1, l2, q1min, q1max, q2min, q2max, x0, y0, r0)
% WORKSPACE Summary of this function goes here. Created by KeGUO 30/09/2022
%   Detailed explanation goes here
% l1: the length of the first link
% l2: the length of the second link
% q1min, q1max: the first joint limits
% q2min, q2max: the second joint limits
% x0, y0: the position of the obstacle disk
% r0: the radisu of the obstacle disk
% Return: the plot of the workspace of the SCARA or if it exists a
% collision between the link 2 of the robot and the Obsatlce it return a
% message error and no plot

% To judge the robot is under which situation
CASE = 1; % the default situation, there is no joint limits or obstacle

if q1min == -pi && q1max == pi
    if r0 ~= 0
        CASE = 2; % there is no joint limits but obstacle
    end
elseif r0 == 0
    CASE = 3; % there is joint limit but no obstacle
else
    CASE = 4; % there is joint limit and obstacle
end


% Consider the obstacle
oo1 = [x0, y0];
theta = atan2(y0, x0);
alpha = asin(r0/norm(oo1));

alpha1 = theta - alpha;
alpha2 = theta + alpha;


% Consider the relation between the obstacle and joint limits
if CASE == 2 || CASE == 4
    if alpha1 < q1max && alpha2 > q1max
        q1max = alpha1; 
    elseif alpha2 > q1min && alpha1 < q1min
        q1min = alpha2;
    elseif alpha1 > q1min && alpha2 < q2max
        q1min1 = q1min;
        q1min2 = alpha1;
        q1max1 = alpha2;
        q1max2 = q1max;
        if CASE ~= 2 % CASE 2 belongs to CASE 5
            CASE = 5; % the obstacle is all in the joint limits, it is a bit special so need to be a new case
        end
    end
end


% Set the q1 and q2
if CASE ~= 5 && CASE ~= 2
    q1 = q1min : (q1max - q1min)/100: q1max;
else
    q11 = q1min1 : (q1min2 - q1min1)/50: q1min2;
    q12 = q1max1 : (q1max2 - q1max1)/50: q1max2;
end
q2 = q2min : (q2max - q2min)/100: q2max;


% Consider the collision between the link 2 and the obstacle
% If it exists, break the function and print a error message

n = 20; % Sample n points on the link2
sn = 20; % Sample sn point on the link2 joint space
if CASE == 4
    q2_check = q2min : (q2max - q2min)/sn: q2max; % to check if it exists a collision between link2 and obstacle
    C = [x0, y0];
    for liter = 1:length(q2_check)
        A = [l1*cos(q1min), l1*sin(q1min)];
        B = [l1*cos(q1min) + l2*cos(q1min + q2_check(liter)),
            l1*sin(q1min) + l2*sin(q1min + q2_check(liter))];
        for i=0:n % n points on the link2
            M=A+(i/n)*(B-A); % M is a given point between A and B
            dmin((liter-1)*n + i+1)=norm(M-C);
        end      
    end
    Dmin=min(dmin); 

    for liter = 1:length(q2_check)
        A = [l1*cos(q1max), l1*sin(q1max)];
        B = [l1*cos(q1max) + l2*cos(q1max + q2_check(liter)),
            l1*sin(q1max) + l2*sin(q1max + q2_check(liter))];
        for i=0:n % n points on the link2
            M=A+(i/n)*(B-A); % M is a given point between A and B
            dmax((liter-1)*n + i+1)=norm(M-C);
        end      
    end
    Dmax=min(dmax); 

    if Dmin < r0 || Dmax < r0
        error('There is a collison between the link 2 and the obstacle');
    end

elseif CASE == 2 || CASE == 5    

    q2_check = q2min : (q2max - q2min)/sn: q2max; % to check if it exists a collision between link2 and obstacle
    C = [x0, y0];

    for liter = 1:length(q2_check)
        A = [l1*cos(q1min1), l1*sin(q1min1)];
        B = [l1*cos(q1min1) + l2*cos(q1min1 + q2_check(liter)),
            l1*sin(q1min1) + l2*sin(q1min1 + q2_check(liter))];
        for i=0:n % n points on the link2
            M=A+(i/n)*(B-A); % M is a given point between A and B
            dmin1((liter-1)*n + i+1)=norm(M-C);
        end       
    end
    Dmin1=min(dmin1);

    for liter = 1:length(q2_check)
        A = [l1*cos(q1min2), l1*sin(q1min2)];
        B = [l1*cos(q1min2) + l2*cos(q1min2 + q2_check(liter)),
            l1*sin(q1min2) + l2*sin(q1min2 + q2_check(liter))];
        for i=0:n % n points on the link2
            M=A+(i/n)*(B-A); % M is a given point between A and B
            dmin2((liter-1)*n + i+1)=norm(M-C);
        end
    end
    Dmin2=min(dmin2);

    for liter = 1:length(q2_check)
        A = [l1*cos(q1max1), l1*sin(q1max1)];
        B = [l1*cos(q1max1) + l2*cos(q1max1 + q2_check(liter)),
            l1*sin(q1max1) + l2*sin(q1max1 + q2_check(liter))];
        for i=0:n % n points on the link2
            M=A+(i/n)*(B-A); % M is a given point between A and B
            dmax1((liter-1)*n + i+1)=norm(M-C);
        end       
    end
    Dmax1=min(dmax1);

    for liter = 1:length(q2_check)
        A = [l1*cos(q1max2), l1*sin(q1max2)];
        B = [l1*cos(q1max2) + l2*cos(q1max2 + q2_check(liter)),
            l1*sin(q1max2) + l2*sin(q1max2 + q2_check(liter))];
        for i=0:n % n points on the link2
            M=A+(i/n)*(B-A); % M is a given point between A and B
            dmax2((liter-1)*n + i+1)=norm(M-C);
        end
    end
    Dmax2=min(dmax2);

    if Dmin1 < r0 || Dmax1 < r0 || Dmin2 < r0 || Dmax2 < r0
        error('There is a collison between the link 2 and the obstacle');
    end

end



% Create the figure
figure(1);
hold on;
axis equal;
title('The Workspace of SCARA robot');
xlabel("x");
ylabel("y");


% To draw a robot 
if CASE ~= 5 && CASE ~= 2
    q1rand = q1(randi(length(q1)));
else
    q1rand = q11(randi(length(q11)));
end

q2rand = q2min + (q2max - q2min)*rand(1);
scatter(0, 0 ,'r', 'filled');
hold on;

x1rand = l1*cos(q1rand);
y1rand = l1*sin(q1rand);
h1 = scatter(x1rand, y1rand ,'r', 'filled');
line([0, x1rand], [0, y1rand], 'color', 'r');

x2rand = l1*cos(q1rand) + l2*cos(q1rand + q2rand);
y2rand = l1*sin(q1rand)+ l2*sin(q1rand + q2rand);
scatter(x2rand, y2rand ,'r', 'filled')
line([x1rand, x2rand], [y1rand, y2rand], 'color', 'r');


% Solutions
if CASE == 1 % there is no joint limits or obstacle
    % Fix q2
    % Under this Case, the maximun is the singularity
    xmax = l1*cos(q1) + l2*cos(q1+0);
    ymax = l1*sin(q1) + l2*sin(q1+0);

    xmin = l1*cos(q1) + l2*cos(q1+q2max);
    ymin = l1*sin(q1) + l2*sin(q1+q2max);

    % Plot the workspace
    h2 = plot(xmax, ymax, 'b');
    h3 = plot(xmin, ymin, 'b');

    legend([h1,h2], 'Robot', 'Workspace');

elseif CASE == 3 % there is joint limits but no obstacle

    % Fix q2
    xmax = l1*cos(q1) + l2*cos(q1+q2min);
    ymax = l1*sin(q1) + l2*sin(q1+q2min);

    xmin = l1*cos(q1) + l2*cos(q1+q2max);
    ymin = l1*sin(q1) + l2*sin(q1+q2max);

    % Plot the workspace
    h2 = plot(xmax, ymax, 'b');
    h3 = plot(xmin, ymin, 'b');

    % Singularly
    if q2min < 0 && q2max > 0
        xsin = l1*cos(q1) + l2*cos(q1+0);
        ysin = l1*sin(q1) + l2*sin(q1+0);

        % Plot the workspace
        hs = plot(xsin, ysin, 'g');
    end

    % Fix q1
    xmid1 = l1*cos(q1min) + l2*cos(q1min+q2);
    ymid1 = l1*sin(q1min) + l2*sin(q1min+q2);

    xmid2 = l1*cos(q1max) + l2*cos(q1max+q2);
    ymid2 = l1*sin(q1max) + l2*sin(q1max+q2);

    % Plot the workspace
    h4 = plot(xmid1, ymid1, 'b');
    h5 = plot(xmid2, ymid2, 'b');

    legend([h1, h2, hs], 'Robot', 'Workspace', 'Singularity');

elseif CASE == 2 % there is no joint limits but obstacle and definitly is is also under case 5

    % fix q2
    % Under this Case, the maximun is the singularity
    % and it is better to show this singularity in the plot 
    xmax1 = l1*cos(q11) + l2*cos(q11);
    ymax1 = l1*sin(q11) + l2*sin(q11);

    xmin1 = l1*cos(q11) + l2*cos(q11+q2max);
    ymin1 = l1*sin(q11) + l2*sin(q11+q2max);

    xmax2 = l1*cos(q12) + l2*cos(q12);
    ymax2 = l1*sin(q12) + l2*sin(q12);

    xmin2 = l1*cos(q12) + l2*cos(q12+q2max);
    ymin2 = l1*sin(q12) + l2*sin(q12+q2max);

    hs = plot(xmax1, ymax1, 'g');
    h3 = plot(xmin1, ymin1, 'b');
    h4 = plot(xmax2, ymax2, 'g');
    h5 = plot(xmin2, ymin2, 'b');

    % fix q1
    % Actually it is necessary to draw this cause to determine the
    % trajectory problem....but not so sure
    xmid1 = l1*cos(q1min1) + l2*cos(q1min1+q2);
    ymid1 = l1*sin(q1min1) + l2*sin(q1min1+q2);

    xmid2 = l1*cos(q1min2) + l2*cos(q1min2+q2);
    ymid2 = l1*sin(q1min2) + l2*sin(q1min2+q2);

    xmid3 = l1*cos(q1max1) + l2*cos(q1max1+q2);
    ymid3 = l1*sin(q1max1) + l2*sin(q1max1+q2);

    xmid4 = l1*cos(q1max2) + l2*cos(q1max2+q2);
    ymid4 = l1*sin(q1max2) + l2*sin(q1max2+q2);

    h6 = plot(xmid1, ymid1, 'b');
    h7 = plot(xmid2, ymid2, 'b');

    h8 = plot(xmid3, ymid3, 'b');
    h9 = plot(xmid4, ymid4, 'b');

    obs = scatter(x0, y0, 'black', 'filled');

    legend([h1,obs,h8,hs], 'Robot', 'Desk Obstacle','Workspace','Singularity');

elseif CASE == 4 % there is joint limit and obstacle, but obstacle not all in the joint limits
    % deal with as same as the CASE 3
    % Fix q2
    xmax = l1*cos(q1) + l2*cos(q1+q2min);
    ymax = l1*sin(q1) + l2*sin(q1+q2min);

    xmin = l1*cos(q1) + l2*cos(q1+q2max);
    ymin = l1*sin(q1) + l2*sin(q1+q2max);

    % Plot the workspace
    h2 = plot(xmax, ymax, 'b');
    h3 = plot(xmin, ymin, 'b');

    % Singularly
    if q2min < 0 && q2max > 0
        xsin = l1*cos(q1) + l2*cos(q1+0);
        ysin = l1*sin(q1) + l2*sin(q1+0);

        % Plot the workspace
        hs = plot(xsin, ysin, 'g');
    end

    % Fix q1
    xmid1 = l1*cos(q1min) + l2*cos(q1min+q2);
    ymid1 = l1*sin(q1min) + l2*sin(q1min+q2);

    xmid2 = l1*cos(q1max) + l2*cos(q1max+q2);
    ymid2 = l1*sin(q1max) + l2*sin(q1max+q2);

    % Plot the workspace
    h4 = plot(xmid1, ymid1, 'b');
    h5 = plot(xmid2, ymid2, 'b');

    obs = scatter(x0, y0, 'black', 'filled');

    legend([h1,obs, h2, hs], 'Robot','Obstacle', 'Workspace', 'Singularity');

elseif CASE == 5 % there is joint limit and obstacle, similari to CASE 2

    % fix q2
    xmax1 = l1*cos(q11) + l2*cos(q11+q2min);
    ymax1 = l1*sin(q11) + l2*sin(q11+q2min);

    xmin1 = l1*cos(q11) + l2*cos(q11+q2max);
    ymin1 = l1*sin(q11) + l2*sin(q11+q2max);

    xmax2 = l1*cos(q12) + l2*cos(q12+q2min);
    ymax2 = l1*sin(q12) + l2*sin(q12+q2min);

    xmin2 = l1*cos(q12) + l2*cos(q12+q2max);
    ymin2 = l1*sin(q12) + l2*sin(q12+q2max);

    h2 = plot(xmax1, ymax1, 'b');
    h3 = plot(xmin1, ymin1, 'b');
    h4 = plot(xmax2, ymax2, 'b');
    h5 = plot(xmin2, ymin2, 'b');

    % Singularly
    if q2min < 0 && q2max > 0
        xsin1 = l1*cos(q11) + l2*cos(q11+0);
        ysin1 = l1*sin(q11) + l2*sin(q11+0);

        xsin2 = l1*cos(q12) + l2*cos(q12+0);
        ysin2 = l1*sin(q12) + l2*sin(q12+0);

        % Plot the workspace
        hs1 = plot(xsin1, ysin1, 'g');
        hs2 = plot(xsin2, ysin2, 'g');
    end

    % fix q1
    % Actually it is necessary to draw this cause to determine the
    % trajectory problem....but not so sure
    xmid1 = l1*cos(q1min1) + l2*cos(q1min1+q2);
    ymid1 = l1*sin(q1min1) + l2*sin(q1min1+q2);

    xmid2 = l1*cos(q1min2) + l2*cos(q1min2+q2);
    ymid2 = l1*sin(q1min2) + l2*sin(q1min2+q2);

    xmid3 = l1*cos(q1max1) + l2*cos(q1max1+q2);
    ymid3 = l1*sin(q1max1) + l2*sin(q1max1+q2);

    xmid4 = l1*cos(q1max2) + l2*cos(q1max2+q2);
    ymid4 = l1*sin(q1max2) + l2*sin(q1max2+q2);

    h6 = plot(xmid1, ymid1, 'b');
    h7 = plot(xmid2, ymid2, 'b');

    h8 = plot(xmid3, ymid3, 'b');
    h9 = plot(xmid4, ymid4, 'b');

    obs = scatter(x0, y0, 'black', 'filled');

    legend([h1,obs,h8,hs1], 'Robot','Obstacle', 'Workspace','Singularity');

end

% To draw the obstacle disk
circles = [x0, y0, r0];
rectangle('Position',[circles(1,1)-circles(1,3),circles(1,2)-circles(1,3),2*circles(1,3),2*circles(1,3)],'Curvature',[1,1],'linewidth',1,'facecolor','k');axis equal;




end

