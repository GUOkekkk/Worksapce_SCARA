%% For the test of the worksapce
close;
clc;
clear all;

% % Configuration with the collison with link 2
% l1 = 1;
% l2 = 1;
% 
% q1min = -132*pi/180;
% q1max = 132*pi/180;
% q2min = -141*pi/180;
% q2max = 141*pi/180;
% 
% % q1min = -pi;
% % q1max = pi;
% % q2min = -pi;
% % q2max = pi;
% 
% x0 = 1.2;
% y0 = 0.1;
% r0 = 0.3;

% Configuration without collison with link 2
l1 = 1.5;
l2 = 0.5;

q1min = -132*pi/180;
q1max = 132*pi/180;
q2min = -141*pi/180;
q2max = 141*pi/180;

% q1min = -pi;
% q1max = pi;
% q2min = -pi;
% q2max = pi;

x0 = 1.2;
y0 = 0.1;
r0 = 0.2;

%% Test Worksapce of SCARA
workspace_SCARA(l1, l2, q1min, q1max, q2min, q2max, x0, y0, r0);