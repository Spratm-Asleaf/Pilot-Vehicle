%{
Online supplementary materials of the paper titled 
"A Railway Accident Prevention System Using An Intelligent Pilot Vehicle"
Authored By: Shixiong Wang (1), Xinke Li (1), Zhirui Chen (1), and Yang Liu (1,2)
From (1) the Department of Industrial Systems Engineering and Management, National University of Singapore 
and  (2) the Department of Civil and Environmental Engineering, National University of Singapore

@Function: This is the first part of the codes which design the autonomous driving of the pilot vehicle.
@Author: Shixiong Wang
@Date: First Written on 4 Sep 2020, Updated on 9 Feb 2023
@Email: s.wang@u.nus.edu; wsx.gugo@gmail.com
@Site: https://github.com/Spratm-Asleaf/Pilot-Vehicle

@Externel Dependence & Acknowledgement: https://github.com/PSOPT/psopt
%}

%% Runge--Kutta Integration
% I copied this function from a web page but I forget from which. Thanks and sorry to the anonymous author!

%% Starts
function [t,y] = RungeKutta(ufunc,y0,h,a,b)
    n = floor((b-a)/h);
    t = zeros(1,n+1);
    y = zeros(length(y0),n+1);
    t(1) = a;
    y(:,1)=y0;
    for i=1:n
        t(i+1) = t(i)+h;
        k1 = ufunc(t(i),y(:,i));
        k2 = ufunc(t(i)+h/2,y(:,i)+h*k1/2);
        k3 = ufunc(t(i)+h/2,y(:,i)+h*k2/2);
        k4 = ufunc(t(i)+h,y(:,i)+h*k3);
        y(:,i+1) = y(:,i)+h*(k1+2*k2+2*k3+k4)/6;
    end
    t = t';
    y = y';
end