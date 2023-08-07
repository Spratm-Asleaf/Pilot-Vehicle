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

%% This file simulates the feedback controller for pilot's autonomous driving
% This file is expected to be invoked AFTER running "Xref_Generator.m" because we first have designed profiles and then we track them.

%% Start
clear all;
close all;
clc;

global P I D;
global m;

LoadData;           % Data Generated by "Xref_Generator.m"

%% PID Parameters
m = 2e3;
P = 0.8*m;
I = 0.1*m;
D = 6*m;

%% Simulate the system (with control)
% @def: y = [pm, x, v, u]';
MAX_TIME = 2100;    % maximum time span --- can just be a guessed value here; 
                    % you can adjust this value to see what will happen (e.g., 500)
% the two statements below are equivalent; choose whichever you want
% [t,y] = ode45(@Syst,[0 MAX_TIME],[0.01,XrefInit,0,0]);      % Adaptive-Step
[t,y] = RungeKutta(@Syst,[0.01,XrefInit,0,0],0.1,0,MAX_TIME); % Fixed-Step

%% Plot the results
len = length(PmPointsXref)/5;
% real time x and expected x_ref
figure;
range = [0 x_max + 100];
plot(PmPointsXref,x_max*ones(1,len*5),'k--','linewidth',1.5);
hold on;
plot(y(:,1),y(:,2),'b',y(:,1),ppval(pp_Xref,y(:,1)),'r--','linewidth',2.5);
hold on;
plot(pms*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',1.5);
hold on;
plot(pmc*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',1.5);
hold on;
plot(pmb*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',1.5);
hold on;
plot(pme*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',1.5);
legend('{\it x_{max}}','{\it x}','{\it x_{ref}}');
set(gca, 'Xtick', [0 pms pmc pmb pme R]);
set(gca,'xticklabel',{'{\it 0}' '{\it p^s_m}' '{\it p^c_m}' '{\it p^b_m}' '{\it p^e_m}' '{\it R}'},'fontsize',16);
set(gca,'FontName','Times New Roman');
set(gca,'Fontsize',16);
axis([0 R range]);


figure;
range = [0 vm_max+Delta_v_max+3];
plot(y(:,1),y(:,3),'b',y(:,1),ppval(pp_Vm,y(:,1)),'r--','linewidth',2.5);
% plot(pm,vm_max*ones(1,len*5),'k--','linewidth',1.5);
% hold on;
% plot(pm,(vm_max+Delta_v_max)*ones(1,len*5),'k--','linewidth',1.5);
hold on;
plot(pms*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',1.5);
hold on;
plot(pmc*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',1.5);
hold on;
plot(pmb*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',1.5);
hold on;
plot(pme*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',1.5);
% ylabel('{\it v} and {\it v_m}');
legend('{\it v}','{\it v_m}');
set(gca, 'Xtick', [0 pms pmc pmb pme R]);
set(gca,'xticklabel',{'{\it 0}' '{\it p^s_m}' '{\it p^c_m}' '{\it p^b_m}' '{\it p^e_m}' '{\it R}'},'fontsize',16);
set(gca,'FontName','Times New Roman');
set(gca,'Fontsize',16);
axis([0 R range]);


figure;
range = [U_underline-0.1 U_overline+0.1];
plot(y(:,1),y(:,4)/m,'b','linewidth',2.5);
hold on;
plot(linspace(0,R,len*5),U_underline*ones(1,len*5),'r--','linewidth',1.5);
hold on;
plot(linspace(0,R,len*5),U_overline*ones(1,len*5),'r--','linewidth',1.5);
hold on;
plot(pms*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',1.5);
hold on;
plot(pmc*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',1.5);
hold on;
plot(pmb*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',1.5);
hold on;
plot(pme*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',1.5);
ylabel('{\it u divided by mass}');
set(gca, 'Xtick', [0 pms pmc pmb pme R]);
set(gca,'xticklabel',{'{\it 0}' '{\it p^s_m}' '{\it p^c_m}' '{\it p^b_m}' '{\it p^e_m}' '{\it R}'},'fontsize',16);
set(gca,'FontName','Times New Roman');
set(gca,'Fontsize',16);
axis([0 R range]);
