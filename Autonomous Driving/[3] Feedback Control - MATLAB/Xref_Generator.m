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

%% This file designs desired position and velocity profiles for pilot's autonomous driving
% This file is expected to be invoked BEFORE running "Xref_Feedback_Controller.m" because we first have designed profiles and then we track them.

clear all;
close all;
clc;

%% Set paras
R = 50e3;                   % (m)eter. For figure clarity and visual effect, we donot use 300e3 here.
                            %          It causes no changes to the experiment results.
                            %          Readers can try 300e3 by themselves to compare the
                            %          results.
vm_max = 120/3.6;           % m/s
Delta_v_max = 10;           % m/s
epsilon = 1.0;              %
m = 2e3;                    % Kg
c0 = 0.01176;               
c1 = 7.7616310e-4;
c2 = 1.6e-5;
pms = 10e3;                 % p^s_m, unit is meter. For figure clarity and visual effect,
                            % we donot use 15e3 here. It causes no changes to the 
                            % experiment results. Readers can try 15e3 by
                            % themselves to compare the results.
pme = R - pms;              % p^e_m, unit is meter
x_0 = 0.1e3;                % m
x_max = 1.5e3;              % m
U_underline = -0.5;         % times of mass, braking force should be negative
U_overline  = 0.6;          % times of mass, traction force is positive, by definition

%% Import data from PSOPT solver
formulation = 'total';
switch formulation
    case 'total'
        % data for Acceleration stage, i.e., p^s_m <= pm <= p^c_m
        Acc_pm = load('PSOPT Solutions\Acc_pm.dat');
        Acc_u = load('PSOPT Solutions\Acc_u.dat');
        Acc_x_v = load('PSOPT Solutions\Acc_x_v.dat');
        pmc = pms + Acc_pm(end);

        % data for Deceleration stage, i.e., p^b_m <= pm <= p^e_m
        Dec_pm = load('PSOPT Solutions\Dec_pm.dat');
        Dec_u = load('PSOPT Solutions\Dec_u.dat');
        Dec_x_v = load('PSOPT Solutions\Dec_x_v.dat');
        pmb = pme - Dec_pm(end);
    case 'min-max'
        % data for Acceleration stage, i.e., p^s_m <= pm <= p^c_m
        Acc_pm = load('PSOPT Solutions\Acc_pm-gamma.dat');
        Acc_u = load('PSOPT Solutions\Acc_u-gamma.dat');
        Acc_x_v = load('PSOPT Solutions\Acc_x_v-gamma.dat');
        pmc = pms + Acc_pm(end);

        % data for Deceleration stage, i.e., p^b_m <= pm <= p^e_m
        Dec_pm = load('PSOPT Solutions\Dec_pm-gamma.dat');
        Dec_u = load('PSOPT Solutions\Dec_u-gamma.dat');
        Dec_x_v = load('PSOPT Solutions\Dec_x_v-gamma.dat');
        pmb = pme - Dec_pm(end);
end

%% Design vm and dvm/dpm versus pm
len = length(Acc_pm);
vm = vm_max*ones(1,len*5);
dvm = zeros(1,len*5);       % derivative of vm w.r.t pm

% vm and dvm/dpm
AttenPara = -0.0025/3;      % Attenuation Parameter; i.e., "\beta" in the paper

% vm in Acceleration stage
pm = linspace(0,pms,len);   % pm in Acceleration stage
vm(1:len) = vm_max * (1-exp(AttenPara*pm));
dvm(1:len) = -AttenPara * vm_max * exp(AttenPara*pm);

% vm in Deceleration stage
pm = linspace(pme,R,len);   % pm in Deceleration stage
vm(end-len+1:end) = vm_max * (1-exp(AttenPara*(R-pm)));
dvm(end-len+1:end) = AttenPara * vm_max * (exp(AttenPara*(R-pm)));

%% Get: x_ref, v, u*
pm = linspace(0,R,len*5);
% 0 <= pm <= p^s_m
    pm_1 = linspace(0,pms,len);
    x_ref_1 = x_0*ones(1,len);
    v_ref_1 = vm(1:len);
    u_star_1 = (vm(1:len).*dvm(1:len)+c0+c1*vm(1:len)+c2*vm(1:len).*vm(1:len))*m/epsilon;
% p^s_m <= pm <= p^c_m
    pm_2 = pms + Acc_pm;
    x_ref_2 = Acc_x_v(1,:);
    v_ref_2 = Acc_x_v(2,:);
    u_star_2 = Acc_u;
% p^c_m <= pm <= p^b_m
    pm_3 = linspace(pmc,pmb,len);
    x_ref_3 = x_max*ones(1,len);
    v_ref_3 = vm_max*ones(1,len);
    u_star_3 = (c0+c1*vm_max+c2*vm_max*vm_max)*m/epsilon * ones(1,len);
% p^b_m <= pm <= p^e_m
    pm_4 = pmb + Dec_pm;
    x_ref_4 = Dec_x_v(1,:);
    v_ref_4 = Dec_x_v(2,:);
    u_star_4 = Dec_u;
% p^e_m <= pm <= R
    pm_5 = linspace(pme,R,len);
    x_ref_5 = x_0*ones(1,len);
    v_ref_5 = vm(end-len+1:end);
    u_star_5 = (vm(end-len+1:end).*dvm(end-len+1:end)+c0+c1*vm(end-len+1:end)+c2*vm(end-len+1:end).*vm(end-len+1:end))*m/epsilon;
    
%% Import Data
% Therefore, the length of pm is "5*len"
pm = [pm_1 pm_2 pm_3 pm_4 pm_5];
x_ref = [x_ref_1 x_ref_2 x_ref_3 x_ref_4 x_ref_5];
v_ref = [v_ref_1 v_ref_2 v_ref_3 v_ref_4 v_ref_5];
u_star = [u_star_1 u_star_2 u_star_3 u_star_4 u_star_5];

VmPoints = vm;
dVmPoints = dvm;
PmPointsVm = linspace(0,R,len*5);       % Pm's key points for Vm; used in fitting when conduting feedback control
                                        % Because vm is generated in terms of "linear" pm
XrefPoints = x_ref;
PmPointsXref = pm;                      % Pm's key points for Xref; used in fitting when conduting feedback control
                                        % After getting pm, we then design x_ref
XrefInit = x_0;

save('Data.mat','VmPoints','dVmPoints','PmPointsVm','XrefPoints','PmPointsXref','XrefInit','pms',...
    'pmc','pmb','pme','R','x_max','vm_max','Delta_v_max','U_underline','U_overline');
    
%% Plot
figure;
% plot v_m
range = [0 35];
plot(pm,vm,'b','linewidth',2.5);
hold on;
plot(pm,vm_max*ones(1,len*5),'r--','linewidth',2);
hold on;
plot(pms*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
hold on;
plot(pmc*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
hold on;
plot(pmb*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
hold on;
plot(pme*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
set(gca, 'Xtick', [0 pms pmc pmb pme R]);
set(gca,'xticklabel',{'{\it 0}' '{\it p^s_m}' '{\it p^c_m}' '{\it p^b_m}' '{\it p^e_m}' '{\it R}'},'fontsize',10);
ylabel('{\it v_m}');
set(gca,'FontName','Times New Roman');
set(gca,'Fontsize',16);
axis([0 R range]);

figure;
% plot x_ref
range = [0 x_max + 100];
plot(pm,x_ref,'b','linewidth',2.5);
hold on;
plot(pm,x_max*ones(1,len*5),'r--','linewidth',2);
hold on;
plot(pms*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
hold on;
plot(pmc*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
hold on;
plot(pmb*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
hold on;
plot(pme*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
ylabel('{\it x_{ref}}');
set(gca, 'Xtick', [0 pms pmc pmb pme R]);
set(gca,'xticklabel',{'{\it 0}' '{\it p^s_m}' '{\it p^c_m}' '{\it p^b_m}' '{\it p^e_m}' '{\it R}'},'fontsize',10);
set(gca,'FontName','Times New Roman');
set(gca,'Fontsize',16);
axis([0 R range]);

figure;
%plot vm and v
range = [0 vm_max+Delta_v_max+3];
plot(pm,v_ref,'b','linewidth',2.5);
hold on;
plot(pm,vm_max*ones(1,len*5),'r--','linewidth',2);
hold on;
plot(pm,(vm_max+Delta_v_max)*ones(1,len*5),'r--','linewidth',2);
hold on;
plot(pms*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
hold on;
plot(pmc*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
hold on;
plot(pmb*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
hold on;
plot(pme*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
ylabel('{\it v}');
set(gca, 'Xtick', [0 pms pmc pmb pme R]);
set(gca,'xticklabel',{'{\it 0}' '{\it p^s_m}' '{\it p^c_m}' '{\it p^b_m}' '{\it p^e_m}' '{\it R}'},'fontsize',10);
set(gca,'FontName','Times New Roman');
set(gca,'Fontsize',16);
axis([0 R range]);
% text(0,0,'{\it v^\Delta_m}','fontname','times new roman');

figure;
% plot relative velocity
range = [-25 Delta_v_max+3];
plot(pm,Delta_v_max*ones(1,len*5),'r--','linewidth',2.5);
hold on;
plot(pm,v_ref-vm,'b','linewidth',2.5);
hold on;
plot(pms*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
hold on;
plot(pmc*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
hold on;
plot(pmb*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
hold on;
plot(pme*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
ylabel('{\it v - v_m}');
set(gca, 'Xtick', [0 pms pmc pmb pme R]);
set(gca,'xticklabel',{'{\it 0}' '{\it p^s_m}' '{\it p^c_m}' '{\it p^b_m}' '{\it p^e_m}' '{\it R}'},'fontsize',10);
set(gca,'FontName','Times New Roman');
set(gca,'Fontsize',16);
axis([0 R range]);

figure;
% plot optimal command u*
range = [U_underline-0.1 U_overline+0.1];
plot(pm,U_underline*ones(1,len*5),'r--','linewidth',2.5);
hold on;
plot(pm,U_overline*ones(1,len*5),'r--','linewidth',2.5);
hold on;
plot(pm,u_star/m,'b','linewidth',2.5);
hold on;
plot(pms*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
hold on;
plot(pmc*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
hold on;
plot(pmb*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
hold on;
plot(pme*ones(1,len*5),linspace(range(1),range(2),len*5),'g--','linewidth',2);
ylabel('{\it u divided by mass}');
set(gca, 'Xtick', [0 pms pmc pmb pme R]);
set(gca,'xticklabel',{'{\it 0}' '{\it p^s_m}' '{\it p^c_m}' '{\it p^b_m}' '{\it p^e_m}' '{\it R}'},'fontsize',10);
set(gca,'FontName','Times New Roman');
set(gca,'Fontsize',16);
axis([0 R range]);


