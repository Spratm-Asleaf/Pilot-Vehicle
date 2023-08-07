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

global pp_Vm pp_dVm pp_Xref pp_dXref pp_ddXref

load('Data.mat');

%% Vm, dVm
pp_Vm = spline(PmPointsVm, VmPoints);           % pp_: piecewise polynomial
pp_dVm = spline(PmPointsVm, dVmPoints);

%% Xref, dXref, ddXref
len = length(PmPointsXref)/5;
PmPointsXref([1*len 2*len 3*len 4*len]+1) = [];
XrefPoints([1*len 2*len 3*len 4*len]+1) = [];

% Get derivative of X_ref
dXrefPoints = diff(XrefPoints);
dXrefPoints = dXrefPoints./diff(PmPointsXref);
dXrefPoints = [dXrefPoints(1) dXrefPoints];
% Get second order derivative of X_ref
ddXrefPoints = diff(dXrefPoints);
ddXrefPoints = ddXrefPoints./diff(PmPointsXref);
ddXrefPoints = [ddXrefPoints(1) ddXrefPoints];

pp_Xref = spline(PmPointsXref, XrefPoints);
pp_dXref = spline(PmPointsXref, dXrefPoints);
pp_ddXref = spline(PmPointsXref, ddXrefPoints);

%% Reload original data
load('Data.mat');