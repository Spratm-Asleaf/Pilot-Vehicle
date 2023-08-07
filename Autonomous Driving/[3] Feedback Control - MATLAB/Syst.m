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

function dy = Syst(t, y)
%{
    t is time;

    y is system state vector, which is defined as
    y = [
      Pm
      x
      v
      u
    ];

    the prefix "pp_" means "picewise polynomial"; this is used for polynomial fitting with "spline method"
%}

global pp_Vm pp_dVm pp_Xref pp_dXref pp_ddXref
global P I D
global m

c0 = 0.01176;
c1 = 7.7616310e-4;
c2 = 1.6e-5;

dy = zeros(4,1);

% Pm; dpm = vm
dy(1) = ppval(pp_Vm, y(1));

% x; dx = v - vm
dy(2) = y(3) - ppval(pp_Vm, y(1));

% v; dv = (u/m - c0 - c1*v - c2*v*v)
u = y(4);
dy(3) = (u/m - c0 - c1*y(3) - c2*y(3)*y(3)) + 0.0*randn;

% u; feedback control input; see Appendix for detailed derivation
dy(4) = P*(dy(1)*ppval(pp_dXref, y(1)) - dy(2)) + ...
        I*(ppval(pp_Xref, y(1)) - y(2)) + ...
        D*(dy(1)*ppval(pp_dVm, y(1))*ppval(pp_dXref, y(1)) + dy(1)*dy(1)*ppval(pp_ddXref, y(1)) - dy(3) + dy(1)*ppval(pp_dVm, y(1)));
end

