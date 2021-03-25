% Path following for the kinematic model of an Ackermann type car-like
% robot using transverse feedback linearization in continous time and
% under sampling.  Mohamed Elobaid 2021  mohamed.elobaid@uniroma1.it


clc
clear all

simTime = 20; 

T = 0.3; 
emulation = T;

l = 1; 
r = 2;


l1 = 1; 
l2 = 1; 
l3 = 1; 
a0 = l1*l2*l3; 
a1 = (l1+l2)*l3+l1*l1; 
a2 = l1*l2*l3;
a0 = 1; 
a1 = 4;
a2 = 3; 

k0 = 5; 


Change = 0;

a       = 3;
b       = 1.05*a;

pathType = 1;  % pathType 1 for circle, 2 for Casini oval;
refType = 1;   % specify type of velocity ref 1 for step changes, 2 for ramp

v0 = 2; 
v_ref = 2; 
ref0 = [0 0]';
m = 0.1;

A = [0 1 0;0 0 1;a0 a1 a2];
lambdatr = eig(A);
Ad  = [1 T 0;0 1 T;0 0 1];
bd = [0;0; T];
lamdad1 = exp(lambdatr(1)*T); lamdad2 = exp(lambdatr(2)*T); lamdad3 = exp(lambdatr(3)*T);
pd = [lamdad1 lamdad2 lamdad3];
k1 = place(Ad,bd,pd);
Ac = [0 1 0;0 0 1;0 0 0]; bc = [0;0;1];
k2 = place(Ac,bc,lambdatr');

% start close to path in a "feasible pose"
if (pathType == 1)
    x0 = 3; 
    y0 = 2; 
    theta0 = pi;
    phi0 = 0;
else
    x0 = 1; 
    y0 = 2; 
    theta0 = pi;
    phi0 = 0;
end


%---------------------------------------------------------------------%
%--------------------------- SIMULATION ------------------------------%
%---------------------------------------------------------------------%

out = sim('Ackeraman.slx');


t = 0:10^-3:simTime;

Draw_traj(out.x,out.y,out.xe,out.ye, out.xmr, out.ymr, out.xsd, out.ysd);
   
figure 
subplot(2,1,1);

plot(t, out.v, 'k','LineWidth', 2);
hold on; grid on;
plot(t, out.ve, 'r--','LineWidth', 2);
plot(t, out.vmr, 'b','LineWidth', 2);
plot(t, out.vsd,'LineWidth', 2);
l = legend('$v$ Continuous-time', '$v$ Emulation', '$v$ MR Sampling', '$v$ SR Sampling');
set(l,'Interpreter','Latex'); l.FontSize = 20;
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex'); l.FontSize = 20;

subplot(2,1,2);

plot(t, out.w, 'k','LineWidth', 2);
hold on; grid on;
plot(t, out.we, 'r--','LineWidth', 2);
plot(t, out.wmr, 'b','LineWidth', 2);
plot(t, out.wsd,'LineWidth', 2);
l = legend( '\omega Continuous time', '\omega Emulation', '\omega MR Sampling', '\omega SR Sampling');
set(l,'Interpreter','Latex'); l.FontSize = 20;
l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex'); l.FontSize = 20;



function Draw_traj(x,y,xe,ye, xmr, ymr, xsd, ysd)


set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)
line_width = 1.5;
fontsize_labels = 14;

x_c_1 = [];  x_e_1 = [];  x_mr_1 = []; x_sd_1 = [];
y_c_1 = [];  y_e_1 = [];  y_mr_1 = []; y_sd_1 = [];

figure('Name','Path following trajectories')
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);
for k = 1:50:size(x,1)
    x1 = x(k); y1 = y(k); xe1 = xe(k);  ye1 = ye(k); xmr1 = xmr(k);  ymr1 = ymr(k); xsd1 = xsd(k);  ysd1 = ysd(k);
    x_c_1 = [x_c_1 x1]; x_e_1 = [x_e_1 xe1]; x_mr_1 = [x_mr_1 xmr1]; x_sd_1 = [x_sd_1 xsd1];
    y_c_1 = [y_c_1 y1]; y_e_1 = [y_e_1 ye1]; y_mr_1 = [y_mr_1 ymr1]; y_sd_1 = [y_sd_1 ysd1];
    scatter(x(1),y(1),'k','diamond', 'LineWidth', 5);
    hold on;
    c = plot(x_c_1,y_c_1,'-k','linewidth',line_width);hold on % 
    plot(x1,y1,'-sk','MarkerSize',20, ...
        'MarkerEdgeColor','black',...
        'MarkerFaceColor',[0 0 0]);
    e = plot(x_e_1,y_e_1,'-r','linewidth',line_width);    
    plot(xe1,ye1,'-sk','MarkerSize',20, ...
        'MarkerEdgeColor','red',...
        'MarkerFaceColor',[1 .6 .6]);
    mr = plot(x_mr_1,y_mr_1,'-b','linewidth',line_width);    
    plot(xmr1,ymr1,'-sk','MarkerSize',20, ...
        'MarkerEdgeColor','red',...
        'MarkerFaceColor',[0 .1 .9]);
    sd = plot(x_sd_1,y_sd_1,'color','#D95319','linewidth',line_width);    
    plot(xsd1,ysd1,'-sk','MarkerSize',20, ...
        'MarkerEdgeColor','red',...
        'MarkerFaceColor',[0.7 .4 .1]);
    hold off
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels);
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels);
    legend([c e mr sd],'Continuous time','Emulation', 'MR Sampling','SR Sampling' ,'latex','FontSize',fontsize_labels);
    axis([-3 3 -3 3])
    pause(0.05)
    box on;
    grid on
    drawnow
end
end




