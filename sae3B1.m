step=0.00001;
finish=10;
t=0:step:finish;
format long;

%% Constants-Parameters

m1=2;
g=9.8;
l=0.5;

%% Estimations
J_est=0.12;
m2_est=0.24;
lc_est=0.44;
b_est=0.005;

a_est=(m1*lc_est+m2_est*l)*g;
h_est=(J_est+m2_est*l^2);

%% Real Values

J=0.15;
m2=1.4;
lc=0.3;
b=0.009;

a=(m1*lc+m2*l)*g;
h=(J+m2*l^2);

%% Control Constants

e1 = 8.82; %|a-hat{a}|<e1
e2 = 0.82; %|beta/hat{beta}-1|<e2
e3 = 0.005; %|b-hat{b}|<e3

zeta=1; % zeta must be greater or equal than one - two real poles
c=20*(zeta^2-1+zeta*sqrt(zeta^2-1));

% Move Eigenvalues
k1=100+10*c;
k2=20+c;

%% Desired Inputs (for Tracking)

theta_d1= @(t) zeros(length(t),1);
theta_d2= @(t)pi/2*cos(pi*t);
dtheta_d2dt = @(t)  -(pi^2)/2*sin(pi*t);
ddtheta_d2dt = @(t)  -(pi^3)/2*cos(pi*t);
theta_d3= @(t) pi/2*cos(2*pi*t);
dtheta_d3dt = @(t) -(pi^2)*sin(2*pi*t);
ddtheta_d3dt = @(t) -2*(pi^3)*cos(2*pi*t);
theta_d={theta_d1,theta_d1,theta_d1; theta_d2,dtheta_d2dt,ddtheta_d2dt; theta_d3,dtheta_d3dt,ddtheta_d3dt};

%% Lyapunov's Equation Solution

Q=[20 0; 0 0.06];
A_tilde=[0,1;-k1,-k2];
P=lyap(A_tilde',Q);

%% Controllers

delta_v = @(z1,z2,t,i) ((1/h_est*(e2+1)*(e1+e3*abs(z2))+e2*abs(k1*(theta_d{i,1}(t)-z1)+k2*(theta_d{i,2}(t)-z2)+theta_d{i,3}(t)) )/(1-e2) ).*sgn([theta_d{i,1}(t)-z1,theta_d{i,2}(t)-z2]*P*[0;-1]) ; % magnitude of uncertainties handler
u_linear= @(z1,z2,t) a_est*cos(z1)+b_est*z2; % remove non-linear, uknown part
u_eigenval=@(z1,z2,t) (k1*z1+k2*z2); % move eigenvalues

%% System Response - Phase Portrait

for i=1:1:3
    [t,x] = ode23(@(t,x) odefun(t,x,u_linear,u_eigenval,delta_v,a,b,h,h_est,theta_d,i),t,[pi/4;0],odeset('RelTol',1e-6,'AbsTol',1e-3));
   
    u_con=u_controller(t,x,u_linear,u_eigenval,delta_v,h_est,theta_d,i);
    figure('Name','Controller');
    plot(t,u_con);
    title('Controller');
    xlabel('Time t (sec)');
    
    figure('Name','Position');
    plot(t,x(:,1)) %Position
    hold on;
    plot(t,theta_d{i,1}(t));
    title('System Response - Position');
    legend('$\theta$','$\theta_d$','Interpreter','latex');
    xlabel('Time t (sec)');
    
    figure('Name','Velocity');
    plot(t,x(:,2)) %Velocity
    hold on;
    plot(t,theta_d{i,2}(t));
    title('System Response - Velocity');
    legend('$\dot{\theta}$','$\dot{\theta_d}$','Interpreter','latex');
    xlabel('Time t (sec)');
    
    figure('Name','Position Error');
    plot(t,(theta_d{i,1}(t)-x(:,1))) %Position error
    title('System Response - Position Error');
    xlabel('Time t (sec)');
    
    figure('Name','Velocity Error');
    plot(t,(theta_d{i,2}(t)-x(:,2))) %Velocity error
    title('System Response - Velocity Error');
    xlabel('Time t (sec)');
    
    f = @(t,x) [ % Phase Portrait function - In this case x is error
        x(2);
        theta_d{i,3}(t)+b/h*(x(2))+a/h*cos(x(1))-1/h*(u_linear(theta_d{i,1}(t)-x(1),theta_d{i,2}(t)-x(2),t)+h_est*u_eigenval(x(1),x(2),t)-h_est*delta_v(theta_d{i,1}(t)-x(1),theta_d{i,2}(t)-x(2),t,i)+h_est*theta_d{i,3}(t))
        ];
    
    % Uncomment this block to remove uncertainties handler (delta_v)
    
    %f = @(t,x) [ %Phase Portrait function - without Uncertainties Handler
    %  x(2);
    %  theta_d{i,3}(t)+b/h*(x(2))+a/h*cos(x(1))-1/h*(u_linear(theta_d{i,1}(t)-x(1),theta_d{i,2}(t)-x(2),t)+h_est*u_eigenval(x(1),x(2),t)+h_est*theta_d{i,3}(t))
    %];
 
     slope_field(f,[pi/4;0]); % Plot Phase Portrait
end

%% ODE - Phase Portrait Functions

function res = odefun(t,x,u_linear,u_eigenval,delta_v,a,b,h,h_est,theta_d,i)
u_con=u_controller(t,x',u_linear,u_eigenval,delta_v,h_est,theta_d,i);
res=[
    x(2);
    -b/h*x(2)-a/h*cos(x(1))+1/h*u_con;
    ];
end

function res = u_controller(t,x,u_linear,u_eigenval,delta_v,h_est,theta_d,i)

u_c=(u_linear(x(:,1),x(:,2),t)+h_est*u_eigenval(theta_d{i,1}(t)-x(:,1),theta_d{i,2}(t)-x(:,2),t)-h_est*delta_v(x(:,1),x(:,2),t,i)+h_est*theta_d{i,3}(t));

% Uncomment next line to remove uncertainties handler (delta_v)
%u_c=(u_linear(x(1),x(2),t)+h_est*u_eigenval(theta_d{i,1}(t)-x(1),theta_d{i,2}(t)-x(2),t)+h_est*theta_d{i,3}(t));

R=50;

for j=1:length(u_c)
    if (abs(u_c(j))>R)
    u_c(j)=R*sign(u_c(j)); 
    end
end

res=u_c;
end

function res = sgn(x)
epsilon=1e-3;
if(x==0)
    res=0;
elseif(abs(x)>=epsilon)
    res=x./abs(x);
else
    res=x/epsilon;
end
% Uncomment next line to use normal sgn
% res=x./abs(x);  
end

function slope_field(f,init_values)
figure('Name','Error Phase Portrait - init_val: [pi/4,0]');
x1 = linspace(-6.5,4.5,100);
x2 = linspace(-10.5,0.5,100);
[x,y] = meshgrid(x1,x2);
u = zeros(size(x));
v = zeros(size(x));
t=0;
for i = 1:numel(x)
    Y = f(t,[x(i); y(i)]);
    u(i) = Y(1);
    v(i) = Y(2);
end
quiver(x,y,u,v,'b'); figure(gcf)
xlabel('e_1')
ylabel('e_2')
title('Phase Portrait of Error')
axis tight equal;
hold on
for i=1:1:size(init_values,2)
    [ts,ys] = ode45(f,[0,50],init_values(:,i));
    plot(ys(:,1),ys(:,2))
    plot(ys(1,1),ys(1,2),'bo')
    plot(ys(end,1),ys(end,2),'ks')
end
hold off
end