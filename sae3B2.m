step=0.0001;
finish=100;
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
b_est_init=0.005;

a_est_init=(m1*lc_est+m2_est*l)*g;
h_est_init=(J_est+m2_est*l^2);

%% Real Values

J=0.15;
m2=1.4;
lc=0.3;
b=0.009;

a=(m1*lc+m2*l)*g;
h=(J+m2*l^2);

%% Control Constants

d=9;
gamma=10;
lambda=12;

%% Inputs

theta_d1= @(t) pi/2*cos(pi*t);
dtheta_d1dt = @(t)  -(pi^2)/2*sin(pi*t);
ddtheta_d1dt = @(t)  -(pi^3)/2*cos(pi*t);
theta_d2= @(t) pi/2*cos(2*pi*t);
dtheta_d2dt = @(t) -(pi^2)*sin(2*pi*t);
ddtheta_d2dt = @(t) -2*(pi^3)*cos(2*pi*t);
theta_d={theta_d1,dtheta_d1dt,ddtheta_d1dt; theta_d2,dtheta_d2dt,ddtheta_d2dt};

%% System Response

% initialize some variables
h_est=zeros(1,2);
b_est=zeros(1,2);
a_est=zeros(1,2);

for i=1:1:2
    [t,y] = ode45(@(t,y) odeEstim(t,y,theta_d,i,lambda,a,b,h,d,gamma),t,[pi/2;0;h_est_init;b_est_init;a_est_init],odeset('RelTol',1e-7,'AbsTol',1e-4));
    
    figure('Name','Position');
    plot(t,y(:,1)) %Position
    hold on;
    plot(t,theta_d{i,1}(t));
    title('System Response - Position');
    legend('$\theta$','$\theta_d$','Interpreter','latex');
    xlabel('Time t (sec)');
    
    figure('Name','Velocity');
    plot(t,y(:,2)) %Velocity
    hold on;
    plot(t,theta_d{i,2}(t));
    title('System Response - Velocity');
    legend('$\dot{\theta}$','$\dot{\theta_d}$','Interpreter','latex');
    xlabel('Time t (sec)');
    
    figure('Name','Position Error');
    plot(t,theta_d{i,1}(t)-y(:,1)) %position error
    title('Position Error');
    xlabel('Time t (sec)');
    
    figure('Name','Velocity Error');
    plot(t,theta_d{i,2}(t)-y(:,2)) %velocity error
    title('Velocity Error');
    xlabel('Time t (sec)');
    
    s=y(:,2)+lambda*y(:,1)-theta_d{i,2}(t)-lambda*theta_d{i,1}(t);
    figure('Name','S Error');
    plot(t,s); %s error
    title('S Error');
    xlabel('Time t (sec)');
    
    figure('Name','h Estimation');
    plot(t,y(:,3)); %h parameter estimation
    hold on;
    plot (t,h*ones(length(t),1),'-.')
    title('h Parameter Estimation');
    legend('$\hat{h}$','$h$','Interpreter','latex');
    xlabel('Time t (sec)');
    
    figure('Name','b Estimation');
    plot(t,y(:,4)); %b parameter estimation
    hold on;
    plot (t,b*ones(length(t),1),'-.')
    title('b Parameter Estimation');
    legend('$\hat{b}$','$b$','Interpreter','latex');
    xlabel('Time t (sec)');
    
    figure('Name','a Estimation');
    plot(t,y(:,5)); %a parameter estimation
    hold on;
    plot (t,a*ones(length(t),1),'-.')
    title('a Parameter Estimation');
    legend('$\hat{a}$','$a$','Interpreter','latex');
    xlabel('Time t (sec)');
    
    h_est(i)=y(end,3);
    b_est(i)=y(end,4);
    a_est(i)=y(end,5);
    
end

%% Results - Final Estimations

h_est
b_est
a_est

%% Functions

function res = odeEstim(t,x,theta_d,i,lambda,a,b,h,d,gamma)
s=s_error(t,x',theta_d,i,lambda);
Z=Zeta(t,x',theta_d,i,lambda);
res=[
    x(2);
    -b/h*x(2)-a/h*cos(x(1))+1/h*u_controller(x,Z,s,d);
    -gamma*s*Z
    ];
end

function res = u_controller(x,Z,s,d)

u_c= Z'*[x(3);x(4);x(5)]-d*s;
R=50;

for j=1:length(u_c)
    if (abs(u_c(j))>R)
        u_c(j)=R*sign(u_c(j));
    end
end

res=u_c;
end

function res = Zeta(t,x,theta_d,i,lambda)
res=[
    theta_d{i,3}(t)'+lambda*theta_d{i,2}(t)'-lambda*x(:,2)' ;
    theta_d{i,2}(t)'+lambda*theta_d{i,1}(t)'-lambda*x(:,1)';
    cos(x(:,1)')
    ];
end

function res = s_error(t,x,theta_d,i,lambda)
res=(x(:,2)+lambda*x(:,1)-theta_d{i,2}(t)-lambda*theta_d{i,1}(t));
end