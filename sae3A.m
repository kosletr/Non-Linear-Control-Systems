step=0.001;
finish=20;
t=0:step:finish;

%% Math functions
u = @(x) +(x>=0);
%ramp = @(x) x.*u(x);
m = @(x1,x2)  0.2*(u(x1-0.2)*u(x2)+u(x1-0.1)*u(-x2))-0.2*(u(-x1-0.1)*u(x2)+u(-x1-0.2)*u(-x2)) ;

%% Inputs
r1=zeros(length(t),1);
r2=(1.2).*u(t).';
r3=(0.4).*u(t).';
r4=(0.8).*u(t).';

%% Inital Values
init_values=[ [-1,0.5]; [0.05,1.3];[0.4,0.6];[1,-0.5];[0.5,-1];[0.02,-0.2];[-0.5,-0.5];[0.1,0.01]].';

%% Linear System Response
f1=figure( 'Name','Fig1');
f2=figure( 'Name','Fig2');


for i=1:8
    [t,x] = ode45(@(t,x) dedt(t,step,x,r1),t,init_values(:,i));
    if(i==1)
        figure(f1);
    elseif(i==5)
        figure(2);
    end
    subplot(2,2,mod(i,4)+1);
    plot(t,x);
    title('System Response on r(t)=u(t) - Linear System');
    legend('x_1','x_2');
    xlabel('Time t (sec)');
    txt1 =['x_1(0)=',num2str(init_values(1,i))];
    txt2=[' x_2(0)=',num2str(init_values(2,i))];
    txt={txt1;txt2};
    ylabel(txt);
end

f3=figure( 'Name','Fig3');

figure(f3);
f = @(t,x) [x(2); -4*x(1)-x(2)];
slope_field(f,init_values)
title('Slope Field - Linear System - r(t)=u(t)');


f4=figure( 'Name','Fig4');
f5=figure( 'Name','Fig5');

for i=1:8
    [t,x] = ode45(@(t,x) dedt(t,step,x,r2),t,init_values(:,i));
    if(i==1)
        figure(f4);
    elseif(i==5)
        figure(f5);
    end
    subplot(2,2,mod(i,4)+1);
    plot(t,x);
    title('System Response on r(t)=1.2tu(t) - Linear System');
    legend('x_1','x_2');
    xlabel('Time t (sec)');
    txt1 =['x_1(0)=',num2str(init_values(1,i))];
    txt2=[' x_2(0)=',num2str(init_values(2,i))];
    txt={txt1;txt2};
    ylabel(txt);
end

f6=figure( 'Name','Fig6');

figure(f6);
f = @(t,x) [x(2); 1.2-4*x(1)-x(2)];
slope_field(f,init_values)
title('Slope Field - Linear System - r(t)=1.2tu(t)');


f7=figure( 'Name','Fig7');
f8=figure( 'Name','Fig8');

for i=1:8
    [t,x] = ode45(@(t,x) demdt(t,step,x,m,r1),t,init_values(:,i));
    if(i==1)
        figure(f7);
    elseif(i==5)
        figure(f8);
    end
    subplot(2,2,mod(i,4)+1);
    plot(t,x);
    title('System Response on r(t)=u(t) - Non Linear System');
    legend('x_1','x_2');
    xlabel('Time t (sec)');
    txt1 =['x_1(0)=',num2str(init_values(1,i))];
    txt2=[' x_2(0)=',num2str(init_values(2,i))];
    txt={txt1;txt2};
    ylabel(txt);
    yticks(-2:0.2:2)
end


f9=figure( 'Name','Fig9');

figure(f9);
f = @(t,x) [x(2); -4*m(x(1),x(2))-x(2)];
slope_field(f,init_values)
title('Slope Field - Non Linear System - r(t)=u(t)');


%% Non Linear System Response

f10=figure( 'Name','Fig10');
f11=figure( 'Name','Fig11');

for i=1:8
    [t,x] = ode45(@(t,x) demdt(t,step,x,m,r2),t,init_values(:,i));
    if(i==1)
        figure(f10);
    elseif(i==5)
        figure(f11);
    end
    subplot(2,2,mod(i,4)+1);
    plot(t,x);
    title('System Response on r(t)=1.2*t*u(t) - Non Linear System');
    legend('x_1','x_2');
    xlabel('Time t (sec)');
    txt1 =['x_1(0)=',num2str(init_values(1,i))];
    txt2=[' x_2(0)=',num2str(init_values(2,i))];
    txt={txt1;txt2};
    ylabel(txt);
end

f12=figure( 'Name','Fig12');

figure(f12);
f = @(t,x) [x(2); 1.2-4*m(x(1),x(2))-x(2)];
slope_field(f,init_values)
title('Slope Field - Non Linear System - r(t)=1.2tu(t)');


f13=figure( 'Name','Fig13');
f14=figure( 'Name','Fig14');

for i=1:8
    [t,x] = ode45(@(t,x) demdt(t,step,x,m,r3),t,init_values(:,i));
    if(i==1)
        figure(f13);
    elseif(i==5)
        figure(f14);
    end
    subplot(2,2,mod(i,4)+1);
    plot(t,x);
    title('System Response on r(t)=0.4*t*u(t) - Non Linear System');
    legend('x_1','x_2');
    xlabel('Time t (sec)');
    txt1 =['x_1(0)=',num2str(init_values(1,i))];
    txt2=[' x_2(0)=',num2str(init_values(2,i))];
    txt={txt1;txt2};
    ylabel(txt);
    yticks(-2:0.2:2)
end

f15=figure( 'Name','Fig15');

figure(f15);
f = @(t,x) [x(2); 0.4-4*m(x(1),x(2))-x(2)];
slope_field(f,init_values)
title('Slope Field - Non Linear System - r(t)=0.4tu(t)');


f16=figure( 'Name','Fig16');
f17=figure( 'Name','Fig17');

for i=1:8
    [t,x] = ode45(@(t,x) demdt(t,step,x,m,r4),t,init_values(:,i));
    if(i==1)
        figure(f16);
    elseif(i==5)
        figure(f17);
    end
    subplot(2,2,mod(i,4)+1);
    plot(t,x);
    title('System Response on r(t)=0.8*t*u(t) - Non Linear System');
    legend('x_1','x_2');
    xlabel('Time t (sec)');
    txt1 =['x_1(0)=',num2str(init_values(1,i))];
    txt2=[' x_2(0)=',num2str(init_values(2,i))];
    txt={txt1;txt2};
    ylabel(txt);
end

f18=figure( 'Name','Fig18');

f = @(t,x) [x(2); 0.8-4*m(x(1),x(2))-x(2)];
slope_field(f,init_values)
title('System Response on r(t)=0.8*t*u(t) - Non Linear System');

%% ODE - Phase Portrait Functions
function res = demdt(t,step,x,m,r)
res=[x(2) ; -4*m(x(1),x(2))-x(2)+r(floor(t/step)+1)];
end

function res = dedt(t,step,x,r)
res=[x(2) ; -4*x(1)-x(2)+r(floor(t/step)+1)];
end

function slope_field(f,init_values)

x1 = linspace(-2,2,80);
x2 = linspace(-1.5,2,80);
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
xlabel('x_1')
ylabel('x_2')
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