%INPUT parameters
%MATERIAL
%d : material density [kg/mm^3]
%sigma: conductivity of the material [1/Ohm mm]
%R: radius of the cylindric particle
%L: length of the cylindric particle
%SYSTEM
% b: constant intensity of the magnetic field [Tesla]
% k: number of couples of poles (N/S)
%Rdrum: radius of the rotating drum [mm]
%o_drum: rotating speed of the drum [rad/s]
%EXP SETTINGS:
%X0,Y0: particle coordinates at time T=0. Origin of the axes is the center
%of rotation of the drum


function eddy_simple_multi

R=0.00225/2; %radius of the particle [m]
L=0.00225; %height of the particle [m]
b = 0.324; %magnetic induction at the shell [T]
shape=4; %shape coefficient [1,5]
material=1; %non-ferrous particles material [1==aluminium, 2=copper]
k=9; %number of north-south poles of the rotor
Rdrum=0.150; %radius of the rotor [m]
o_drum=-209; %rotational speed of the rotor [rad/s]
Y_splitter=-1.5; % position of the splitter
VX0=0; %starting velocity X
VY0=0; %starting velocity Y
X0=0.154;
Y0=0.15;


if material==1 %aluminium
    d=2700; %density[Kg/m^3]
    sigma=32*10^6; %conductivity [1/Ohm m]
end
if material==2 %copper
    d=8900; %density[Kg/m^3]
    sigma=59*10^6; %conductivity [1/Ohm m]
end
if material==3 %magnesium
    d=1820; %density[Kg/m^3]
    sigma=6.67*10^6; %conductivity [1/Ohm m]
end

if shape==1
    V=pi*R^2*L; %volume of the particle [m3]
    m=d*V; %particle mass [Kg]
    J=0.08333*m*(L^2);
end
if shape==2
    V=pi*R^2*L; %volume of the particle [m3]
    m=d*V; %particle mass [Kg]
    J=0.5*m*R^2;
end
if shape==3 
    V=(4/3)*pi*R^3; %volume of the particle [m3]
    m=d*V; %particle mass [Kg]
    J=(2/5)*m*R^2;
end
if shape==4 
    V=pi*R^2*L; %volume of the particle [m3]
    m=d*V; %particle mass [Kg]
    J=0.25*m*(R^2);
end
if shape==5
    V=pi*R^2*L; %volume of the particle [m3]
    m=d*V; %particle mass [Kg]
    J=0.5*m*(R^2);
end


%constants
mu0=4*10^(-7)*pi; %constant of permeability of vacuum.
g=-9.81;  %gravity acceleration [m/s2]
ro=1.225;  %fluid density (air) [Kg/m^3]
%ro=998;  %water denisty



% Use an inconsistent initial condition to test initialization.
y0 = [VX0+10^(-20),VY0+10^(-20),0,X0,Y0];
%0.37
tspan = [0, 1.2];

M=zeros(5,5);
M(1,1)=1;
M(2,2)=1;
M(3,3)=1;
M(4,4)=1;
M(5,5)=1;
M=M';


% Use the LSODI example tolerances.  The 'MassSingular' property is
% left at its default 'maybe' to test the automatic detection of a DAE.
options = odeset('Mass',M,'AbsTol',10^(-12),'events',@event);
%options = odeset('Mass',@MASS,'MStateDependence','strong','MassSingular','yes');

[t,y,TE,YE,IE] = ode45(@f,tspan,y0,options,Rdrum,o_drum,k,b,m,mu0,g,R,L,V,ro,sigma,J,shape,Y_splitter);

%y(:,2) = 1e4*y(:,2);
[XC,YC]=scircle1(0,0,Rdrum);
XCONV = -0.5:0.001:0;
figure;

plot(y(:,4),y(:,5),XC,YC,XCONV,Rdrum)
grid on 
axis equal;
%plot(y(:,3));
YE(4)-X0

ylabel('Y');
title('Trajectory of parts in Eddy current systems');
xlabel('X');

% --------------------------------------------------------------------------
function out = f(t,y,Rdrum,o_drum,k,b,m,mu0,g,R,L,V,ro,sigma,J,shape,Y_splitter)

r=(y(4)^2+y(5)^2)^(0.5);
fi=atan((y(5))*((y(4))^(-1)));
omega_B=-k*(((abs(y(1))*sin(fi)+abs(y(2))*cos(fi))*(r)^(-1))-o_drum)+y(3);
%((abs(y(1))*sin(fi)+abs(y(2))*cos(fi))*(r)^(-1))

if shape==1 %cyclindric parts perpendicular orientation
    rho=abs(y(3))*R*(y(1)^(2)+y(2)^(2))^(-0.5);
    cl=rho*(2.2*rho+0.7)^(-1);
    cd=(0.1+2*cl^(2))^(0.5);
    Fd=cd*ro*(y(1)^(2)+y(2)^(2))*(pi*R^(2))/2;
    Fl=cl*ro*(y(1)^(2)+y(2)^(2))*(pi*R^(2))/2;
    
    ct=0.008;
    gsi=mu0*omega_B*sigma*(2*R)^(2);
    %R_gsi=0;
    %I_gsi=9*24*gsi*((8*(576))^(-1));
    R_gsi=9*gsi^(2)*((8*(576+gsi^(2)))^(-1));
    I_gsi=9*24*gsi*((8*(576+gsi^(2)))^(-1));
end

if shape==2 %cyclindric parts parallel orientation
    rho=abs(y(3))*R*(y(1)^(2)+y(2)^(2))^(-0.5);
    cl=rho*(2.2*rho+0.7)^(-1);
    cd=(0.1+2*cl^(2))^(0.5);
    Fd=cd*ro*(y(1)^(2)+y(2)^(2))*R*L;
    Fl=cl*ro*(y(1)^(2)+y(2)^(2))*R*L;
    ct=0.008;

    gsi=mu0*omega_B*sigma*(2*R)^(2);
    %R_gsi=0;
    %I_gsi=9*24*gsi*((8*(576))^(-1));
    R_gsi=3*gsi^(2)*(2*(576+gsi^(2)))^(-1);
    I_gsi=3*24*gsi*(2*(576+gsi^(2)))^(-1);
end

if shape==3 %sphere 
    rho=abs(y(3))*R*(y(1)^(2)+y(2)^(2))^(-0.5);
    cl=rho*(2.2*rho+0.7)^(-1);
    cd=(0.1+2*cl^(2))^(0.5);
    Fd=cd*ro*(y(1)^(2)+y(2)^(2))*pi*R^(2)/2;
    Fl=cl*ro*(y(1)^(2)+y(2)^(2))*pi*R^(2)/2;
    ct=0.007;
    
    gsi=mu0*omega_B*sigma*(2*R)^(2);
    R_gsi=21*gsi^(2)*(20*(1764+gsi^(2)))^(-1);
    I_gsi=21*42*gsi*(20*(1764+gsi^(2)))^(-1);
end

if shape==4 %disk perpendicular orientation
    rho=abs(y(3))*R*(y(1)^(2)+y(2)^(2))^(-0.5);
    cl=1.4*rho*(rho+1)^(-1);
    cd=(0.16+0.25*cl^(2))^(0.5);
    ct=0.03;
    Fd=cd*ro*(y(1)^(2)+y(2)^(2))*(pi*R^(2))/2;
    Fl=cl*ro*(y(1)^(2)+y(2)^(2))*(pi*R^(2))/2;
    
    gsi=mu0*omega_B*sigma*(2*R)^(2);
    %R_gsi=0;
    %I_gsi=(1/64)*gsi;
    R_gsi=0.6*pi*L*(2*R)^(-1)*gsi^(2)*(4*(256+(0.6*pi*L)^2*(2*R)^(-2)*gsi^(2)))^(-1);
    I_gsi=16*gsi*(4*(256+(0.6*pi*L)^2*(2*R)^(-2)*gsi^(2)))^(-1);
end

if shape==5 %disk parallel orientation
    rho=abs(y(3))*R*(y(1)^(2)+y(2)^(2))^(-0.5);
    cl=1.4*rho*(rho+1)^(-1);
    cd=(0.16+0.25*cl^(2))^(0.5);
    Fd=cd*ro*(y(1)^(2)+y(2)^(2))*R*L;
    Fl=cl*ro*(y(1)^(2)+y(2)^(2))*R*L;
    ct=0.03;
   
    gsi=mu0*omega_B*sigma*(L)^2;
    %R_gsi=0;
    %I_gsi=(1/12)*gsi;
    R_gsi=gsi^(2)*(144+gsi^(2))^(-1);
    I_gsi=12*gsi*(144+gsi^(2))^(-1);
end

B=b*(Rdrum*r^(-1))^(k+1);
Fr=(k+1)*(B^(2))*V*(mu0*r)^(-1)*R_gsi;
Ffi=(k+1)*(B^(2))*V*(mu0*r)^(-1)*I_gsi;
if y(4)>0 %condizione per quadrante 1 del sistema di riferimento
Fx=abs(Fr)*cos(fi)+abs(Ffi)*sin(fi);
Fy=abs(Fr)*sin(fi)-abs(Ffi)*cos(fi);
else
Fx=-abs(Fr)*cos(fi)-abs(Ffi)*sin(fi);
Fy=-abs(Fr)*sin(fi)+abs(Ffi)*cos(fi);
end


%Fdx=abs(Fd)*(-sign(y(1)))*cos(atan(abs(y(2))*(abs(y(1))^(-1))));
%Fdy=abs(Fd)*(-sign(y(2)))*sin(atan(abs(y(2))*(abs(y(1))^(-1))));
%Flx=abs(Fl)*sin(atan(-y(2)*(y(1)^(-1))));
%Fly=abs(Fl)*cos(atan(abs(y(2))*(abs(y(1))^(-1))));

%drag and lift ruotano e non serve modificare. Si attivano solo dopo che
%sono entrato in zona flying.

Fdx=-abs(Fd)*cos(atan(-y(2)*(y(1)^(-1))));
Fdy=abs(Fd)*sin(atan(-y(2)*(y(1)^(-1))));
Flx=abs(Fl)*sin(atan(-y(2)*(y(1)^(-1))));
Fly=abs(Fl)*cos(atan(-y(2)*(y(1)^(-1))));


out = [(Fx+Flx+Fdx)*m^(-1)
       (Fy+Fly+Fdy+m*g)*m^(-1)
       (-B^(2)*V*(mu0^(-1))*I_gsi-ct*(2*R)^(5)*ro*abs(y(3))*y(3))*(J^(-1))
        y(1)
        y(2)];
%
%
%
function [value,isterminal,direction] = event(t,y,Rdrum,o_drum,k,b,m,mu0,g,R,L,V,ro,sigma,J,shape,Y_splitter)
value=y(5)-Y_splitter;
isterminal=1;
direction=-1;

