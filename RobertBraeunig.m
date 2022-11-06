close all; clear all; clc;

dt = 1;
t0 = 0.3; % time
m0 = 3000000; 
T1 = 3.3e7;
T2 = 4.9e6; 
T3 = 1e6; 
Isp1 = 250;
Isp2 = 420;
Isp3 = 420;
mdot1 = T1/(9.81*Isp1);
mdot2 = T2/(9.81*Isp2);
mdot3 = T3/(9.81*Isp3);
pitch0 = 0;
phi0 = 90;
theta0 = 0;
mu = 3.986005e14; 
h0 = 0;
latitude = 28.5; 
Re = 6373249; %Radius of Earth
R0 = Re+h0; %Radius to vehicle
Pa0 = 101325; %Ambient Pressure [Pa]
Ta0 = 300.8; % Ambient Temperature [K]
Rs = 287.053; %Gas constant
gamma = 1.4;
g0 = mu/(R0)^2;
density0 = Pa0/(Rs*Ta0);
C0 = (gamma*Rs*Ta0)^1/2;
A0 = T1/101325;


%Space Fixed velocity
Vv0 = 0; %Vertical
Vh0 = 0; %Horizontal 
Vst0 = (Vv0^2+Vh0^2)^1/2; %Total
phi_s0 = atan(Vv0/Vh0); %flight path

%Earth fixed
Vr0 = 2*pi*R0/86164*cos(phi0); % Rotation of Earth
Vet0 = (Vv0^2 + (Vh0-Vr0)^2)^1/2;% velocity
phi_e0 = atan(Vv0/(Vh0-Vr0)); %flight path

Ma0 = Vet0/C0; 
Cd0 = Ma0;
D0 = density0*Vet0^2*Cd0*A0/2;

%Horizontal Acceleration
Ah_A0 = -Vh0*Vv0/R0;
Ah_T0 = T1/m0*cos(theta0);
Ah_D0 = D0/m0*cos(pitch0);
Ah_t0 = Ah_A0 + Ah_T0 - Ah_D0;

%Vertical Acceleration
Av_A0 = Vh0^2/R0;
Av_T0 = T1/m0*sin(theta0);
Av_D0 = D0/m0*sin(pitch0);
Av_t0 = -g0 + Av_A0 + Av_T0 - Av_D0;

%intial conditions 
%t = t0;
% = T1;
%mt1 = m0;
%Av_t1 = Av_t0;
%Vet1 = Vet0;
%phi_e1 = phi_e0;
%h1 = h0;
%loop
%LunarData = fopen('LunarV.dat','w');
%fprintf(LunarData,'%f %f %f %f %f\n',t,T,mt1,Av_t1,Vet1,phi_e1,h1);
t=t0+dt;
while t<164
    mt1 = m0-mdot1*dt;
    T = T1-A0*Pa0;
    pitch = pitch0*dt;
    g = mu/R0^2;
    D1 = density0*Vet0^2*Cd0*A0/2;
    Av_A1 = Vh0^2/R0;
    Av_T1 = T/mt1*sin(theta0);
    Av_D1 = D1/mt1*sin(pitch0);
    Av_t1 = -g0 + Av_A1 + Av_T1 - Av_D1;
    Ah_A1 = -Vh0*Vv0/R0;
    Ah_T1 = T/mt1*cos(theta0);
    Ah_D1 = D1/mt1*cos(pitch0);
    Ah_t1 = Ah_A1 + Ah_T1 - Ah_D1;
    Vv1 = Vv0 + (Av_t0+Av_t1)/2*dt;
    Vh1 = Vh0 + (Ah_t0+Ah_t1)/2*dt;
    Vst1 = (Vv1^2+Vh1^2)^1/2; 
    phi_s1 = atan(Vv1/Vh1);
    h1 = h0+(Vv0+Vv1)/2*dt;
    R1 = Re+h1;
    Vr1 = 2*pi*R1/86164*cos(latitude);
    Vet1 = (Vv1^2 + (Vh1-Vr1)^2)^1/2;
    phi_e1 = atan(Vv1/(Vh1-Vr1));
    Pa1 = Pa0*dt;
    Ta1 = Ta0*dt;
    Density = Pa1/(Rs*Ta1);
    C1 = (gamma*Rs*Ta1)^1/2;
    Ma1 = Vet1/C1;
    Cd1 = Cd0*dt;
   % fprintf(LunarData,'%f %f %f %f %f\n',t,T,mt1,Av_t1,Vet1,phi_e1,h1);
    t=t+dt;
end 

%load LunarV.dat
% plots
plot(t,Av_t1);
grid on
title('Acceleration vs time')
ylabel('Acceleration [m/s^2]')
xlabel('time [s]')
hold on
figure
plot(t,h1);
grid on
title('altitude vs time')
ylabel('height [m]')
xlabel('time [s]')
hold on

