%%Motor Parameters
PRated = 4000 ; %W
nRated = 1430 ; %rpm

VRated = 230 ; % l-l Volt

fRated = 50 ; % Hz

s = 1:-0.001:0;
Rs = 1.405 ; % Stator resistance ohm
Ls = 5.839e-3 ; %Stator Inductance Henry
Xs = Ls*2*pi*fRated ; 

Rr = 1.395 ; % Rotor resistance ohm
Lr = 5.839e-3 ; % Rotor inductance Henry
Xr = Lr*2*pi*fRated;

Lm = 0.1722 ; % Magnetization Inductance Henry
Xm = Lm*2*pi*fRated;

Jtotal = 0.04 ;% Total inertia Load + Motor kgm^2
p = 4 ; %Pole number
Tload = 26.7 ;% Rated Torque Nm

Zth = (-Xm*Xs+1i*Rs*Xm)/(Rs+1i*(Xs+Xm));  % Real part of the Zth
Rth = real(Zth);
Xth = imag(Zth);

Vth = 1i*Xm*VRated/(Rs+1i*(Xs+Xm)) ;

ws = 2*pi*fRated/(p/2);%Synchronous Speed rad/sec

ns = 30*ws/pi;

nr = (1-s).*ns;

Te = 3*(real(Vth))^2*Rr./(((Rth+Rr./s).^2+(Xth+Xr)^2).*s.*ws);

%Starting Torque

% Starting Torque when slip is equal to 1
TStart = 3*(real(Vth))^2*Rr./(((Rth+Rr).^2+(Xth+Xr)^2)*ws);

TStartIndex = find(Te==TStart);
nrStartIndex = find(nr==ns*(1-1)); 

strStart = [num2str(nr(nrStartIndex)),' , ' ,num2str(TStart) ];
% As you know ,it has to be zero. 
%However ,to plot it on the graph i have to find the index
%%

%Maximum Torque

%Maximum torque is obtained when airgap power is maximum
%Also, when R2/s power is equal to rest of the circuit power
sMax = Rr/sqrt(Rth^2+(Xth+Xr)^2); % To label on the plot
TMax = 3 *(real(Vth)^2)/(2*ws*(Rth+sqrt(Rth^2+(Xth+Xr)^2)));

temp = abs(Te-TMax);
[~,idx] = min(temp);

TMaxIndex = idx;%find(Te==TMax);
nrMaxIndex = idx;%find(nr==ns*(1-sMax));
strMax = [num2str(nr(nrMaxIndex)),' , ' ,num2str(TMax) ];
%%
temp = abs(Te-Tload);
[~,idx]= min(temp);

TLoadIndex = idx;
nrLoadIndex = idx;

strLoad = [num2str(nr(nrLoadIndex)),' , ' ,num2str(Tload) ];


%%

%% Constant V/f Drive(Constant Flux)
% 40 Hz case
constant = 50/40;
f2Rated = fRated/constant;

ws2 = 2*pi*f2Rated/(p/2);
ns2 = 30*ws2/pi;
nr2 = (1-s).*ns2;

Xs2 = Ls*2*pi*f2Rated ; 
Xr2 = Lr*2*pi*f2Rated;
Xm2 = Lm*2*pi*f2Rated;

Zth2 = (-Xm2*Xs2+1i*Rs*Xm2)/(Rs+1i*(Xs2+Xm2));
Rth2 = real(Zth2);
Xth2 = imag(Zth2);


%To find how much voltage we have to boost,I used our maximum torque value 
%So I derived needed thevenin voltage for current frequency
Vboost2_1 = sqrt(TMax*(2*ws2*(Rth2+sqrt(Rth2^2+(Xth2+Xr2)^2)))/3);
Vboost2_2 = Vboost2_1*(Rs+1i*(Xs2+Xm2))/(1i*Xm2)-VRated/constant;

V2Rated = VRated/constant+Vboost2_2;

Vth2 = 1i*Xm2*V2Rated/(Rs+1i*(Xs2+Xm2)) ;

Te2 = 3*(real(Vth2))^2*Rr./(((Rth2+Rr./s).^2+(Xth2+Xr2)^2).*s.*ws2);


% 37.5 Hz case
constant = 50/37.5;
f3Rated = fRated/constant;

ws3 = 2*pi*f3Rated/(p/2);
ns3 = 30*ws3/pi;
nr3 = (1-s).*ns3;

Xs3 = Ls*2*pi*f3Rated ; 
Xr3 = Lr*2*pi*f3Rated;
Xm3 = Lm*2*pi*f3Rated;

Zth3 = (-Xm3*Xs3+1i*Rs*Xm3)/(Rs+1i*(Xs3+Xm3));
Rth3 = real(Zth3);
Xth3 = imag(Zth3);


%To find how much voltage we have to boost,I used our maximum torque value 
%So I derived needed thevenin voltage for current frequency
Vboost3_1 = sqrt(TMax*(2*ws3*(Rth3+sqrt(Rth3^2+(Xth3+Xr3)^2)))/3);
Vboost3_2 = Vboost3_1*(Rs+1i*(Xs3+Xm3))/(1i*Xm3)-VRated/constant;

V3Rated = VRated/constant+Vboost3_2;

Vth3 = 1i*Xm3*V3Rated/(Rs+1i*(Xs3+Xm3)) ;

Te3 = 3*(real(Vth3))^2*Rr./(((Rth3+Rr./s).^2+(Xth3+Xr3)^2).*s.*ws3);

% 25 Hz case
constant = 50/25;
f4Rated = fRated/constant;

ws4 = 2*pi*f4Rated/(p/2);
ns4 = 30*ws4/pi;
nr4 = (1-s).*ns4;

Xs4 = Ls*2*pi*f4Rated ; 
Xr4 = Lr*2*pi*f4Rated;
Xm4 = Lm*2*pi*f4Rated;

Zth4 = (-Xm4*Xs4+1i*Rs*Xm4)/(Rs+1i*(Xs4+Xm4));
Rth4 = real(Zth4);
Xth4 = imag(Zth4);


%To find how much voltage we have to boost,I used our maximum torque value 
%So I derived needed thevenin voltage for current frequency
Vboost4_1 = sqrt(TMax*(2*ws4*(Rth4+sqrt(Rth4^2+(Xth4+Xr4)^2)))/3);
Vboost4_2 = Vboost4_1*(Rs+1i*(Xs4+Xm4))/(1i*Xm4)-VRated/constant;

V4Rated = VRated/constant+Vboost4_2;

Vth4 = 1i*Xm4*V4Rated/(Rs+1i*(Xs4+Xm4)) ;

Te4 = 3*(real(Vth4))^2*Rr./(((Rth4+Rr./s).^2+(Xth4+Xr4)^2).*s.*ws4);

% 25 Hz case
constant = 50/12.5;
f5Rated = fRated/constant;

ws5 = 2*pi*f5Rated/(p/2);
ns5 = 30*ws5/pi;
nr5 = (1-s).*ns5;

Xs5 = Ls*2*pi*f5Rated ; 
Xr5 = Lr*2*pi*f5Rated;
Xm5 = Lm*2*pi*f5Rated;

Zth5 = (-Xm5*Xs5+1i*Rs*Xm5)/(Rs+1i*(Xs5+Xm5));
Rth5 = real(Zth5);
Xth5 = imag(Zth5);


%To find how much voltage we have to boost,I used our maximum torque value 
%So I derived needed thevenin voltage for current frequency
Vboost5_1 = sqrt(TMax*(2*ws5*(Rth5+sqrt(Rth5^2+(Xth5+Xr5)^2)))/3);
Vboost5_2 = Vboost5_1*(Rs+1i*(Xs5+Xm5))/(1i*Xm5)-VRated/constant;

V5Rated = VRated/constant+Vboost5_2;

Vth5 = 1i*Xm5*V5Rated/(Rs+1i*(Xs5+Xm5)) ;

Te5 = 3*(real(Vth5))^2*Rr./(((Rth5+Rr./s).^2+(Xth5+Xr5)^2).*s.*ws5);

%%

%To plot other waveform please swap the comments where is ! symbol

figure
plot(nr,Te);
hold on;
% plot(nr2,Te2); %!
% hold on;       %!
% plot(nr3,Te3); %!
% hold on;       %!
% plot(nr4,Te4); %!
% hold on;       %!
% plot(nr5,Te5); %!
% hold on;       %!
plot(nr(nrStartIndex),Te(TStartIndex),'r*')
hold on;
plot(nr(nrMaxIndex),Te(TMaxIndex),'g*')
hold on;
plot(nr(nrLoadIndex),Te(TLoadIndex),'c*')
hold on; 

text(nr(nrStartIndex),Te(TStartIndex),strStart,'HorizontalAlignment','left');
text(nr(nrMaxIndex),Te(TMaxIndex),strMax,'HorizontalAlignment','left');
text(nr(nrLoadIndex),Te(TLoadIndex),strLoad,'HorizontalAlignment','left');

legend('TvsS 50Hz','Tstart 50Hz' , 'Tmax 50Hz','Tload 50Hz')
%legend('TvsS 50Hz','TvsS 40Hz','TvsS 37.5Hz','TvsS 25Hz','TvsS 12.5Hz', 'Tstart 50Hz' , 'Tmax 50Hz','Tload 50Hz') %!
title('Torque vs Speed Characteristics of Induction Motors')
%title('Torque vs Speed Characteristics of Induction Motors for Constant Flux Operation') %!
xlabel('Rotor speed (rpm)')
ylabel('Torque (Nm)')
grid on;