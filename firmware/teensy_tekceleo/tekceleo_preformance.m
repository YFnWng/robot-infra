clear;
close all

voltage = 7.54; % V
RPM = linspace(0,250,26);
current = [0.058, 1.510, 1.45, 1.415, 1.4, 1.4, 1.39, 1.378, 1.365, 1.355, 1.343, ...
    1.333, 1.320, 1.303, 1.292, 1.272, 1.254, 1.236, 1.220, 1.214, 1.190, 1.180, ...
    1.162, 1.156, 1.170, 1.142]; % A
PSU_power = [0.429, 11.35, 10.92, 10.66, 10.56, 10.56, 10.47, 10.38, 10.28, 10.22, 10.12, ...
    10.06, 9.6, 9.827, 9.736, 9.593, 9.462, 9.307, 9.198, 9.153, 8.980, 8.912, ...
    8.769, 8.716, 8.831, 8.618];
nominal_torque = PSU_power./(RPM/60*2*pi);

figure;
subplot(2,1,1); 
plot(RPM, PSU_power);
title("PSU power")
ylabel("W")
grid on
subplot(2,1,2);
plot(RPM, nominal_torque);
title("torque, assuming 100% efficiency")
xlabel("RPM");
ylabel("Nm")
grid on

%%
linRate = 67.319841; % mm/rev
rotRate = 0.375; % rev/rev
bendRate = 0.375; % rev/rev; both modules use identical rotary bending
jointVRate = [linRate/60.0, rotRate/60.0*360.0, bendRate/60.0*360.0, ...
              linRate/60.0, rotRate/60.0*360.0, bendRate/60.0*360.0 ]; % mm/s or deg/s per RPM

165*jointVRate./[20, 5, 4, 20, 5, 5]
