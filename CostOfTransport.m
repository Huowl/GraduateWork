function CoT = CostOfTransport(simout,actuator)

measBody = simout.measBody;
yData = timeseries2timetable(measBody.y).y;
xData = timeseries2timetable(measBody.x).x;
zData = timeseries2timetable(measBody.z).z;

    if actuator == 2 || actuator == 4 % If PD ctrl
        PowLegs = simout.measPow;
        Energy = PowLegs.PwRfront.EnergyFemur.Data(end) + PowLegs.PwRfront.EnergyTibia.Data(end)...
            + PowLegs.PwRrear.EnergyFemur.Data(end) + PowLegs.PwRrear.EnergyTibia.Data(end) ...
            + PowLegs.PwLfront.EnergyFemur.Data(end) + PowLegs.PwLfront.EnergyTibia.Data(end) ...
            + PowLegs.PwLrear.EnergyFemur.Data(end) + PowLegs.PwLrear.EnergyTibia.Data(end);
    elseif actuator == 1 % If Motor
        measPow = simout.PowLegs;
        P_front_right = abs(measPow.PwRfront.VoltageFemur.Data.*measPow.PwRfront.CurrentFemur.Data) ...
            + abs(measPow.PwRfront.VoltageTibia.Data.*measPow.PwRfront.CurrentTibia.Data);
        P_front_left = abs(measPow.PwLfront.VoltageFemur.Data.*measPow.PwLfront.CurrentFemur.Data) ...
            + abs(measPow.PwLfront.VoltageTibia.Data.*measPow.PwLfront.CurrentTibia.Data);
        P_rear_right = abs(measPow.PwRrear.VoltageFemur.Data.*measPow.PwRrear.CurrentFemur.Data) ... 
            + abs(measPow.PwRrear.VoltageTibia.Data.* measPow.PwRrear.CurrentTibia.Data);
        P_rear_left = abs(measPow.PwLrear.VoltageFemur.Data.*measPow.PwLrear.CurrentFemur.Data) ... 
            + abs(measPow.PwLrear.VoltageTibia.Data.*measPow.PwLrear.CurrentTibia.Data);
        Energy = trapz(timeData,P_front_left) + trapz(timeData,P_front_right)...
            + trapz(timeData,P_rear_left) + trapz(timeData,P_rear_right); 
    end

%% Weight 

body_mass = 6.0;
foot_mass = 0.06;
hip_mass = 0.595;
tibia_mass = 0.151;
femur_mass = 0.888;

gravity = 9.8;

W = (body_mass + 4*(foot_mass+hip_mass+tibia_mass+femur_mass))*gravity;

%% Distance

delta_x = xData(2:end) - xData(1:end-1);
delta_y = yData(2:end) - yData(1:end-1);
delta_z = zData(2:end) - zData(1:end-1);
del_radius_vector = sqrt(delta_x.^2 + delta_y.^2 + delta_z.^2);
distance = sum(del_radius_vector);

    %% Penalty function
CoT = Energy / W / distance;
end