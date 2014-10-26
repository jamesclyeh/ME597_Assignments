function  measurements = MeasurementModel(actualX, actualY, actualDec )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    measurements = [actualX + 0.5*randn(), ...
        actualY + 0.5*randn(), actualDec + 10*randn()];
end

