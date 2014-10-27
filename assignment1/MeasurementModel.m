function  measurements = MeasurementModel(actualX, actualY, actualDec )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    measurements = [actualX + normrnd(0,0.5), ...
        actualY + normrnd(0,0.5), actualDec + normrnd(0,10*pi()/180)];
end

