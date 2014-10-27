function  measurements = MeasurementModel(actualX, actualY, actualDec, t)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    if mod(t, 10) == 1
        measurements = [actualX + normrnd(0,0.01), ...
            actualY + normrnd(0,0.01), actualDec + normrnd(0,10*pi()/180)];
    else
        measurements = [actualX + normrnd(0,0.5), ...
            actualY + normrnd(0,0.5), actualDec + normrnd(0,10*pi()/180)];
    end
end

