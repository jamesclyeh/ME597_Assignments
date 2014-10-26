function [ x, y, dec ] = MeasurementModel(actualX, actualY, actualDec )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    [x, y, dec] = [actualX + 0.5*randn(), actualY + 0.5*randn(), actualDec + 0.5*randn()];
end

