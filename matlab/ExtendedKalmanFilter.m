
classdef ExtendedKalmanFilter < KalmanFilter
    properties 
        dH;
        P0;
    end
    methods
        function obj = ExtendedKalmanFilter(NumStates, NumMeasurements, NumControlInputs, Dt, P0)
            obj = obj@KalmanFilter(NumStates, NumMeasurements, NumControlInputs, Dt);
            obj.H = 0;
            obj.dH = zeros(NumStates, NumMeasurements)';
            obj.P0 = P0;
        end

        function initializeMatrices(obj, F, G, Q, R, W, P)
            obj.F = F;
            obj.G = G;
            obj.Q = Q;
            obj.R = R;
            obj.W = W;
            obj.P = P;
        end
        function computeH_and_dH(obj)
            %evaluates the measurement matrix H and its derivative dH
            %at the current state estimate x
            %need to implement you own based on your system
            obj.dH(1,1) = obj.P0 * -1 * 1.185427*10^-4 * (1 - obj.X(1)/44330)^4.255;
            obj.dH(1,2) = 0;

            obj.H = obj.P0 * (1 - obj.X(1)/44330)^(1/5.255);
        end
        function update(obj, Z_)
            %update the state and covariance matrix estimates using the
            %given measurement
            %need to implement you own based on your system
            obj.computeH_and_dH();
            obj.K = obj.P * obj.dH' / (obj.dH * obj.P * obj.dH' + obj.R);
            obj.X = obj.X + obj.K * (Z_ - obj.H);
            obj.P = (obj.I - obj.K * obj.dH) * obj.P * (obj.I - obj.K * obj.dH)' + obj.K * obj.R * obj.K';
            obj.Z = Z_;
        end
    end
end