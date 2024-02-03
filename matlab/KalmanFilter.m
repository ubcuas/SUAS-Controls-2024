%{
    Algortihm:

    1. initialize the vectors and matrices
    2. predict: X = F*X + G*U + W  <- Get values of U 
                P = F*P*F.transpose() + Q
    3. update:  K = P*H.transpose()*(H*P*H.transpose() + R).inverse()  (Kalman gain)
                X = X + K*(Z - H*X)  <- Get values of Z
                P = (I - K*H)*P(I - K*H).transpose() + K*R*K.transpose()
%}
classdef KalmanFilter < handle
    properties
        NumStates;
        NumMeasurements;
        NumControlInputs;
        Dt;
        X;  % State vector
        U;  % Control vector
        Z;  % Measurement vector
        F;  % State transition matrix
        G;  % Control matrix
        Q;  % Process noise covariance
        H;  % Measurement matrix
        R;  % Measurement noise covariance
        P;  % Estimate error covariance
        K;  % Kalman Gain
        I;  % Identity matrix
        W;  % Process noise
    end
    methods
        %constructor
        function obj = KalmanFilter(NumStates, NumMeasurements, NumControlInputs, Dt)
            obj.NumStates = NumStates;
            obj.NumMeasurements = NumMeasurements;
            obj.NumControlInputs = NumControlInputs;
            obj.Dt = Dt;
            obj.X = zeros(NumStates, 1);
            obj.U = zeros(NumControlInputs, 1);
            obj.Z = zeros(NumMeasurements, 1);
            obj.F = eye(NumStates);
            obj.G = zeros(NumStates, NumControlInputs);
            obj.Q = zeros(NumStates, NumStates);
            obj.H = zeros(NumMeasurements, NumStates);
            obj.R = zeros(NumMeasurements, NumMeasurements);
            obj.P = zeros(NumStates, NumStates);
            obj.K = zeros(NumStates, NumMeasurements);
            obj.I = eye(NumStates);
            obj.W = zeros(NumStates, 1);
        end

        % Provide all the matrices to this function
        function initializeMatrices(obj, F, G, Q, H, R, W, P)
            obj.F = F;
            obj.G = G;
            obj.Q = Q;
            obj.H = H;
            obj.R = R;
            obj.W = W;
            obj.P = P;
        end

        %function to predict the next state
        function predict(obj, U_)
            % Predicted state estimate
            obj.X = obj.F * obj.X + obj.G * U_ + obj.W;
            % Predicted estimate covariance
            obj.P = obj.F * obj.P * obj.F' + obj.Q;
            %save the control input for the next iteration
            obj.U = U_;
        end

        %function to update the state
        function update(obj, Z_)
            % Measurement residual
            obj.Z = Z_;

            %Calculate Kalman Gain
            obj.K = (obj.P * obj.H') / (obj.H * obj.P * obj.H' + obj.R);

            % Update state estimate
            obj.X = obj.X + obj.K * (obj.Z - obj.H * obj.X);

            % Update estimate covariance
            obj.P = (obj.I - obj.K * obj.H) * obj.P * (obj.I - obj.K * obj.H)' + obj.K * obj.R * obj.K';
        end

        %function to get the current state
        function X_ = getState(obj)
            X_ = obj.X;
        end
    end
end