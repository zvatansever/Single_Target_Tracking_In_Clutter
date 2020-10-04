[z_ingate,~] = function EllipsoidalGating(object, z, sensor, gatingsize)

% predicted state : object.x
% predicted covariance: object.covariance
% measurement model Jacobian: sensor.h
% measurement noise: sensor.R

%measurement model Jacobian
H_jacobian=sensor.H(object.x);

%innovation covariance
S=H_jacobian*object.covariance*H_jacobian'+sensor.R;
S=(S+S')/2;

%predicted measurement

z_predicted=H_jacobian*object.X;

%innovation

[~,N]=size(z);
innovation=z-z_predicted;

% squared mahalanobis distance
            dm = zeros(1,N);
            
            for n = 1:N
                dm(n) = innovation(:,n)' * inv(S) * innovation(:,n);
            end
            
            meas_in_gate = (dm < gating_size)';
            z_ingate = z(:,dm < gating_size);
        end


