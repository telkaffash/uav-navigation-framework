function [x_upd, P_upd] = ekf_update(x, P, z, H, R, h_fn)
% ekf_update  Generic EKF measurement update (linear or linearised).
%
% Inputs:
%   x     (nx1) predicted state
%   P     (nxn) predicted covariance
%   z     measurement vector
%   H     measurement Jacobian matrix
%   R     measurement noise covariance
%   h_fn  function handle: h_fn(x) returns expected measurement vector
%
% Outputs:
%   x_upd  updated state
%   P_upd  updated covariance (symmetrised)

innov = z - h_fn(x);
S     = H * P * H' + R;
K     = P * H' / S;
x_upd = x + K * innov;
P_upd = (eye(size(P,1)) - K * H) * P;
P_upd = 0.5 * (P_upd + P_upd');   % enforce symmetry

end