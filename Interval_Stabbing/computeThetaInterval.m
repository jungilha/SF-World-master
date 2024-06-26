function [theta_intervals, coeffs] = computeThetaInterval(A1, B1, A2, B2, A3, B3, obs)
% tic
% deriveUniqueSol();
    

% A1=params(1);B1=params(2);A2=params(3);B2=params(4);A3=params(5);B3=params(6); 


a1 = obs(1);
b1 = obs(2);
c1 = obs(3);
% d1

% innerPt = 0.9999*obs;     % short candidate interval
% innerPt = 0.9995*obs; 
innerPt = 0.999*obs; 
% innerPt = 0.995*obs; 
% innerPt = 0.99*obs;        % long candidate interval
innerPt_x = innerPt(1);
innerPt_y = innerPt(2);
innerPt_z = innerPt(3);
d1 = -(a1*innerPt_x+b1*innerPt_y+c1*innerPt_z);




%% define delta and compute theta
a_poly_coeff = (16*A1^2*A2^2*c1^2*d1^2 - 16*A1^2*A2^2*c1^4 - 16*A1^2*A2^2*b1^2*c1^2 + 32*A1^2*A2*A3*b1^3*c1 + 32*A1^2*A2*A3*b1*c1^3 - 32*A1^2*A2*A3*b1*c1*d1^2 - 16*A1^2*A3^2*b1^4 - 16*A1^2*A3^2*b1^2*c1^2 + 16*A1^2*A3^2*b1^2*d1^2 + 32*A1*A2^3*a1*b1*c1^2 - 64*A1*A2^2*A3*a1*b1^2*c1 + 32*A1*A2^2*A3*a1*c1^3 + 32*A1*A2*A3^2*a1*b1^3 - 64*A1*A2*A3^2*a1*b1*c1^2 + 32*A1*A3^3*a1*b1^2*c1 - 16*A2^4*a1^2*c1^2 - 16*A2^4*c1^4 + 16*A2^4*c1^2*d1^2 + 32*A2^3*A3*a1^2*b1*c1 + 64*A2^3*A3*b1*c1^3 - 32*A2^3*A3*b1*c1*d1^2 - 16*A2^2*A3^2*a1^2*b1^2 - 16*A2^2*A3^2*a1^2*c1^2 - 96*A2^2*A3^2*b1^2*c1^2 + 16*A2^2*A3^2*b1^2*d1^2 + 16*A2^2*A3^2*c1^2*d1^2 + 32*A2*A3^3*a1^2*b1*c1 + 64*A2*A3^3*b1^3*c1 - 32*A2*A3^3*b1*c1*d1^2 - 16*A3^4*a1^2*b1^2 - 16*A3^4*b1^4 + 16*A3^4*b1^2*d1^2 + 16*B1^2*B2^2*b1^2*c1^2 + 16*B1^2*B2^2*c1^4 - 16*B1^2*B2^2*c1^2*d1^2 - 32*B1^2*B2*B3*b1^3*c1 - 32*B1^2*B2*B3*b1*c1^3 + 32*B1^2*B2*B3*b1*c1*d1^2 + 16*B1^2*B3^2*b1^4 + 16*B1^2*B3^2*b1^2*c1^2 - 16*B1^2*B3^2*b1^2*d1^2 - 32*B1*B2^3*a1*b1*c1^2 + 64*B1*B2^2*B3*a1*b1^2*c1 - 32*B1*B2^2*B3*a1*c1^3 - 32*B1*B2*B3^2*a1*b1^3 + 64*B1*B2*B3^2*a1*b1*c1^2 - 32*B1*B3^3*a1*b1^2*c1 + 16*B2^4*a1^2*c1^2 + 16*B2^4*c1^4 - 16*B2^4*c1^2*d1^2 - 32*B2^3*B3*a1^2*b1*c1 - 64*B2^3*B3*b1*c1^3 + 32*B2^3*B3*b1*c1*d1^2 + 16*B2^2*B3^2*a1^2*b1^2 + 16*B2^2*B3^2*a1^2*c1^2 + 96*B2^2*B3^2*b1^2*c1^2 - 16*B2^2*B3^2*b1^2*d1^2 - 16*B2^2*B3^2*c1^2*d1^2 - 32*B2*B3^3*a1^2*b1*c1 - 64*B2*B3^3*b1^3*c1 + 32*B2*B3^3*b1*c1*d1^2 + 16*B3^4*a1^2*b1^2 + 16*B3^4*b1^4 - 16*B3^4*b1^2*d1^2);
b_poly_coeff = (4*A1^2*A2^2*c1^2*d1^2 - 4*A1^2*A2^2*c1^4 - 4*A1^2*A2^2*b1^2*c1^2 + 8*A1^2*A2*A3*b1^3*c1 + 8*A1^2*A2*A3*b1*c1^3 - 8*A1^2*A2*A3*b1*c1*d1^2 - 4*A1^2*A3^2*b1^4 - 4*A1^2*A3^2*b1^2*c1^2 + 4*A1^2*A3^2*b1^2*d1^2 + 4*A1^2*B2^2*b1^2*c1^2 + 4*A1^2*B2^2*c1^4 - 4*A1^2*B2^2*c1^2*d1^2 - 8*A1^2*B2*B3*b1^3*c1 - 8*A1^2*B2*B3*b1*c1^3 + 8*A1^2*B2*B3*b1*c1*d1^2 + 4*A1^2*B3^2*b1^4 + 4*A1^2*B3^2*b1^2*c1^2 - 4*A1^2*B3^2*b1^2*d1^2 + 8*A1*A2^3*a1*b1*c1^2 - 16*A1*A2^2*A3*a1*b1^2*c1 + 8*A1*A2^2*A3*a1*c1^3 + 8*A1*A2*A3^2*a1*b1^3 - 16*A1*A2*A3^2*a1*b1*c1^2 + 16*A1*A2*B1*B2*b1^2*c1^2 + 16*A1*A2*B1*B2*c1^4 - 16*A1*A2*B1*B2*c1^2*d1^2 - 16*A1*A2*B1*B3*b1^3*c1 - 16*A1*A2*B1*B3*b1*c1^3 + 16*A1*A2*B1*B3*b1*c1*d1^2 - 24*A1*A2*B2^2*a1*b1*c1^2 + 32*A1*A2*B2*B3*a1*b1^2*c1 - 16*A1*A2*B2*B3*a1*c1^3 - 8*A1*A2*B3^2*a1*b1^3 + 16*A1*A2*B3^2*a1*b1*c1^2 + 8*A1*A3^3*a1*b1^2*c1 - 16*A1*A3*B1*B2*b1^3*c1 - 16*A1*A3*B1*B2*b1*c1^3 + 16*A1*A3*B1*B2*b1*c1*d1^2 + 16*A1*A3*B1*B3*b1^4 + 16*A1*A3*B1*B3*b1^2*c1^2 - 16*A1*A3*B1*B3*b1^2*d1^2 + 16*A1*A3*B2^2*a1*b1^2*c1 - 8*A1*A3*B2^2*a1*c1^3 - 16*A1*A3*B2*B3*a1*b1^3 + 32*A1*A3*B2*B3*a1*b1*c1^2 - 24*A1*A3*B3^2*a1*b1^2*c1 - 4*A2^4*a1^2*c1^2 - 4*A2^4*c1^4 + 4*A2^4*c1^2*d1^2 + 8*A2^3*A3*a1^2*b1*c1 + 16*A2^3*A3*b1*c1^3 - 8*A2^3*A3*b1*c1*d1^2 - 4*A2^2*A3^2*a1^2*b1^2 - 4*A2^2*A3^2*a1^2*c1^2 - 24*A2^2*A3^2*b1^2*c1^2 + 4*A2^2*A3^2*b1^2*d1^2 + 4*A2^2*A3^2*c1^2*d1^2 + 4*A2^2*B1^2*b1^2*c1^2 + 4*A2^2*B1^2*c1^4 - 4*A2^2*B1^2*c1^2*d1^2 - 24*A2^2*B1*B2*a1*b1*c1^2 + 16*A2^2*B1*B3*a1*b1^2*c1 - 8*A2^2*B1*B3*a1*c1^3 + 24*A2^2*B2^2*a1^2*c1^2 + 24*A2^2*B2^2*c1^4 - 24*A2^2*B2^2*c1^2*d1^2 - 24*A2^2*B2*B3*a1^2*b1*c1 - 48*A2^2*B2*B3*b1*c1^3 + 24*A2^2*B2*B3*b1*c1*d1^2 + 4*A2^2*B3^2*a1^2*b1^2 + 4*A2^2*B3^2*a1^2*c1^2 + 24*A2^2*B3^2*b1^2*c1^2 - 4*A2^2*B3^2*b1^2*d1^2 - 4*A2^2*B3^2*c1^2*d1^2 + 8*A2*A3^3*a1^2*b1*c1 + 16*A2*A3^3*b1^3*c1 - 8*A2*A3^3*b1*c1*d1^2 - 8*A2*A3*B1^2*b1^3*c1 - 8*A2*A3*B1^2*b1*c1^3 + 8*A2*A3*B1^2*b1*c1*d1^2 + 32*A2*A3*B1*B2*a1*b1^2*c1 - 16*A2*A3*B1*B2*a1*c1^3 - 16*A2*A3*B1*B3*a1*b1^3 + 32*A2*A3*B1*B3*a1*b1*c1^2 - 24*A2*A3*B2^2*a1^2*b1*c1 - 48*A2*A3*B2^2*b1*c1^3 + 24*A2*A3*B2^2*b1*c1*d1^2 + 16*A2*A3*B2*B3*a1^2*b1^2 + 16*A2*A3*B2*B3*a1^2*c1^2 + 96*A2*A3*B2*B3*b1^2*c1^2 - 16*A2*A3*B2*B3*b1^2*d1^2 - 16*A2*A3*B2*B3*c1^2*d1^2 - 24*A2*A3*B3^2*a1^2*b1*c1 - 48*A2*A3*B3^2*b1^3*c1 + 24*A2*A3*B3^2*b1*c1*d1^2 - 4*A3^4*a1^2*b1^2 - 4*A3^4*b1^4 + 4*A3^4*b1^2*d1^2 + 4*A3^2*B1^2*b1^4 + 4*A3^2*B1^2*b1^2*c1^2 - 4*A3^2*B1^2*b1^2*d1^2 - 8*A3^2*B1*B2*a1*b1^3 + 16*A3^2*B1*B2*a1*b1*c1^2 - 24*A3^2*B1*B3*a1*b1^2*c1 + 4*A3^2*B2^2*a1^2*b1^2 + 4*A3^2*B2^2*a1^2*c1^2 + 24*A3^2*B2^2*b1^2*c1^2 - 4*A3^2*B2^2*b1^2*d1^2 - 4*A3^2*B2^2*c1^2*d1^2 - 24*A3^2*B2*B3*a1^2*b1*c1 - 48*A3^2*B2*B3*b1^3*c1 + 24*A3^2*B2*B3*b1*c1*d1^2 + 24*A3^2*B3^2*a1^2*b1^2 + 24*A3^2*B3^2*b1^4 - 24*A3^2*B3^2*b1^2*d1^2 - 4*B1^2*B2^2*b1^2*c1^2 - 4*B1^2*B2^2*c1^4 + 4*B1^2*B2^2*c1^2*d1^2 + 8*B1^2*B2*B3*b1^3*c1 + 8*B1^2*B2*B3*b1*c1^3 - 8*B1^2*B2*B3*b1*c1*d1^2 - 4*B1^2*B3^2*b1^4 - 4*B1^2*B3^2*b1^2*c1^2 + 4*B1^2*B3^2*b1^2*d1^2 + 8*B1*B2^3*a1*b1*c1^2 - 16*B1*B2^2*B3*a1*b1^2*c1 + 8*B1*B2^2*B3*a1*c1^3 + 8*B1*B2*B3^2*a1*b1^3 - 16*B1*B2*B3^2*a1*b1*c1^2 + 8*B1*B3^3*a1*b1^2*c1 - 4*B2^4*a1^2*c1^2 - 4*B2^4*c1^4 + 4*B2^4*c1^2*d1^2 + 8*B2^3*B3*a1^2*b1*c1 + 16*B2^3*B3*b1*c1^3 - 8*B2^3*B3*b1*c1*d1^2 - 4*B2^2*B3^2*a1^2*b1^2 - 4*B2^2*B3^2*a1^2*c1^2 - 24*B2^2*B3^2*b1^2*c1^2 + 4*B2^2*B3^2*b1^2*d1^2 + 4*B2^2*B3^2*c1^2*d1^2 + 8*B2*B3^3*a1^2*b1*c1 + 16*B2*B3^3*b1^3*c1 - 8*B2*B3^3*b1*c1*d1^2 - 4*B3^4*a1^2*b1^2 - 4*B3^4*b1^4 + 4*B3^4*b1^2*d1^2);
c_poly_coeff = (16*A1^2*A2*B2*c1^2*d1^2 - 16*A1^2*A2*B2*c1^4 - 16*A1^2*A2*B2*b1^2*c1^2 + 16*A1^2*A2*B3*b1^3*c1 + 16*A1^2*A2*B3*b1*c1^3 - 16*A1^2*A2*B3*b1*c1*d1^2 + 16*A1^2*A3*B2*b1^3*c1 + 16*A1^2*A3*B2*b1*c1^3 - 16*A1^2*A3*B2*b1*c1*d1^2 - 16*A1^2*A3*B3*b1^4 - 16*A1^2*A3*B3*b1^2*c1^2 + 16*A1^2*A3*B3*b1^2*d1^2 - 16*A1*A2^2*B1*b1^2*c1^2 - 16*A1*A2^2*B1*c1^4 + 16*A1*A2^2*B1*c1^2*d1^2 + 48*A1*A2^2*B2*a1*b1*c1^2 - 32*A1*A2^2*B3*a1*b1^2*c1 + 16*A1*A2^2*B3*a1*c1^3 + 32*A1*A2*A3*B1*b1^3*c1 + 32*A1*A2*A3*B1*b1*c1^3 - 32*A1*A2*A3*B1*b1*c1*d1^2 - 64*A1*A2*A3*B2*a1*b1^2*c1 + 32*A1*A2*A3*B2*a1*c1^3 + 32*A1*A2*A3*B3*a1*b1^3 - 64*A1*A2*A3*B3*a1*b1*c1^2 - 16*A1*A3^2*B1*b1^4 - 16*A1*A3^2*B1*b1^2*c1^2 + 16*A1*A3^2*B1*b1^2*d1^2 + 16*A1*A3^2*B2*a1*b1^3 - 32*A1*A3^2*B2*a1*b1*c1^2 + 48*A1*A3^2*B3*a1*b1^2*c1 - 16*A1*B1*B2^2*b1^2*c1^2 - 16*A1*B1*B2^2*c1^4 + 16*A1*B1*B2^2*c1^2*d1^2 + 32*A1*B1*B2*B3*b1^3*c1 + 32*A1*B1*B2*B3*b1*c1^3 - 32*A1*B1*B2*B3*b1*c1*d1^2 - 16*A1*B1*B3^2*b1^4 - 16*A1*B1*B3^2*b1^2*c1^2 + 16*A1*B1*B3^2*b1^2*d1^2 + 16*A1*B2^3*a1*b1*c1^2 - 32*A1*B2^2*B3*a1*b1^2*c1 + 16*A1*B2^2*B3*a1*c1^3 + 16*A1*B2*B3^2*a1*b1^3 - 32*A1*B2*B3^2*a1*b1*c1^2 + 16*A1*B3^3*a1*b1^2*c1 + 16*A2^3*B1*a1*b1*c1^2 - 32*A2^3*B2*a1^2*c1^2 - 32*A2^3*B2*c1^4 + 32*A2^3*B2*c1^2*d1^2 + 16*A2^3*B3*a1^2*b1*c1 + 32*A2^3*B3*b1*c1^3 - 16*A2^3*B3*b1*c1*d1^2 - 32*A2^2*A3*B1*a1*b1^2*c1 + 16*A2^2*A3*B1*a1*c1^3 + 48*A2^2*A3*B2*a1^2*b1*c1 + 96*A2^2*A3*B2*b1*c1^3 - 48*A2^2*A3*B2*b1*c1*d1^2 - 16*A2^2*A3*B3*a1^2*b1^2 - 16*A2^2*A3*B3*a1^2*c1^2 - 96*A2^2*A3*B3*b1^2*c1^2 + 16*A2^2*A3*B3*b1^2*d1^2 + 16*A2^2*A3*B3*c1^2*d1^2 + 16*A2*A3^2*B1*a1*b1^3 - 32*A2*A3^2*B1*a1*b1*c1^2 - 16*A2*A3^2*B2*a1^2*b1^2 - 16*A2*A3^2*B2*a1^2*c1^2 - 96*A2*A3^2*B2*b1^2*c1^2 + 16*A2*A3^2*B2*b1^2*d1^2 + 16*A2*A3^2*B2*c1^2*d1^2 + 48*A2*A3^2*B3*a1^2*b1*c1 + 96*A2*A3^2*B3*b1^3*c1 - 48*A2*A3^2*B3*b1*c1*d1^2 - 16*A2*B1^2*B2*b1^2*c1^2 - 16*A2*B1^2*B2*c1^4 + 16*A2*B1^2*B2*c1^2*d1^2 + 16*A2*B1^2*B3*b1^3*c1 + 16*A2*B1^2*B3*b1*c1^3 - 16*A2*B1^2*B3*b1*c1*d1^2 + 48*A2*B1*B2^2*a1*b1*c1^2 - 64*A2*B1*B2*B3*a1*b1^2*c1 + 32*A2*B1*B2*B3*a1*c1^3 + 16*A2*B1*B3^2*a1*b1^3 - 32*A2*B1*B3^2*a1*b1*c1^2 - 32*A2*B2^3*a1^2*c1^2 - 32*A2*B2^3*c1^4 + 32*A2*B2^3*c1^2*d1^2 + 48*A2*B2^2*B3*a1^2*b1*c1 + 96*A2*B2^2*B3*b1*c1^3 - 48*A2*B2^2*B3*b1*c1*d1^2 - 16*A2*B2*B3^2*a1^2*b1^2 - 16*A2*B2*B3^2*a1^2*c1^2 - 96*A2*B2*B3^2*b1^2*c1^2 + 16*A2*B2*B3^2*b1^2*d1^2 + 16*A2*B2*B3^2*c1^2*d1^2 + 16*A2*B3^3*a1^2*b1*c1 + 32*A2*B3^3*b1^3*c1 - 16*A2*B3^3*b1*c1*d1^2 + 16*A3^3*B1*a1*b1^2*c1 + 16*A3^3*B2*a1^2*b1*c1 + 32*A3^3*B2*b1^3*c1 - 16*A3^3*B2*b1*c1*d1^2 - 32*A3^3*B3*a1^2*b1^2 - 32*A3^3*B3*b1^4 + 32*A3^3*B3*b1^2*d1^2 + 16*A3*B1^2*B2*b1^3*c1 + 16*A3*B1^2*B2*b1*c1^3 - 16*A3*B1^2*B2*b1*c1*d1^2 - 16*A3*B1^2*B3*b1^4 - 16*A3*B1^2*B3*b1^2*c1^2 + 16*A3*B1^2*B3*b1^2*d1^2 - 32*A3*B1*B2^2*a1*b1^2*c1 + 16*A3*B1*B2^2*a1*c1^3 + 32*A3*B1*B2*B3*a1*b1^3 - 64*A3*B1*B2*B3*a1*b1*c1^2 + 48*A3*B1*B3^2*a1*b1^2*c1 + 16*A3*B2^3*a1^2*b1*c1 + 32*A3*B2^3*b1*c1^3 - 16*A3*B2^3*b1*c1*d1^2 - 16*A3*B2^2*B3*a1^2*b1^2 - 16*A3*B2^2*B3*a1^2*c1^2 - 96*A3*B2^2*B3*b1^2*c1^2 + 16*A3*B2^2*B3*b1^2*d1^2 + 16*A3*B2^2*B3*c1^2*d1^2 + 48*A3*B2*B3^2*a1^2*b1*c1 + 96*A3*B2*B3^2*b1^3*c1 - 48*A3*B2*B3^2*b1*c1*d1^2 - 32*A3*B3^3*a1^2*b1^2 - 32*A3*B3^3*b1^4 + 32*A3*B3^3*b1^2*d1^2);
d_poly_coeff = (8*A1^2*A2*B2*c1^2*d1^2 - 8*A1^2*A2*B2*c1^4 - 8*A1^2*A2*B2*b1^2*c1^2 + 8*A1^2*A2*B3*b1^3*c1 + 8*A1^2*A2*B3*b1*c1^3 - 8*A1^2*A2*B3*b1*c1*d1^2 + 8*A1^2*A3*B2*b1^3*c1 + 8*A1^2*A3*B2*b1*c1^3 - 8*A1^2*A3*B2*b1*c1*d1^2 - 8*A1^2*A3*B3*b1^4 - 8*A1^2*A3*B3*b1^2*c1^2 + 8*A1^2*A3*B3*b1^2*d1^2 - 8*A1*A2^2*B1*b1^2*c1^2 - 8*A1*A2^2*B1*c1^4 + 8*A1*A2^2*B1*c1^2*d1^2 + 24*A1*A2^2*B2*a1*b1*c1^2 - 16*A1*A2^2*B3*a1*b1^2*c1 + 8*A1*A2^2*B3*a1*c1^3 + 16*A1*A2*A3*B1*b1^3*c1 + 16*A1*A2*A3*B1*b1*c1^3 - 16*A1*A2*A3*B1*b1*c1*d1^2 - 32*A1*A2*A3*B2*a1*b1^2*c1 + 16*A1*A2*A3*B2*a1*c1^3 + 16*A1*A2*A3*B3*a1*b1^3 - 32*A1*A2*A3*B3*a1*b1*c1^2 - 8*A1*A3^2*B1*b1^4 - 8*A1*A3^2*B1*b1^2*c1^2 + 8*A1*A3^2*B1*b1^2*d1^2 + 8*A1*A3^2*B2*a1*b1^3 - 16*A1*A3^2*B2*a1*b1*c1^2 + 24*A1*A3^2*B3*a1*b1^2*c1 + 8*A1*B1*B2^2*b1^2*c1^2 + 8*A1*B1*B2^2*c1^4 - 8*A1*B1*B2^2*c1^2*d1^2 - 16*A1*B1*B2*B3*b1^3*c1 - 16*A1*B1*B2*B3*b1*c1^3 + 16*A1*B1*B2*B3*b1*c1*d1^2 + 8*A1*B1*B3^2*b1^4 + 8*A1*B1*B3^2*b1^2*c1^2 - 8*A1*B1*B3^2*b1^2*d1^2 - 8*A1*B2^3*a1*b1*c1^2 + 16*A1*B2^2*B3*a1*b1^2*c1 - 8*A1*B2^2*B3*a1*c1^3 - 8*A1*B2*B3^2*a1*b1^3 + 16*A1*B2*B3^2*a1*b1*c1^2 - 8*A1*B3^3*a1*b1^2*c1 + 8*A2^3*B1*a1*b1*c1^2 - 16*A2^3*B2*a1^2*c1^2 - 16*A2^3*B2*c1^4 + 16*A2^3*B2*c1^2*d1^2 + 8*A2^3*B3*a1^2*b1*c1 + 16*A2^3*B3*b1*c1^3 - 8*A2^3*B3*b1*c1*d1^2 - 16*A2^2*A3*B1*a1*b1^2*c1 + 8*A2^2*A3*B1*a1*c1^3 + 24*A2^2*A3*B2*a1^2*b1*c1 + 48*A2^2*A3*B2*b1*c1^3 - 24*A2^2*A3*B2*b1*c1*d1^2 - 8*A2^2*A3*B3*a1^2*b1^2 - 8*A2^2*A3*B3*a1^2*c1^2 - 48*A2^2*A3*B3*b1^2*c1^2 + 8*A2^2*A3*B3*b1^2*d1^2 + 8*A2^2*A3*B3*c1^2*d1^2 + 8*A2*A3^2*B1*a1*b1^3 - 16*A2*A3^2*B1*a1*b1*c1^2 - 8*A2*A3^2*B2*a1^2*b1^2 - 8*A2*A3^2*B2*a1^2*c1^2 - 48*A2*A3^2*B2*b1^2*c1^2 + 8*A2*A3^2*B2*b1^2*d1^2 + 8*A2*A3^2*B2*c1^2*d1^2 + 24*A2*A3^2*B3*a1^2*b1*c1 + 48*A2*A3^2*B3*b1^3*c1 - 24*A2*A3^2*B3*b1*c1*d1^2 + 8*A2*B1^2*B2*b1^2*c1^2 + 8*A2*B1^2*B2*c1^4 - 8*A2*B1^2*B2*c1^2*d1^2 - 8*A2*B1^2*B3*b1^3*c1 - 8*A2*B1^2*B3*b1*c1^3 + 8*A2*B1^2*B3*b1*c1*d1^2 - 24*A2*B1*B2^2*a1*b1*c1^2 + 32*A2*B1*B2*B3*a1*b1^2*c1 - 16*A2*B1*B2*B3*a1*c1^3 - 8*A2*B1*B3^2*a1*b1^3 + 16*A2*B1*B3^2*a1*b1*c1^2 + 16*A2*B2^3*a1^2*c1^2 + 16*A2*B2^3*c1^4 - 16*A2*B2^3*c1^2*d1^2 - 24*A2*B2^2*B3*a1^2*b1*c1 - 48*A2*B2^2*B3*b1*c1^3 + 24*A2*B2^2*B3*b1*c1*d1^2 + 8*A2*B2*B3^2*a1^2*b1^2 + 8*A2*B2*B3^2*a1^2*c1^2 + 48*A2*B2*B3^2*b1^2*c1^2 - 8*A2*B2*B3^2*b1^2*d1^2 - 8*A2*B2*B3^2*c1^2*d1^2 - 8*A2*B3^3*a1^2*b1*c1 - 16*A2*B3^3*b1^3*c1 + 8*A2*B3^3*b1*c1*d1^2 + 8*A3^3*B1*a1*b1^2*c1 + 8*A3^3*B2*a1^2*b1*c1 + 16*A3^3*B2*b1^3*c1 - 8*A3^3*B2*b1*c1*d1^2 - 16*A3^3*B3*a1^2*b1^2 - 16*A3^3*B3*b1^4 + 16*A3^3*B3*b1^2*d1^2 - 8*A3*B1^2*B2*b1^3*c1 - 8*A3*B1^2*B2*b1*c1^3 + 8*A3*B1^2*B2*b1*c1*d1^2 + 8*A3*B1^2*B3*b1^4 + 8*A3*B1^2*B3*b1^2*c1^2 - 8*A3*B1^2*B3*b1^2*d1^2 + 16*A3*B1*B2^2*a1*b1^2*c1 - 8*A3*B1*B2^2*a1*c1^3 - 16*A3*B1*B2*B3*a1*b1^3 + 32*A3*B1*B2*B3*a1*b1*c1^2 - 24*A3*B1*B3^2*a1*b1^2*c1 - 8*A3*B2^3*a1^2*b1*c1 - 16*A3*B2^3*b1*c1^3 + 8*A3*B2^3*b1*c1*d1^2 + 8*A3*B2^2*B3*a1^2*b1^2 + 8*A3*B2^2*B3*a1^2*c1^2 + 48*A3*B2^2*B3*b1^2*c1^2 - 8*A3*B2^2*B3*b1^2*d1^2 - 8*A3*B2^2*B3*c1^2*d1^2 - 24*A3*B2*B3^2*a1^2*b1*c1 - 48*A3*B2*B3^2*b1^3*c1 + 24*A3*B2*B3^2*b1*c1*d1^2 + 16*A3*B3^3*a1^2*b1^2 + 16*A3*B3^3*b1^4 - 16*A3*B3^3*b1^2*d1^2);
e_poly_coeff = (12*A1^2*A2^2*c1^2*d1^2 - 12*A1^2*A2^2*c1^4 - 12*A1^2*A2^2*b1^2*c1^2 + 24*A1^2*A2*A3*b1^3*c1 + 24*A1^2*A2*A3*b1*c1^3 - 24*A1^2*A2*A3*b1*c1*d1^2 - 12*A1^2*A3^2*b1^4 - 12*A1^2*A3^2*b1^2*c1^2 + 12*A1^2*A3^2*b1^2*d1^2 - 4*A1^2*B2^2*b1^2*c1^2 - 4*A1^2*B2^2*c1^4 + 4*A1^2*B2^2*c1^2*d1^2 + 8*A1^2*B2*B3*b1^3*c1 + 8*A1^2*B2*B3*b1*c1^3 - 8*A1^2*B2*B3*b1*c1*d1^2 - 4*A1^2*B3^2*b1^4 - 4*A1^2*B3^2*b1^2*c1^2 + 4*A1^2*B3^2*b1^2*d1^2 + 24*A1*A2^3*a1*b1*c1^2 - 48*A1*A2^2*A3*a1*b1^2*c1 + 24*A1*A2^2*A3*a1*c1^3 + 24*A1*A2*A3^2*a1*b1^3 - 48*A1*A2*A3^2*a1*b1*c1^2 - 16*A1*A2*B1*B2*b1^2*c1^2 - 16*A1*A2*B1*B2*c1^4 + 16*A1*A2*B1*B2*c1^2*d1^2 + 16*A1*A2*B1*B3*b1^3*c1 + 16*A1*A2*B1*B3*b1*c1^3 - 16*A1*A2*B1*B3*b1*c1*d1^2 + 24*A1*A2*B2^2*a1*b1*c1^2 - 32*A1*A2*B2*B3*a1*b1^2*c1 + 16*A1*A2*B2*B3*a1*c1^3 + 8*A1*A2*B3^2*a1*b1^3 - 16*A1*A2*B3^2*a1*b1*c1^2 + 24*A1*A3^3*a1*b1^2*c1 + 16*A1*A3*B1*B2*b1^3*c1 + 16*A1*A3*B1*B2*b1*c1^3 - 16*A1*A3*B1*B2*b1*c1*d1^2 - 16*A1*A3*B1*B3*b1^4 - 16*A1*A3*B1*B3*b1^2*c1^2 + 16*A1*A3*B1*B3*b1^2*d1^2 - 16*A1*A3*B2^2*a1*b1^2*c1 + 8*A1*A3*B2^2*a1*c1^3 + 16*A1*A3*B2*B3*a1*b1^3 - 32*A1*A3*B2*B3*a1*b1*c1^2 + 24*A1*A3*B3^2*a1*b1^2*c1 - 12*A2^4*a1^2*c1^2 - 12*A2^4*c1^4 + 12*A2^4*c1^2*d1^2 + 24*A2^3*A3*a1^2*b1*c1 + 48*A2^3*A3*b1*c1^3 - 24*A2^3*A3*b1*c1*d1^2 - 12*A2^2*A3^2*a1^2*b1^2 - 12*A2^2*A3^2*a1^2*c1^2 - 72*A2^2*A3^2*b1^2*c1^2 + 12*A2^2*A3^2*b1^2*d1^2 + 12*A2^2*A3^2*c1^2*d1^2 - 4*A2^2*B1^2*b1^2*c1^2 - 4*A2^2*B1^2*c1^4 + 4*A2^2*B1^2*c1^2*d1^2 + 24*A2^2*B1*B2*a1*b1*c1^2 - 16*A2^2*B1*B3*a1*b1^2*c1 + 8*A2^2*B1*B3*a1*c1^3 - 24*A2^2*B2^2*a1^2*c1^2 - 24*A2^2*B2^2*c1^4 + 24*A2^2*B2^2*c1^2*d1^2 + 24*A2^2*B2*B3*a1^2*b1*c1 + 48*A2^2*B2*B3*b1*c1^3 - 24*A2^2*B2*B3*b1*c1*d1^2 - 4*A2^2*B3^2*a1^2*b1^2 - 4*A2^2*B3^2*a1^2*c1^2 - 24*A2^2*B3^2*b1^2*c1^2 + 4*A2^2*B3^2*b1^2*d1^2 + 4*A2^2*B3^2*c1^2*d1^2 + 24*A2*A3^3*a1^2*b1*c1 + 48*A2*A3^3*b1^3*c1 - 24*A2*A3^3*b1*c1*d1^2 + 8*A2*A3*B1^2*b1^3*c1 + 8*A2*A3*B1^2*b1*c1^3 - 8*A2*A3*B1^2*b1*c1*d1^2 - 32*A2*A3*B1*B2*a1*b1^2*c1 + 16*A2*A3*B1*B2*a1*c1^3 + 16*A2*A3*B1*B3*a1*b1^3 - 32*A2*A3*B1*B3*a1*b1*c1^2 + 24*A2*A3*B2^2*a1^2*b1*c1 + 48*A2*A3*B2^2*b1*c1^3 - 24*A2*A3*B2^2*b1*c1*d1^2 - 16*A2*A3*B2*B3*a1^2*b1^2 - 16*A2*A3*B2*B3*a1^2*c1^2 - 96*A2*A3*B2*B3*b1^2*c1^2 + 16*A2*A3*B2*B3*b1^2*d1^2 + 16*A2*A3*B2*B3*c1^2*d1^2 + 24*A2*A3*B3^2*a1^2*b1*c1 + 48*A2*A3*B3^2*b1^3*c1 - 24*A2*A3*B3^2*b1*c1*d1^2 - 12*A3^4*a1^2*b1^2 - 12*A3^4*b1^4 + 12*A3^4*b1^2*d1^2 - 4*A3^2*B1^2*b1^4 - 4*A3^2*B1^2*b1^2*c1^2 + 4*A3^2*B1^2*b1^2*d1^2 + 8*A3^2*B1*B2*a1*b1^3 - 16*A3^2*B1*B2*a1*b1*c1^2 + 24*A3^2*B1*B3*a1*b1^2*c1 - 4*A3^2*B2^2*a1^2*b1^2 - 4*A3^2*B2^2*a1^2*c1^2 - 24*A3^2*B2^2*b1^2*c1^2 + 4*A3^2*B2^2*b1^2*d1^2 + 4*A3^2*B2^2*c1^2*d1^2 + 24*A3^2*B2*B3*a1^2*b1*c1 + 48*A3^2*B2*B3*b1^3*c1 - 24*A3^2*B2*B3*b1*c1*d1^2 - 24*A3^2*B3^2*a1^2*b1^2 - 24*A3^2*B3^2*b1^4 + 24*A3^2*B3^2*b1^2*d1^2 - 12*B1^2*B2^2*b1^2*c1^2 - 12*B1^2*B2^2*c1^4 + 12*B1^2*B2^2*c1^2*d1^2 + 24*B1^2*B2*B3*b1^3*c1 + 24*B1^2*B2*B3*b1*c1^3 - 24*B1^2*B2*B3*b1*c1*d1^2 - 12*B1^2*B3^2*b1^4 - 12*B1^2*B3^2*b1^2*c1^2 + 12*B1^2*B3^2*b1^2*d1^2 + 24*B1*B2^3*a1*b1*c1^2 - 48*B1*B2^2*B3*a1*b1^2*c1 + 24*B1*B2^2*B3*a1*c1^3 + 24*B1*B2*B3^2*a1*b1^3 - 48*B1*B2*B3^2*a1*b1*c1^2 + 24*B1*B3^3*a1*b1^2*c1 - 12*B2^4*a1^2*c1^2 - 12*B2^4*c1^4 + 12*B2^4*c1^2*d1^2 + 24*B2^3*B3*a1^2*b1*c1 + 48*B2^3*B3*b1*c1^3 - 24*B2^3*B3*b1*c1*d1^2 - 12*B2^2*B3^2*a1^2*b1^2 - 12*B2^2*B3^2*a1^2*c1^2 - 72*B2^2*B3^2*b1^2*c1^2 + 12*B2^2*B3^2*b1^2*d1^2 + 12*B2^2*B3^2*c1^2*d1^2 + 24*B2*B3^3*a1^2*b1*c1 + 48*B2*B3^3*b1^3*c1 - 24*B2*B3^3*b1*c1*d1^2 - 12*B3^4*a1^2*b1^2 - 12*B3^4*b1^4 + 12*B3^4*b1^2*d1^2);

a_muPoly_coeff = (4*A2^4*c1^4 - 16*A2^3*A3*b1*c1^3 + 24*A2^2*A3^2*b1^2*c1^2 - 16*A2*A3^3*b1^3*c1 + 4*A3^4*b1^4 - 4*B2^4*c1^4 + 16*B2^3*B3*b1*c1^3 - 24*B2^2*B3^2*b1^2*c1^2 + 16*B2*B3^3*b1^3*c1 - 4*B3^4*b1^4);
b_muPoly_coeff = (A2^4*c1^4 - 4*A2^3*A3*b1*c1^3 + 6*A2^2*A3^2*b1^2*c1^2 - 6*A2^2*B2^2*c1^4 + 12*A2^2*B2*B3*b1*c1^3 - 6*A2^2*B3^2*b1^2*c1^2 - 4*A2*A3^3*b1^3*c1 + 12*A2*A3*B2^2*b1*c1^3 - 24*A2*A3*B2*B3*b1^2*c1^2 + 12*A2*A3*B3^2*b1^3*c1 + A3^4*b1^4 - 6*A3^2*B2^2*b1^2*c1^2 + 12*A3^2*B2*B3*b1^3*c1 - 6*A3^2*B3^2*b1^4 + B2^4*c1^4 - 4*B2^3*B3*b1*c1^3 + 6*B2^2*B3^2*b1^2*c1^2 - 4*B2*B3^3*b1^3*c1 + B3^4*b1^4);
c_muPoly_coeff = (8*A2^3*B2*c1^4 - 8*A2^3*B3*b1*c1^3 - 24*A2^2*A3*B2*b1*c1^3 + 24*A2^2*A3*B3*b1^2*c1^2 + 24*A2*A3^2*B2*b1^2*c1^2 - 24*A2*A3^2*B3*b1^3*c1 + 8*A2*B2^3*c1^4 - 24*A2*B2^2*B3*b1*c1^3 + 24*A2*B2*B3^2*b1^2*c1^2 - 8*A2*B3^3*b1^3*c1 - 8*A3^3*B2*b1^3*c1 + 8*A3^3*B3*b1^4 - 8*A3*B2^3*b1*c1^3 + 24*A3*B2^2*B3*b1^2*c1^2 - 24*A3*B2*B3^2*b1^3*c1 + 8*A3*B3^3*b1^4);
d_muPoly_coeff = (4*A2^3*B2*c1^4 - 4*A2^3*B3*b1*c1^3 - 12*A2^2*A3*B2*b1*c1^3 + 12*A2^2*A3*B3*b1^2*c1^2 + 12*A2*A3^2*B2*b1^2*c1^2 - 12*A2*A3^2*B3*b1^3*c1 - 4*A2*B2^3*c1^4 + 12*A2*B2^2*B3*b1*c1^3 - 12*A2*B2*B3^2*b1^2*c1^2 + 4*A2*B3^3*b1^3*c1 - 4*A3^3*B2*b1^3*c1 + 4*A3^3*B3*b1^4 + 4*A3*B2^3*b1*c1^3 - 12*A3*B2^2*B3*b1^2*c1^2 + 12*A3*B2*B3^2*b1^3*c1 - 4*A3*B3^3*b1^4);
e_muPoly_coeff = 3*A2^4*c1^4 - 12*A2^3*A3*b1*c1^3 + 18*A2^2*A3^2*b1^2*c1^2 + 6*A2^2*B2^2*c1^4 - 12*A2^2*B2*B3*b1*c1^3 + 6*A2^2*B3^2*b1^2*c1^2 - 12*A2*A3^3*b1^3*c1 - 12*A2*A3*B2^2*b1*c1^3 + 24*A2*A3*B2*B3*b1^2*c1^2 - 12*A2*A3*B3^2*b1^3*c1 + 3*A3^4*b1^4 + 6*A3^2*B2^2*b1^2*c1^2 - 12*A3^2*B2*B3*b1^3*c1 + 6*A3^2*B3^2*b1^4 + 3*B2^4*c1^4 - 12*B2^3*B3*b1*c1^3 + 18*B2^2*B3^2*b1^2*c1^2 - 12*B2*B3^3*b1^3*c1 + 3*B3^4*b1^4;




% *q^4 + *q^3 + *q^2 + *q + 
mucoeffs = [
 (b_muPoly_coeff - a_muPoly_coeff + e_muPoly_coeff), ...
 (2*c_muPoly_coeff - 4*d_muPoly_coeff), ...
 (2*e_muPoly_coeff - 6*b_muPoly_coeff), ...
 (2*c_muPoly_coeff + 4*d_muPoly_coeff), ...
 a_muPoly_coeff + b_muPoly_coeff + e_muPoly_coeff];

coeffs = [
 (b_poly_coeff - a_poly_coeff + e_poly_coeff), ...
 (2*c_poly_coeff - 4*d_poly_coeff), ...
 (2*e_poly_coeff - 6*b_poly_coeff), ...
 (2*c_poly_coeff + 4*d_poly_coeff), ...
 a_poly_coeff + b_poly_coeff + e_poly_coeff];


tan_t_roots = roots(coeffs);
theta_results_all = atan(tan_t_roots);

theta_results = [];
for i = 1:size(theta_results_all, 1)
    if isreal( theta_results_all(i) )
        theta_results = [theta_results; real(theta_results_all(i)) ];
    end
end


theta_results = sort(theta_results);
similar_ids = [];
for i = 1:size(theta_results, 1)-1
   if abs(theta_results(i) - theta_results(i+1))<1e-4
       similar_ids = [similar_ids; [i, i+1]];
   end
end


if size(similar_ids, 1) ==2
    if abs(theta_results(similar_ids(1, 1))-theta_results(similar_ids(1,2))) < ...
            abs(theta_results(similar_ids(2, 1))-theta_results(similar_ids(2,2))) % ͷ�����Ⱥ���������
        similar_ids = similar_ids(1,:);
    else
        similar_ids = similar_ids(2,:);
    end
end

theta_results(similar_ids)=[];

final_thetas = [];

if size(theta_results, 1) == 0
   theta_intervals = [];
   coeffs  = [];
   return
end


if size(theta_results, 1) == 2
    final_thetas = theta_results;
    
else   
    for i = 1:size(theta_results, 1)
    
    %     if theta_results(i) < 0
    %        theta_results(i) = theta_results(i)+pi; 
    %     end

    %     if ismembertol(theta_results(i), final_thetas, 1e-5)
    %         continue; 
    %     end

        theta_test = theta_results(i);
        % �����Ǽ���delta
        tan_t_test = tan(theta_test);
        items_test = [tan_t_test^4, tan_t_test^3, tan_t_test^2, tan_t_test, 1]';
        fenzi_test = coeffs*items_test;
        fenmu_test = mucoeffs*items_test; 

        if fenzi_test == 0 && abs(fenmu_test) < 1e-15 
            continue;
        end
        delta_test = ...
            -( ... 
            fenzi_test ...
            ) ...
            / ...
            ( ...
            fenmu_test ...
            );
        if abs( delta_test )<1e-1  
            final_thetas = [final_thetas; theta_test];
        end
    end
    
end




% final_thetas*180/pi
theta_intervals = [];

if size(final_thetas, 1)==2
    %midTheta = mean(final_thetas);
    midTheta = (1/3) * final_thetas(1) + (2/3) * final_thetas(2);
    theta_test = midTheta;
    
    tan_t_test = tan(theta_test);
    items_test = [tan_t_test^4, tan_t_test^3, tan_t_test^2, tan_t_test, 1]';
    fenzi_test = coeffs*items_test;
    fenmu_test = mucoeffs*items_test;
    
    delta_mid_test = ...
        -( ... 
        fenzi_test ...
        ) ...
        / ...
        ( ...
        fenmu_test ...
        );
    
    
    % check denominator (fenmu_test) stability
    if (abs(fenmu_test) < 10^-15)
        midTheta = mean(final_thetas);
        theta_test = midTheta;
        
        tan_t_test = tan(theta_test);
        items_test = [tan_t_test^4, tan_t_test^3, tan_t_test^2, tan_t_test, 1]';
        fenzi_test = coeffs*items_test;
        fenmu_test = mucoeffs*items_test;
        
        delta_mid_test = ...
            -( ...
            fenzi_test ...
            ) ...
            / ...
            ( ...
            fenmu_test ...
            );
    end
    
    if delta_mid_test>0 
        theta_intervals = [min(final_thetas), max(final_thetas)];
    else  % ���
%         disp('split interval!')
        theta_intervals = [theta_intervals; [max(final_thetas), pi - abs(min(final_thetas))] ];
        %theta_intervals = [theta_intervals; [max(final_thetas), pi/2] ];
    end
    
elseif size(final_thetas, 1)>2

end
   


end



