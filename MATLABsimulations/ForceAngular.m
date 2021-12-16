function T = ForceAngular(theta_pos, xsi, thetaps)

global eps_theta

T = eps_theta * (theta_pos - xsi *thetaps); 