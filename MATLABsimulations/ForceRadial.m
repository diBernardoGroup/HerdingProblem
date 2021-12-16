function F = ForceRadial(yrpos, xsi, rps)

global eps_r rstar delta_rmin

F = eps_r .* (yrpos - xsi .* (rps + rstar) - (1 - xsi) .* (rstar + delta_rmin));

