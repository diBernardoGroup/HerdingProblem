function csi = getXsi(rps)

global rstar



if abs(rps - rstar) >= 0
    csi = 1;
else
    csi = 0;
end