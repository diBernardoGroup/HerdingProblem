This repository contains the Matlab functions and scripts for initializing, simulating and evaluating the herding task with dynamic selection strategies presented in [1] and two heuristic selection strategies. 


[TrialMaker_EulerMaruyama](TrialMaker_EulerMaruyama.m) run N numerical trials with N initial condition for each strategy selected. Simulation paramenters will be saved in [Paramenters/](Paramenters/) while numerical trials will be saved in [Trials/*](Trials/); each in a subfolder named after the target selection strategy implemented. Performance metrics are computed by [getMetrics](getMetrics.m) function and saved in [Metrics/](Metrics/). 


Additional comments are included throughout to assist with comprehension.



[1] Auletta, F., Fiore, D., Richardson, M.J. & di Bernardo, M. (2020) Herding stochastic autonomous agents via local control rules and online global target selection strategies.  https://arxiv.org/abs/2010.00386v2 , Submitted.

------------------------------------------------------------------------------------------
Author: F. Auletta

E-mail : fabrizia.auletta@hdr.mq.edu.au

