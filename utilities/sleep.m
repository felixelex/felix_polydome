function [ wallclock ] = sleep( T_s, wallclock )
% It is used to execute a certain activity, typically in a for loop, every Ts
% seconds while compensanting for the time drift due to computation.

% When called without arguments, it returns the current matlab time.
% It can be used for an intial accurate setting of the wallclock (for
% second accuracy it is not required).

% See desl-development/container/disp_feeder/BMPC/testing/test_sleep_function.m for an example.

curtime = now();

if nargin == 0
    wallclock = curtime;
    return
end

if wallclock == -1
    wallclock = curtime;
end
% denotes the time (matlab timestamp) of the next activation
wallclock = wallclock + T_s/3600/24;

pause_s = ((wallclock) - curtime)*3600*24;
pause(pause_s);
end

