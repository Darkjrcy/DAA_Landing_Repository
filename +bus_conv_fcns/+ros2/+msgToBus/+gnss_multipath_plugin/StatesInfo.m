function slBusOut = StatesInfo(msgIn, slBusOut, varargin)
%#codegen
%   Copyright 2021-2022 The MathWorks, Inc.
    slBusOut.north = double(msgIn.north);
    slBusOut.east = double(msgIn.east);
    slBusOut.up = double(msgIn.up);
    slBusOut.v_north = double(msgIn.v_north);
    slBusOut.v_east = double(msgIn.v_east);
    slBusOut.v_up = double(msgIn.v_up);
    slBusOut.acc_up = double(msgIn.acc_up);
    slBusOut.v_tot = double(msgIn.v_tot);
    slBusOut.acc_tot = double(msgIn.acc_tot);
    slBusOut.course = double(msgIn.course);
    slBusOut.fpa = double(msgIn.fpa);
    slBusOut.roll = double(msgIn.roll);
    slBusOut.angle_attack = double(msgIn.angle_attack);
    slBusOut.sideslip = double(msgIn.sideslip);
    slBusOut.r = double(msgIn.r);
    slBusOut.q = double(msgIn.q);
    slBusOut.p = double(msgIn.p);
    slBusOut.r_dot = double(msgIn.r_dot);
    slBusOut.p_dot = double(msgIn.p_dot);
    slBusOut.q_dot = double(msgIn.q_dot);
end
