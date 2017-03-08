%----------------------- Define statusCallback -----------------------%
function statusCallback_1(src,msg)
    global volt_1;
    volt_1 = msg.Voltage;

end
        
%---------------------------- END ------------------------------------%