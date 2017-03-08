%----------------------- Define statusCallback -----------------------%
function statusCallback_2(src,msg)
    global volt_2;
    volt_2 = msg.Voltage;

end
        
%---------------------------- END ------------------------------------%