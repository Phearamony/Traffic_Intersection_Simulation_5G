u = udpport("datagram", "IPV4", "LocalPort", 6000);
fprintf("Waiting for packet on port 6000...\n");

while true
    if u.NumDatagramsAvailable > 0
        data = read(u, 1, "uint8");
        msg  = char(data.Data');
        fprintf("RECEIVED: %s\n", msg);
    end
    pause(0.01);
end