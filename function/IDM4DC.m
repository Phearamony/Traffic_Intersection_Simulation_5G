 function a4LC=IDM4LC(H,P)
            S=P.X-H.X-H.R0;
            dV=H.V-P.V;
            S0=1.5;
            if(S<0.95) S=0.95; end
            a=2; b=2.5;
            Th=0.9;
            Ss=S0+H.V*Th+H.V*dV/(2*sqrt(a*b));
            f=a*(1-(H.V/H.Vd)^4-(Ss/S)^2);
            a4LC=8*tanh(f/8.0);
 end