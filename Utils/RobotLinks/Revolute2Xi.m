function XI = Revolute2Xi(q,w,h)
   % q, w are 3x1 vector
   % h is the pitch of the screw
   if nargin < 3
      h = 0;
   end
   

    XI = [-cross(w,q) + h*w;
          w];
end