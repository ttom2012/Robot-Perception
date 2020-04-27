%this function is used for rotate the first vector into the second vecter
%and repersent in a quaterion form
function q = qUtoV(u, v)        %two vetor rotation to quaternions
nu = u/norm(u);
nv = v/norm(v);
%the angle between two vector
if (u*v' == -1) %180deg
    q = [0, [1,0,0]];%180deg
else
    half = (nu + nv)/norm(nu + nv);
    q = [nu*half',cross(nu, half)];
end
end