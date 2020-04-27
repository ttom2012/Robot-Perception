%this function can also be seen as a cross product of the quaterion
%however this is different from the other since this one adjust the
%direction of the rotation the rotation has to be with in 90deg
%you can also understanding this function as the second vector as a
%rotation acting on the first vector
function ab = quaternProd(a, b)
    ab(1) = a(1)*b(1)-a(2)*b(2)-a(3)*b(3)-a(4)*b(4);
    ab(2) = a(1)*b(2)+a(2)*b(1)+a(3)*b(4)-a(4)*b(3);
    ab(3) = a(1)*b(3)-a(2)*b(4)+a(3)*b(1)+a(4)*b(2);
    ab(4) = a(1)*b(4)+a(2)*b(3)-a(3)*b(2)+a(4)*b(1);
    if ab(1)<0
        ab=-ab;
    end
end
