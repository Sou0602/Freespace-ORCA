function vo_poly = constructpoly(apex_cone,xint1,yint1,xint2,yint2)

px = apex_cone(1);
py = apex_cone(2);

vo_poly = polyshape([px,xint1,xint2,px],[py,yint1,yint2,py]);                                                                                                                                  




end