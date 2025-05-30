function f = fcn_contact(t,X,p)

params = p.params;
q = X(1:3);
p4 = fcn_p4(q,params);
if p4(2) <=