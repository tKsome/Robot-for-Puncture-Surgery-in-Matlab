ToDeg = 180/pi;%Trance the value of angle
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';
%                                    theta              d          a              alpha
Link= struct('name',  'Body', 'th',    0,         'dz', 0,    'dx', 0,     'alf',  0*ToRad,'az',UZ);    
Link(1)= struct('name','Base' , 'th',  0*ToRad,   'dz', 0,    'dx', 0,     'alf',  0*ToRad,'az',UZ);   

Link(2) = struct('name','J1' , 'th',   0*ToRad,   'dz', 89.459, 'dx',0,      'alf',  90*ToRad,'az',UZ);       
Link(3) = struct('name','J2' , 'th',   0*ToRad,   'dz', 0,      'dx', -425,  'alf',  0*ToRad,'az',UZ);   
Link(4) = struct('name','J3' , 'th',   0*ToRad,   'dz', 0,      'dx',-392.25,'alf',  0*ToRad,'az',UZ);         
Link(5) = struct('name','J4' , 'th',   0*ToRad,   'dz', 109.15, 'dx',0,      'alf',  90*ToRad,'az',UZ);         
Link(6) = struct('name','J5' , 'th',   0*ToRad,   'dz', 94.65,  'dx',0,      'alf',  -90*ToRad,'az',UZ);          
Link(7) = struct('name','J6' , 'th',   0*ToRad,   'dz', 0,      'dx',0,      'alf',  0*ToRad,'az',UZ);
Link(8) = struct('name','J7' , 'th',   0*ToRad,   'dz', 82.3,   'dx',0,      'alf',  0,'az',UZ);  
%主体：2-8关节