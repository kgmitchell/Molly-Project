function Calibrate

initialguess = [ -100 100
                  0   0
                  200 200];

IdentifiedValues = lsqnonlin(@CF , initialguess);

RealValues = [
    -107 95
      6   3
     204 197];

IdentifiedValues;
RealValues;