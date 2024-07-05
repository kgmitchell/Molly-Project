% Midterm Exam
% Kyle Mitchell

% Pods with nominal values (from Table 1 in Paper 3):
% pod#nom = [Si_x, Si_y, Si_z, Ui_x, Ui_y, Ui_z, Li_0]; % dim in mm
pod1nom = [92.1597,     84.4488,  0,   305.4001,   111.1565,  0,  604.8652];
pod2nom = [27.055,     122.0370,  0,   -56.4357,   320.0625,  0,  604.8652];
pod3nom = [-119.2146,   37.5882,  0,  -248.9644,   208.9060,  0,  604.8652];
pod4nom = [-119.2146,  -37.5882,  0,  -248.9644,  -208.9060,  0,  604.8652];
pod5nom = [27.055,    -122.0370,  0,   -56.4357,  -320.0625,  0,  604.8652];
pod6nom = [92.1597,    -84.4488,  0,   305.4001,  -111.1565,  0,  604.8652];
podsnom = [pod1nom; pod2nom; pod3nom; pod4nom; pod5nom; pod6nom];

% Pods with simulated real values (from Table 2 in Paper 3):
% pod#nom = [Si_x, Si_y, Si_z, Ui_x, Ui_y, Ui_z, Li_0]; % dim in mm
pod1simr = [96.6610,     81.7602,   1.0684,   305.2599,   115.0695,   2.6210,  604.4299];
pod2simr = [22.2476,    125.2511,  -0.5530,   -55.2814,   322.9819,   4.2181,  607.2473];
pod3simr = [-122.4519,   36.6453,   4.3547,  -244.7954,   208.0087,   3.9365,  600.4441];
pod4simr = [-120.6859,  -34.4565,  -4.9014,  -252.5755,  -211.8783,  -3.0128,  605.9031];
pod5simr = [24.7769,   -125.0489,  -4.8473,   -53.9678,  -320.6115,   4.3181,  604.5251];
pod6simr = [91.3462,    -80.9866,   0.2515,   302.4266,  -109.4351,   3.3812,  600.0616];
podssimr = [pod1simr; pod2simr; pod3simr; pod4simr; pod5simr; pod6simr];

% Error between pods' nominal and simulated real values
delta_rho = [(pod1simr - pod1nom) (pod2simr - pod2nom) (pod3simr - pod3nom) (pod4simr - pod4nom) (pod5simr - pod5nom) (pod6simr - pod6nom)]';

% Condider delta(delta l) to equal 0
delta_delta_l = zeros(6,1);