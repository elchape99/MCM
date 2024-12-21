function [iTj] = BuildTree()
% This function should build the tree of frames for the chosen manipulator.
% Inputs: 'None'
% Outputs: iTj (4, 4, numberOfLinks). The tree of frames.

% iTj is a 3-dimensional matlab matrix, suitable for defining tree of
% frames. iTj should represent the transformation matrix between the i-th and j-th
% frames. iTj(row,col,link_idx)

% ------------------------- 0T1 ------------------------------%
link_idx = 1;
iTj(1,1,link_idx) =  1  ; iTj(1,2,link_idx) =  0  ; iTj(1,3,link_idx) =  0  ; iTj(1,4,link_idx) =  0   ;
iTj(2,1,link_idx) =  0  ; iTj(2,2,link_idx) =  1  ; iTj(2,3,link_idx) =  0  ; iTj(2,4,link_idx) =  0   ;
iTj(3,1,link_idx) =  0  ; iTj(3,2,link_idx) =  0  ; iTj(3,3,link_idx) =  1  ; iTj(3,4,link_idx) =  3 * 0.0254 ;
iTj(4,1,link_idx) =  0  ; iTj(4,2,link_idx) =  0  ; iTj(4,3,link_idx) =  0  ; iTj(4,4,link_idx) =  1   ;

% ------------------------- 1T2 ------------------------------%
link_idx = 2;
iTj(1,1,link_idx) =   1  ; iTj(1,2,link_idx) =  0  ; iTj(1,3,link_idx) =  0  ; iTj(1,4,link_idx) =  0  ;
iTj(2,1,link_idx) =   0  ; iTj(2,2,link_idx) =  0  ; iTj(2,3,link_idx) =  1  ; iTj(2,4,link_idx) =  0  ;
iTj(3,1,link_idx) =   0  ; iTj(3,2,link_idx) =  -1  ; iTj(3,3,link_idx) =  0  ; iTj(3,4,link_idx) = (14.5 - 3 - 1.5) * 0.0254  ;
iTj(4,1,link_idx) =   0  ; iTj(4,2,link_idx) =  0  ; iTj(4,3,link_idx) =  0  ; iTj(4,4,link_idx) =  1  ;

% ------------------------- 2T3 ------------------------------%
link_idx = 3;
iTj(1,1,link_idx) =  0  ; iTj(1,2,link_idx) =  -1  ; iTj(1,3,link_idx) =  0  ; iTj(1,4,link_idx) = 7.8 * 0.0254;
iTj(2,1,link_idx) =  1  ; iTj(2,2,link_idx) =  0  ; iTj(2,3,link_idx) =  0  ; iTj(2,4,link_idx) =  0.75 * 0.0254  ;
iTj(3,1,link_idx) =  0  ; iTj(3,2,link_idx) =  0  ; iTj(3,3,link_idx) =  1  ; iTj(3,4,link_idx) =  -5 * 0.0254  ;
iTj(4,1,link_idx) =  0  ; iTj(4,2,link_idx) =  0  ; iTj(4,3,link_idx) =  0  ; iTj(4,4,link_idx) =  1  ;

% ------------------------- 3T4 ------------------------------%
link_idx = 4;
iTj(1,1,link_idx) =  0  ; iTj(1,2,link_idx) =  0  ; iTj(1,3,link_idx) =  1  ; iTj(1,4,link_idx) =  5 * 0.0254 ;
iTj(2,1,link_idx) =  0  ; iTj(2,2,link_idx) =  1  ; iTj(2,3,link_idx) =  0  ; iTj(2,4,link_idx) =  0;
iTj(3,1,link_idx) = -1  ; iTj(3,2,link_idx) =  0  ; iTj(3,3,link_idx) =  0  ; iTj(3,4,link_idx) =  0;
iTj(4,1,link_idx) =  0  ; iTj(4,2,link_idx) =  0  ; iTj(4,3,link_idx) =  0  ; iTj(4,4,link_idx) =  1    ;

% ------------------------- 4T5 ------------------------------%
link_idx = 5;
iTj(1,1,link_idx) =  0  ; iTj(1,2,link_idx) =  0  ; iTj(1,3,link_idx) =  -1 ; iTj(1,4,link_idx) =  0  ;
iTj(2,1,link_idx) = -1  ; iTj(2,2,link_idx) =  0  ; iTj(2,3,link_idx) =  0  ; iTj(2,4,link_idx) =  0  ;
iTj(3,1,link_idx) =  0  ; iTj(3,2,link_idx) =  1  ; iTj(3,3,link_idx) =  0  ; iTj(3,4,link_idx) =  3 * 0.0254  ;
iTj(4,1,link_idx) =  0  ; iTj(4,2,link_idx) =  0  ; iTj(4,3,link_idx) =  0  ; iTj(4,4,link_idx) =  1  ; 

% ------------------------- 5T6 ------------------------------%
link_idx = 6;
iTj(1,1,link_idx) =   0  ; iTj(1,2,link_idx) =  0  ; iTj(1,3,link_idx) =  1  ; iTj(1,4,link_idx) =  2.2 * 0.0254 ;
iTj(2,1,link_idx) =   0  ; iTj(2,2,link_idx) =  1  ; iTj(2,3,link_idx) =  0  ; iTj(2,4,link_idx) =  0  ;
iTj(3,1,link_idx) =  -1  ; iTj(3,2,link_idx) =  0  ; iTj(3,3,link_idx) =  0  ; iTj(3,4,link_idx) =  0  ;
iTj(4,1,link_idx) =   0  ; iTj(4,2,link_idx) =  0  ; iTj(4,3,link_idx) =  0  ; iTj(4,4,link_idx) =  1  ;


end
