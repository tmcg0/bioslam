% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

% a function to inspect the Ax=b system of your problem

function jacobianInspector(graph,vals)
startTic=tic;
% check inputs
assert(isa(graph,'gtsam.NonlinearFactorGraph') & isa(vals,'gtsam.Values'));

% () get the corresponding dense matrix
ord=graph.orderingCOLAMD();
gfg=graph.linearize(vals); % gtsam.GaussianFactorGraph (linear)
% () get the sparse matrix A and right hand side b
IJS=gfg.sparseJacobian_();
%   doc of sparseJacobian_(): Matrix version of sparseJacobian(),
%   generates a 3*m matrix with [i,j,s] entries such that S(i(k),j(k)) = s(k), which can be given to MATLAB's sparse.
Ab=sparse(IJS(1,:),IJS(2,:),IJS(3,:));
A=Ab(:,1:end-1); % last column is b
b=full(Ab(:,end)); % full() converts a sparse matrix to full storage
figure; spy(A); title('non-zero entries in measurement Jacobian');
fprintf('inspected sparse system in %.2f seconds\n',toc(startTic));
% remember, elementary, x=inv(A)*b; % equiv to x=A\b;
end