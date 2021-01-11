function [E,W,C,data] = modal_vectors(A,varargin)
% Sort eigenvalues end eigenvectors. Starting with oscilatory modes up to
% an eigenfrequency of w_osc_max rad/s. Modes are sorted according to
% damping ratio.

[~,E,W] = eig(A); 
% W'*A*inv(W') = E;  
E = diag(E);
n = length(E);

%  sort eigenvalues slower than w_osc_max
if nargin == 1
    w_osc_max = 8;
else
    w_osc_max = varargin{1};
end

idx1 = find(abs(E)<w_osc_max & abs(E)>1e-6);
E1 = E(idx1);
W1 = W(:,idx1); %eigenvectors
damping = -real(E1)./abs(E1);
[damping1,idx_sort] = sort(damping);
E1 = E1(idx_sort);
W1 = W1(:,idx_sort);

% sort remainding eigenvalues
idx2 = 1:n;
idx2(idx1) = [];
E2 = E(idx2);
W2 = W(:,idx2);
damping = -real(E2)./abs(E2);
[damping2,idx_sort] = sort(damping);
E2 = E2(idx_sort);
W2 = W2(:,idx_sort);
E = [E1;E2];
W = [W1,W2];

C = zeros(n,n);

i = 1;
while i <= n
    if imag(E(i))==0
        C(:,i) = real(W(:,i));
        i = i+1;
    else
        C(:,[0,1]+i) = [real(W(:,i)),imag(W(:,i+1))];
        i = i+2;
    end
end

C = C'; % modes = C*states

data.freq = abs(E);
data.damping = [damping1;damping2];
end

