function [v,e,w] = modal_analysis(A)
[V,E] = eig(A); % V is matrix of right (column) eigenvectors V = [v1, v2,...,v_n]
E = diag(E);
n = length(E);
%  poorly damped swing modes
idx1 = find(abs(E)<40&abs(E)>1e-6);
E1 = E(idx1);
V1 = V(:,idx1); %eigenvectors
idx2 = 1:n;
idx2(idx1) = [];
E2 = E(idx2);
V2 = V(:,idx2);
damping = -real(E1)./abs(E1);
[damping,idx_sort] = sort(damping);
E1 = E1(idx_sort);
V1 = V1(:,idx_sort);
damping = -real(E2)./abs(E2);
[damping,idx_sort] = sort(damping);
E2 = E2(idx_sort);
V2 = V2(:,idx_sort);
E = [E1;E2];
V = [V1,V2];

freq = abs(E); %
damping = -real(E)./abs(E);
e = E(1); 
disp(['mode=', num2str(e),...
      ' freq=', num2str(abs(e)),...
      ' damping=' num2str(-real(e)./abs(e))]); 
v= V(:,1);  
W = inv(V); % W is matrix of left (row) eigenvectors W = [w1; w2;...;v_n]
w = W(1,:);
% figure()
% ang= angle(W(:,idx_g(1)));
% W = exp(-1j*(ang)).*W; % align all eigenvectors so that omega_1 is 0deg
% compass(W(1,idx_g(1)),'b'); hold on; 
% compass(W(1,idx_g(2)),'b--') 
% compass(W(1,idx_g(3)),'r')
% compass(W(1,idx_g(4)),'r--')
end

