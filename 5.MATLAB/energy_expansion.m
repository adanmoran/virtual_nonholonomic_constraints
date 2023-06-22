%% Energy Expansion
% This script runs Xingbo's constraint and displays the level sets of the
% total mechanical energy along the constraint itself. The goal is to do
% determine if the total mechanical energy is increasing along orbits of a
% VNHC.
% This script displays the level sets of E alongside the VNHC orbits for
% certain starting points (0,pk). It also shows the curve E_dot = 0 in the
% VNHC region, and the sectors where E_dot > 0 and E_dot < 0.
% We will try to use this to find conditions where the flow along E_dot > 0
% is longer than that of E_dot < 0, thereby showing that we have increased
% in energy.

% Run Xingbo's stuff if it hasn't been run already.
if ~exist('XINGBO_NVHC_V2','var')
    xingbo_nvhc_v2
end

fprintf('==================\n');
fprintf(' Energy Expansion \n');
fprintf('==================\n');
fprintf('Creating E(q,f,p,g)\n');
% Compute q2 on the nonholonomic region as a function of (q1,p1) which is
% usable on the boundaries q = 0 and p = 0
constraintFun = sym2fun(constraint);
q_hfun = @(x,y)0;
q2Safe = @(q,p)safeNHDynamics(q,p,q_hfun, constraintFun);

% Compute p2 on the nonholonomic region as a function of (q1,p1) which is
% usble on the boundaries q = 0 and p = 0
p_hfun = matlabFunction(p2Z,'vars',[q1, p1]);
p_nhfun = sym2fun(p2NZ);
p2Safe = @(q,p)safeNHDynamics(q,p,p_hfun,p_nhfun);

% Get the constrained energy for the nonholonomic constraint
Efun = matlabFunction(E,'vars',[q1 q2 p1 p2]);
E_nhFun = @(q,p) Efun(q,q2Safe(q,p),p,p2Safe(q,p));

% Open an image for level sets of E vs VNHC
fprintf('Finding Orbits of the VNHC\n');
E_vs_VNHC = figure;
grid on;
hold on;
xlabel('q');
ylabel('p');
title('E vs. orbits of VNHC');
% Get orbits of the VNHC along certain starting states
pks = [10 11 12 13];
solutions = cell(size(pks));
for i = 1:length(pks)
    pk = pks(i);
    sol = ode45(@(t,x)nhFun(x(1),x(2)),[0,2],[0;pk],...
    odeset('Events',@pZeroEvents,'RelTol',10^-13,'AbsTol',10^-13));
    plot(sol.y(1,:),sol.y(2,:),'--');
    solutions{i} = sol;
end

% Start a new figure for comparing E with the VNHC
fprintf('Finding Contour of E_nh\n');
contourRegion = [0 2.8 0 13];
E_nhContour = fcontour(E_nhFun, contourRegion);
E_nhContour.LevelList = E_nhFun(zeros(size(pks')), pks');
E_nhContour.LineStyle = '-';
% Now also plot the level sets of H extended onto the VNHC plane to see how
% it compares with E_nh (i.e. show H for the VHC alongside E for the VNHC).
% Rather than using hamiltonian_vhc, we'll use the fact that H = 2.5 E_h
% for this mechanical system.
% (it's faster than doing it with hamiltonian_vhc).
Hfun = matlabFunction(2.5*E_h,'vars',[q1 p1]);
fc = fcontour(Hfun,[0 2 0 11]);
fc.LevelList = Hfun(zeros(size(pks')), pks');
fc.LineStyle = ':';

legend({'','','','Orbits','E_nh (- lines)','H (: lines)'});
hold off

% Plot E_nh and H, along with their difference
fprintf('Plotting meshes of E_nh and E_h on the VNHC orbits\n');
[QGrid, PGrid] = ndgrid(linspace(0.001,2.8,500),linspace(0.001,13,500));
E_nhZ = E_nhFun(QGrid,PGrid);
E_hZ = Hfun(QGrid,PGrid)./2.5;
E_mesh_fig = figure;
mesh(QGrid,PGrid,E_nhZ);
xlabel('q');
ylabel('p');
zlabel('E_{nh}(q,p)');
title('Mesh of Nonholonomic E(q,f,p,g)');

Eh_mesh_fig = figure;
mesh(QGrid,PGrid,E_hZ);
xlabel('q');
ylabel('p');
zlabel('E_h(q,p)');
title('Mesh of Holonomic E(q,0,p,0)');

Enh_minus_Eh_fig = figure;
mesh(QGrid,PGrid, E_nhZ - E_hZ);
xlabel('q');
ylabel('p');
zlabel('E_{nh} - E_h');
title('Difference between E_{nh} and E_h on the VNHC domain');

% Find E_dot, the derivative of E along the nonholonomic dynamics
% i.e. E_dot = d/dt E(phi(t,(0,pk)) = L_nh E_nh = dE * nhfun
fprintf('Finding E_dot\n');
E_nh = subs(E,[q2 p2], [constraint, p2NZ]);
dE_nh = [diff(E_nh,q1) diff(E_nh,p1)];
E_dot = dE_nh * dynNH;

% Convert E_dot into a function
E_dotFun = matlabFunction(E_dot,'vars',[q1 p1]);

% Display E_dot
fprintf('Plotting E_dot\n');
E_dot_mesh_fig = figure;
EdZ = E_dotFun(QGrid,PGrid);
mesh(QGrid,PGrid,EdZ);
xlabel('q');
ylabel('p');
zlabel('E_{dot}');
title('Derivative of E along orbits of the VNHC');

% Now find the level set E_dot = 0 only inside the nonholonomic region (not
% on the boundary. Also find the regions where E_dot is positive and
% negative.
fprintf('Plotting the contour of E_dot\n');
EdZ(EdZ > 0) = 1;
EdZ(EdZ < 0) = -1;
% Display these level sets against the VNHC orbits
EdotFig = figure;
hold on;
[EdC,EdH] = contourf(QGrid,PGrid,EdZ,[-1 0 1]);
colorbar('Ticks',[-1,0,1],...
         'TickLabels',{'< 0','0','> 0'});
% Display the orbits of the VNHC on top
for i = 1:length(solutions)
    sol = solutions{i};
    plot(sol.y(1,:),sol.y(2,:),'--');
end
xlabel('q');
ylabel('p');
title('Derivative of E vs. Orbits of the VNHC');
legend({'E_{dot}','Orbits'});