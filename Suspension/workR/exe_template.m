%--------------------------------------------------------------------------
%   Universite catholique de Louvain
%   CEREM : Centre for research in mechatronics
%   http://www.robotran.be  
%   Contact : robotran@prm.ucl.ac.be
%   Version : ROBOTRAN $Version$
%
%   MBsysLab main script template:
%      - featuring default options
%      - to be adapted by the user
%
%   Project : Suspension
%   Author : Team Robotran
%   Date : $Date$ 
%--------------------------------------------------------------------------

%% 1. Initialization and Project Loading [mbs_load]
%--------------------------------------------------------------------------
close all; clear variables; clc;                                            % Cleaning of the Matlab workspace
global MBS_user;                                                            % Declaration of the global user structure
MBS_user.process = '';                                                      % Initialisation of the user field "process"

% Project loading
prjname = 'Suspension';
[mbs_data, mbs_info] = mbs_load(prjname,'default');                         % Option 'default': automatic loading of "$project_name$.mbs" 
mbs_data_ini = mbs_data;                                                    % Backup of the initial multibody data structure
                                                                            % Have a look at the content of the mbs_data structure on www.robotran.be

%% 2. Coordinate partitioning [mbs_exe_part]                                % For constrained MBS only
%--------------------------------------------------------------------------
MBS_user.process = 'part';

mbs_data = mbs_set_qu(mbs_data,[1 2]);                                      % Set variables [qu_id] as independent 
mbs_data = mbs_set_qv(mbs_data,[3 4 6]);                                      % Set variables [qv_id] as dependent
mbs_data = mbs_set_qdriven(mbs_data,[5]);                                      % Set variables [qc_id] as driven

opt.part = {'rowperm','yes','threshold',1e-9,'verbose','yes'};
% other options : 'visualize', 'clearmbsglobal'                             % Help about options on www.robotran.be

[mbs_part,mbs_data] = mbs_exe_part(mbs_data,opt.part);                      % Coordinate partitioning process

% Coordinate partitioning results
disp('Coordinate partitioning results');
disp(['Sorted independent variables = ', mat2str(mbs_part.ind_u)]);
disp(['Permutated dependent variables = ', mat2str(mbs_part.ind_v)]);
disp(['Permutated independent constraints = ', mat2str(mbs_part.hu)]);
disp(['Redundant constraints = ', mat2str(mbs_part.hv)]);


%% 5. Direct dynamics [mbs_exe_dirdyn]
%--------------------------------------------------------------------------
t = 0:0.01:10;

MBS_user.process = 'dirdyn';
mbs_data = mbs_set_qu(mbs_data,mbs_data_ini.qu);                            % Retrieving of the initial set of independent variables

opt.dirdyn = {'time',t,'motion','simulation',...
    'odemethod','ode45','save2file','yes','framerate',1000,...
    'renamefile','no','verbose','yes'};
% other options : 'visualize', 'save2file', 'depinteg', 'dtmax', 'dtinit',
%                 'reltol', 'abstol', 'clearmbsglobal'                      % Help about options on www.robotran.be

[mbs_dirdyn,mbs_data] = mbs_exe_dirdyn(mbs_data,opt.dirdyn);                % Direct dynamics process (time simulation)


%% * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
% Sauvegrade des resultats pour animation
% reechantillonage de la solution: au cas ou le pas de temps utilise pour
% ode45 ne correspond pas au pas de temps necessaire pour l'animation

% vecteur des instants auquels on calcule une image

tAnim=t';
% reechantillonage de la solution
qAnim = interp1(t, mbs_dirdyn.q, tAnim);
% sauvegarde du fichier d'animation
tqAnim=[tAnim qAnim];
save([mbs_data.prjpath '/' prjname '/animationR/qAnim.anim'], 'tqAnim',  '-ASCII');



% Graphical Results
figure;
subplot(2,1,1)
plot(mbs_dirdyn.tsim,mbs_dirdyn.q(:,1));                                    % Joint motion time history : joint n° 1 motion (example)
ylim([0.66 0.72]);
grid on;
subplot(2,1,2);
plot(mbs_dirdyn.tsim,mbs_dirdyn.q(:,2)*180/pi);                                    % Joint motion time history : joint n° 1 motion (example)
ylim([-35 -10]);
grid on;

%% 8. Closing operations (optional)
%--------------------------------------------------------------------------
mbs_rm_allprjpath;                                                          % Cleaning of the Matlab project paths
mbs_del_glob('MBS_user','MBS_info','MBS_data');                             % Cleaning of the global MBS variables
clc;                                                                        % Cleaning of the Matlab command window
