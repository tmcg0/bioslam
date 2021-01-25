% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function printLowerBodyProblemResultsToFilesDefault(lbpe,dataoutDir,label)
startTic=tic;
fprintf('\twriting all data to files for lower body pose problem...');
% write the IMU states
lbpe.printSacrumImuYprToFile(fullfile(dataoutDir,strcat(label,'_Sacrum','.ypr')));
lbpe.printSacrumImuQuatToFile(fullfile(dataoutDir,strcat(label,'_Sacrum','.quat')));
lbpe.printSacrumImuAccelBiasToFile(fullfile(dataoutDir,strcat(label,'_Sacrum','.accelbias')));
lbpe.printSacrumImuGyroBiasToFile(fullfile(dataoutDir,strcat(label,'_Sacrum','.gyrobias')));
lbpe.printSacrumImuDcmToFile(fullfile(dataoutDir,strcat(label,'_Sacrum','.dcm')));
lbpe.printSacrumImuPosToFile(fullfile(dataoutDir,strcat(label,'_Sacrum','.pos')));
lbpe.printSacrumImuVelToFile(fullfile(dataoutDir,strcat(label,'_Sacrum','.vel')));
% right side of body
lbpe.printRThighImuYprToFile(fullfile(dataoutDir,strcat(label,'_RThigh','.ypr')));
lbpe.printRThighImuQuatToFile(fullfile(dataoutDir,strcat(label,'_RThigh','.quat')));
lbpe.printRThighImuAccelBiasToFile(fullfile(dataoutDir,strcat(label,'_RThigh','.accelbias')));
lbpe.printRThighImuGyroBiasToFile(fullfile(dataoutDir,strcat(label,'_RThigh','.gyrobias')));
lbpe.printRThighImuDcmToFile(fullfile(dataoutDir,strcat(label,'_RThigh','.dcm')));
lbpe.printRThighImuPosToFile(fullfile(dataoutDir,strcat(label,'_RThigh','.pos')));
lbpe.printRThighImuVelToFile(fullfile(dataoutDir,strcat(label,'_RThigh','.vel')));
lbpe.printRShankImuYprToFile(fullfile(dataoutDir,strcat(label,'_RShank','.ypr')));
lbpe.printRShankImuQuatToFile(fullfile(dataoutDir,strcat(label,'_RShank','.quat')));
lbpe.printRShankImuAccelBiasToFile(fullfile(dataoutDir,strcat(label,'_RShank','.accelbias')));
lbpe.printRShankImuGyroBiasToFile(fullfile(dataoutDir,strcat(label,'_RShank','.gyrobias')));
lbpe.printRShankImuDcmToFile(fullfile(dataoutDir,strcat(label,'_RShank','.dcm')));
lbpe.printRShankImuPosToFile(fullfile(dataoutDir,strcat(label,'_RShank','.pos')));
lbpe.printRShankImuVelToFile(fullfile(dataoutDir,strcat(label,'_RShank','.vel')));
lbpe.printRFootImuYprToFile(fullfile(dataoutDir,strcat(label,'_RFoot','.ypr')));
lbpe.printRFootImuQuatToFile(fullfile(dataoutDir,strcat(label,'_RFoot','.quat')));
lbpe.printRFootImuAccelBiasToFile(fullfile(dataoutDir,strcat(label,'_RFoot','.accelbias')));
lbpe.printRFootImuGyroBiasToFile(fullfile(dataoutDir,strcat(label,'_RFoot','.gyrobias')));
lbpe.printRFootImuDcmToFile(fullfile(dataoutDir,strcat(label,'_RFoot','.dcm')));
lbpe.printRFootImuPosToFile(fullfile(dataoutDir,strcat(label,'_RFoot','.pos')));
lbpe.printRFootImuVelToFile(fullfile(dataoutDir,strcat(label,'_RFoot','.vel')));
lbpe.printRKneeAnglesToFile(fullfile(dataoutDir,strcat(label,'_RKneeAngles','.csv')));
lbpe.printRHipAnglesToFile(fullfile(dataoutDir,strcat(label,'_RHipAngles','.csv')));
% left side of body
lbpe.printLThighImuYprToFile(fullfile(dataoutDir,strcat(label,'_LThigh','.ypr')));
lbpe.printLThighImuQuatToFile(fullfile(dataoutDir,strcat(label,'_LThigh','.quat')));
lbpe.printLThighImuAccelBiasToFile(fullfile(dataoutDir,strcat(label,'_LThigh','.accelbias')));
lbpe.printLThighImuGyroBiasToFile(fullfile(dataoutDir,strcat(label,'_LThigh','.gyrobias')));
lbpe.printLThighImuDcmToFile(fullfile(dataoutDir,strcat(label,'_LThigh','.dcm')));
lbpe.printLThighImuPosToFile(fullfile(dataoutDir,strcat(label,'_LThigh','.pos')));
lbpe.printLThighImuVelToFile(fullfile(dataoutDir,strcat(label,'_LThigh','.vel')));
lbpe.printLShankImuYprToFile(fullfile(dataoutDir,strcat(label,'_LShank','.ypr')));
lbpe.printLShankImuQuatToFile(fullfile(dataoutDir,strcat(label,'_LShank','.quat')));
lbpe.printLShankImuAccelBiasToFile(fullfile(dataoutDir,strcat(label,'_LShank','.accelbias')));
lbpe.printLShankImuGyroBiasToFile(fullfile(dataoutDir,strcat(label,'_LShank','.gyrobias')));
lbpe.printLShankImuDcmToFile(fullfile(dataoutDir,strcat(label,'_LShank','.dcm')));
lbpe.printLShankImuPosToFile(fullfile(dataoutDir,strcat(label,'_LShank','.pos')));
lbpe.printLShankImuVelToFile(fullfile(dataoutDir,strcat(label,'_LShank','.vel')));
lbpe.printLFootImuYprToFile(fullfile(dataoutDir,strcat(label,'_LFoot','.ypr')));
lbpe.printLFootImuQuatToFile(fullfile(dataoutDir,strcat(label,'_LFoot','.quat')));
lbpe.printLFootImuAccelBiasToFile(fullfile(dataoutDir,strcat(label,'_LFoot','.accelbias')));
lbpe.printLFootImuGyroBiasToFile(fullfile(dataoutDir,strcat(label,'_LFoot','.gyrobias')));
lbpe.printLFootImuDcmToFile(fullfile(dataoutDir,strcat(label,'_LFoot','.dcm')));
lbpe.printLFootImuPosToFile(fullfile(dataoutDir,strcat(label,'_LFoot','.pos')));
lbpe.printLFootImuVelToFile(fullfile(dataoutDir,strcat(label,'_LFoot','.vel')));
lbpe.printLKneeAnglesToFile(fullfile(dataoutDir,strcat(label,'_LKneeAngles','.csv')));
lbpe.printLHipAnglesToFile(fullfile(dataoutDir,strcat(label,'_LHipAngles','.csv')));
fprintf(' done. (%.2f seconds)\n',toc(startTic));
end