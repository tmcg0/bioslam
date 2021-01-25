% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function newfilename = filenameAppender(filename,doRecursionForNewName)
% takes in a filename and appends a unique bit to the end of it.
% useful when you need to create a unique filename.
[filepath,name,ext]=fileparts(filename);
% () if it already ends in an underscore and number, tick up that number. otherwise add _1 to the end.
r=regexp(name,'(_\d{1,2}$)','tokens'); % added an exclamation point to end to make it capture the exact number
if ~isempty(r) % you have an underscore and digit at the end. tick it up.
    digits=str2double(r{1}{1}(2:end));
    newdigits=digits+1;
    newfilename=fullfile(filepath,strcat(name(1: (end-(length(r{1}{1}))) ),'_',num2str(newdigits),ext) );
else % just append _1 to end
    newfilename=fullfile(filepath,strcat(name,'_1',ext));
end
% () make sure the new filename doesn't exist
% recurse until you have a unique filename
if doRecursionForNewName
    while exist(newfilename,'file')==2
        newfilename=filenameAppender(newfilename,0);
    end
end
end

