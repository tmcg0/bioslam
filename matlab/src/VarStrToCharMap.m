% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

classdef VarStrToCharMap < handle
    methods
        %         function obj=VarStrToCharMap(varargin)
        %
        %         end % constructor
    end
    methods (Static)
        function varargout=insert(varStr)
            assert(isa(varStr,'char'),'input variable string as a char (single quotes!--not a string with double quotes)');
            global m_map
            if ~isa(m_map,'cell')
                m_map=cell(0,2);
            end
            mapSize=VarStrToCharMap.getSize();
            newKey=VarStrToCharMap.getCharByInt(mapSize+1);
            m_map(mapSize+1,1:2)={newKey,varStr};
            if nargout==1 % return the new character you just assigned, for convenience
                varargout{1}=VarStrToCharMap.getChar(varStr);
            elseif nargout>1
                error('only one output allowed');
            end
        end
        function print()
            global m_map
            if isa(m_map,'cell')
                disp(m_map);
            else
                fprintf('m_map not existent\n');
            end
        end
        function out=getChar(varStr)
            % get char associated with var str
            global m_map
            out='';
            if isa(m_map,'cell')
                % search in second column.
                found=strcmp(m_map(:,2),varStr);
                if sum(found)==1 % good. you found unique answer.
                    out=m_map{find(found),1};
                else
                    disp(found)
                    error('didn''t find this')
                end
            end
        end
        function out=getSize()
            global m_map
            if isa(m_map,'cell')
                out=size(m_map,1);
            else
                out=0;
            end
        end
        function clear()
            global m_map
            if isa(m_map,'cell')
                m_map=[];
            end
        end % clear()
        function out=getCharByInt(i)
            alphabets = 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ';
            out=alphabets(i);
        end
    end
end % classdef