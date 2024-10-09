classdef gCodeReader < handle
    % Converts gCode into a robot path
    %   Detailed explanation goes here

    properties
        
    end

    methods
        function self = gCodeReader()
            clf
            clc
            self.openGcodeFile();
        end
    end

    methods(Static)

        function openGcodeFile(filename)

            fid = fopen(filename , 'r') % opening file in 'r' mode (reading mode)
            fclose(fid);

           
        end

    end
end