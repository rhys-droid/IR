classdef TreeBot < RobotBaseClass
    %% TreeBot
    %This class is based from an imaginary robot


    properties(Access =public)   
        plyFileNameStem = 'TreeBot';

        %> defaultRealQ 
        defaultRealQ  = [0, -pi/4, pi/6, 0,0,0, 0, 0];
    end

    methods (Access = public) 
%% Constructor 
function self =TreeBot(baseTr)

    	self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T; %* baseTr * trotx(pi/2) * troty(pi/2);

            self.PlotAndColourRobot();  
    
        end

%% CreateModel
        function CreateModel(self) 

            L1 = Link('d',0.3,'a',0,'alpha',-pi/2,'qlim',[-2*pi 2*pi]);
            L2 = Link('d', 0, 'a', 0.4, 'alpha', 0, 'qlim', [-pi/2, 0]);
            L3 = Link('d', 0, 'a', 0.4, 'alpha', 0, 'qlim', [0, pi/2]);
            L4 = Link('d', 0, 'a', 0, 'alpha', -pi/2, 'qlim', [-pi, pi]);
            L5 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-pi/4, 5*pi/4]);
            L6 = Link('d', 0.2, 'a', 0, 'alpha', -pi/2, 'qlim', [-pi, pi]);
            L7 = Link('d', 0.2, 'a', 0, 'alpha', pi/2, 'qlim', [-pi, pi]);
            L8 = Link('d', 0.2, 'a', 0, 'alpha', 0, 'qlim', [-2*pi, 2*pi]);

            
            self.model = SerialLink([L1, L2, L3, L4, L5, L6, L7, L8], 'name',self.name);

        end   
    end

    methods(Static)
%% RealQToModelQ
        % Convert the real Q to the model Q
        function modelQ = RealQToModelQ(realQ)
            modelQ = realQ;
            modelQ(3) = DobotMagician.ComputeModelQ3GivenRealQ2and3( realQ(2), realQ(3) );
            modelQ(4) = pi - realQ(2) - modelQ(3);    
        end
        
%% ModelQ3GivenRealQ2and3
        % Convert the real Q2 & Q3 into the model Q3
        function modelQ3 = ComputeModelQ3GivenRealQ2and3(realQ2,realQ3)
            modelQ3 = pi/2 - realQ2 + realQ3;
        end
        
%% ModelQToRealQ
        % Convert the model Q to the real Q
        function realQ = ModelQToRealQ( modelQ )
            realQ = modelQ;
            realQ(3) = DobotMagician.ComputeRealQ3GivenModelQ2and3( modelQ(2), modelQ(3) );
        end
        
%% RealQ3GivenModelQ2and3
        % Convert the model Q2 & Q3 into the real Q3
        function realQ3 = ComputeRealQ3GivenModelQ2and3( modelQ2, modelQ3 )
            realQ3 = modelQ3 - pi/2 + modelQ2;
        end
    end
end