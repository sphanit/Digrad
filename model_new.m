classdef model_new < handle
    
    properties
        robot
        action_bound = [-50,50];
        action_dim = 0;
        state_dim = 0;
        theta = zeros(1,25);
        dq = zeros(1,25);
        A = 0;
        S = 0;
        ret = [];
        done1 = 0;
	done2 = 0;
        CoM = [0, 0, 0];
        goal = zeros(3,1);
        goal2 = zeros(3,1);
        cf = 0;
        ang_limit = limits();
        tau = zeros(1,25);
        h_prev = zeros(1,6);
        dt = 0.08;
        grab_control = 0;
        grab_control2 = 0;
        theta_ini = zeros(1,25);
        
        %Defaults
        scale = 1000;
        mode ='hard';
        chain='r'; p_or_f=1;
        in_state = 0;
        precision=20;
        prec_bound=40;
        epsilon = 1e-6;
        stable = 0;
        
        %predefine things
        r_arm,l_arm,l_leg,adj,spine
        info,enf,enf2,idx,pos,M,Lc,Pc,init_th
        ho,zmp,dist1,dist2,prev_CoM=zeros(3,1);
        
    end
    methods
        
        function self =  model_new(mode,chain,p_or_f,precision)    %mode = 'easy'/'hard'
            %rng(1);
            
            %Initializations of the global variables and importing data
            self.robot = importrobot('./urdf/main_stl_final.urdf');
            self.robot.DataFormat = 'row';
            self.robot.Gravity = [-9.81 0 0];
            
            %Adding the new variables
            if nargin >= 1
                self.mode = mode;
            end
            if nargin >= 2
                self.chain = chain;
            end
            if nargin >= 3
                self.p_or_f = p_or_f;
            end
            if nargin >= 4
                self.precision=precision;
            end
            
            %Names and Ids for leg base
            self.l_arm = 17;
            self.r_arm = 22;
            self.l_leg = 29;
            self.spine = 12;
            
            %Configuration
            self.theta = zeros(1,25);
            self.theta(13) =  0.05;
            self.theta(17) = -0.05;
            self.theta(3)  = -0.1745;
            self.theta(2)  =  0.1745;
            self.theta(23) =  0.1745;
            self.theta(24) = -0.1745;
            
            %Angle Limits
            self.limit_ang();
            
            %Forward Kinematics
            self.pos = forwardKinematics(self.robot,self.theta);
            [self.CoM, self.M] = centerOfMass(self.robot,self.theta);
            self.prev_CoM = self.CoM;
            
            %ZMP Calculation
            [self.tau,self.ho] = inverseDynamics(self.robot,self.theta,self.dq);
            [self.zmp, self.h_prev] = cal_ZMP(self.M, self.CoM, self.ho, self.h_prev);
            
            %ZMP Check
            stability_check(self);
            
            %Collision check
            col_body = collision_data(self.pos);
            self.cf = collision_check(col_body);
            
            if(self.p_or_f == 0)
                if(strcmp(self.chain,'r') || strcmp(self.chain,'l'))
                    self.action_dim = 4;
                    self.state_dim = 3;
                    self.S = zeros(1,13);
                    self.A = zeros(1,4);
                    if(strcmp(self.chain,'r'))
                        self.enf = self.r_arm;
                        self.info.name = 'right_arm';
                        self.info.type = 'partial';
                        self.info.n_joints = 4;
                        self.info.enf = self.r_arm;
                        self.idx = [16,17,18,19];
                    else
                        self.enf = self.l_arm;
                        self.info.name = 'left_arm';
                        self.info.type = 'partial';
                        self.info.n_joints = 4;
                        self.info.enf = self.l_arm;
                        self.idx = [12,13,14,15];
                        
                    end
                    
                elseif(strcmp(self.chain,'s'))
                    self.action_dim = 5;
                    self.state_dim = 14;
                    self.S = zeros(1,14);
                    self.A = zeros(1,5);
                    self.enf = self.spine;
                    self.idx = [7,8,9,10,11];
                    self.info.name = 'spine';
                    self.info.type = 'partial';
                    self.info.n_joints = 5;
                    self.info.enf = self.spine;
                    
                else
                    fprintf('Error: Enter a chain..!!!');
                end
            else
                if(~isempty(self.chain))
                    self.action_dim = 13;
                    self.state_dim = 30;
                    self.S = zeros(1,30);
                    self.A = zeros(1,13);
                    if(strcmp(self.chain,'s'))
                        self.enf = self.spine;
                        self.info.name = 'spine';
                        self.info.type = 'full';
                        self.info.n_joints = 13;
                        self.info.enf = self.spine;
                    elseif(strcmp(self.chain,'r'))
                        self.enf = self.r_arm;
                        self.info.name = 'right_arm';
                        self.info.type = 'full';
                        self.info.n_joints = 13;
                        self.info.enf = self.r_arm;
			self.enf2 = self.l_arm;
                    else
                        self.enf = self.l_arm;
                        self.info.name = 'left_arm';
                        self.info.type = 'full';
                        self.info.n_joints = 13;
                        self.info.enf = self.l_arm;
                    end
                    self.idx = [7:19];
                else
                    fprintf('Error: Enter a chain..!!!');
                end
            end
            
        end
        
        function [] = stability_check(self)
            
            f1 = self.pos{1}*self.scale;
            f1_z = [f1(3)+88,f1(3)-50,f1(3)-50,f1(3)+88,f1(3)+88];
            f1_y = [f1(2)-34,f1(2)-22,f1(2)+22,f1(2)+34,f1(2)-34];
            
            f2 = self.pos{29}*self.scale;
            f2_z = [f2(3)+88,f2(3)-50,f2(3)-50,f2(3)+88,f2(3)+88];
            f2_y = [f2(2)-34,f2(2)-22,f2(2)+22,f2(2)+34,f2(2)-34];
            self.stable = inpolygon(self.zmp(2)*self.scale,self.zmp(1)*self.scale,[f2_z(1),f2_z(2),f1_z(3),f1_z(4),f2_z(1)],[f2_y(1),f2_y(2),f1_y(3),f1_y(4),f2_y(1)]);
        end
        
        function [S_next] = reset(self)
            
            %Configuration
            self.theta = zeros(1,25);
            self.theta(13) = 0.05;
            self.theta(17) = -0.05;
            self.theta(3)  = -0.1745;
            self.theta(2)  =  0.1745;
            self.theta(23) =  0.1745;
            self.theta(24) = -0.1745;
            
            %Angular velocities
            self.dq = zeros(1,25);
            
            if(strcmp(self.mode, 'easy'))
                %Forward Kinematics
                self.pos = forwardKinematics(self.robot,self.theta);
                self.goal = self.pos{self.enf}*self.scale + [10;10;10];
                
            elseif(strcmp(self.mode, 'hard'))
                
                %Random Configuration of upper body
                
                ang = self.theta;
                ang(self.idx) = (self.ang_limit(2,self.idx) - self.ang_limit(1,self.idx)).*rand(1,self.action_dim) + self.ang_limit(1,self.idx);
                
                %Forward Kinematics
                self.pos = forwardKinematics(self.robot,ang);
                self.goal = self.pos{self.enf}*self.scale;
                self.goal2 = self.pos{self.enf2}*self.scale;
            end
            
            %Random Initial Configuration
            self.theta(self.idx)=(self.ang_limit(2,self.idx) - self.ang_limit(1,self.idx)).*rand(1,self.action_dim) + self.ang_limit(1,self.idx);
            %Forward Kinematics
            self.h_prev = zeros(6,1);
            self.theta_ini = self.theta;
            self.pos = forwardKinematics(self.robot,self.theta);
            [self.CoM, self.M, cmm] = centerOfMass(self.robot,self.theta);
            
            %Calculation of centroidal L
            dcom = self.CoM - self.prev_CoM;
            N = self.M*self.CoM(1)*norm(dcom);
            h = cmm*self.dq'*0;
            self.Lc = h(1:3);
            self.Pc = h(4:6);
            self.prev_CoM = self.CoM;
            
            %ZMP Calculation
            [self.tau,self.ho] = inverseDynamics(self.robot,self.theta,self.dq);
            [self.zmp, self.h_prev] = cal_ZMP(self.M, self.CoM, self.ho, self.h_prev);
            
            stability_check(self)
            
            %Dynamics
            % ddq = forwardDynamics(self.robot,self.theta,self.dq,tau);
            
            %Collision check
            col_body = collision_data(self.pos);
            self.cf = collision_check(col_body);
            
            %Initialize flags
            self.in_state = 0;
            self.done1 = 0;
            self.done2 = 0;
            self.grab_control = 0;
            self.grab_control2 = 0;
            
            self.dist1 = norm(self.pos{self.enf}*self.scale-self.goal);
            self.dist2 = norm(self.pos{self.enf2}*self.scale-self.goal2);
            %State vector
            S_next = self.get_state();
        end
        
        
        function [S_next] = get_state(self)
            a_dim = self.action_dim;
            self.S(1:a_dim) = rad2deg(self.theta(self.idx));
            self.S(a_dim+1:a_dim+3) = self.pos{self.enf}*self.scale - self.pos{self.spine}*self.scale;
            self.S(a_dim+4:a_dim+6) = self.goal - self.pos{self.spine}*self.scale;
	    self.S(a_dim+7:a_dim+9) = self.pos{self.enf2}*self.scale - self.pos{self.spine}*self.scale;
            self.S(a_dim+10:a_dim+12) = self.goal2 - self.pos{self.spine}*self.scale;
	    self.S(a_dim+13:a_dim+15) = self.pos{self.spine}*self.scale;
            %self.S(a_dim+16) = self.cf;
            self.S(a_dim+16) = self.done1;
            self.S(a_dim+17) = self.done2;
            S_next = self.S;
        end
        
        function[rew,dist] = reward(self)
            self.done1 = 0;
            self.done2 = 0;
            self.in_state = 0;
            self.dist1 = norm(self.pos{self.enf}*self.scale-self.goal);
            self.dist2 = norm(self.pos{self.enf2}*self.scale-self.goal2);
            
            r = -(self.dist1/70);%+log10(abs(self.Lc(1)))+log10(abs(self.Lc(2)))+log10(abs(self.Lc(3))));
            r1 = -(self.dist2/70);%+log10(abs(self.Lc(1)))+log10(abs(self.Lc(2)))+log10(abs(self.Lc(3))));
            
            %sum = 0;
            %disp(r);
            %disp(dist);
            
%             t_norm = 25*norm(self.tau(self.idx).*(self.dq(self.idx)));
%             wt(1:9) = 4;
%             wt(10:13) = 1;
%             dq_norm = norm(self.theta(self.idx) - self.theta_ini(self.idx).*wt)/(7.3*pi)*5;
%             
            %dq_norm = norm(self.dq(self.idx) - self.theta_ini(self.idx))/(2*pi)*10;

            
%             %for i=1:length(self.idx)
%             %   r = r - 30*(self.dq(self.idx(i))^2);
%             %sum = sum +  wt(i)*(self.dq(self.idx(i))^2);
%             %end
            %disp(sum);
            %disp('-------');
            r = r;	% - dq_norm;
            r1 = r1;	% - dq_norm;
            
            if(self.stable==0 || self.cf==0)
                r = -10*~self.stable + -10*~self.cf;
                r1 = -10*~self.stable + -10*~self.cf;
            end
             
            if(self.dist1 < self.prec_bound && self.stable)
                r = r + 10; %+ self.prec_bound - abs(log10(dist));% - (log10(abs(self.Lc(1)))+log10(abs(self.Lc(2)))+log10(abs(self.Lc(3))));
                self.in_state = 1;
                %print(self.done)
                self.grab_control = self.grab_control + 1;
                
                if(self.dist1 < self.precision || self.grab_control>15)
                    % disp(self.done)
                    % self.theta
                    r = r + 10*self.grab_control; % - 10*(log10(abs(self.Lc(1)))+log10(abs(self.Lc(2)))+log10(abs(self.Lc(3))));
                    self.done1=1;
                end
            else
                self.grab_control = 0;
            end
            
            if(self.dist2 < self.prec_bound && self.stable)
                r1 = r1 + 10; %+ self.prec_bound - abs(log10(dist));% - (log10(abs(self.Lc(1)))+log10(abs(self.Lc(2)))+log10(abs(self.Lc(3))));
                self.in_state = 1;
                %print(self.done)
                self.grab_control2 = self.grab_control2 + 1;
                
                if(self.dist2 < self.precision || self.grab_control2>15)
                    % disp(self.done)
                    % self.theta
                    r1 = r1 + 10*self.grab_control2; % - 10*(log10(abs(self.Lc(1)))+log10(abs(self.Lc(2)))+log10(abs(self.Lc(3))));
                    self.done2=1;
                end
            else
                self.grab_control2 = 0;
            end
            

	   if (self.done1 && self.done2)
		r = 250;
		r1= 250;
	   end
		

	    %disp(norm(self.Lc))
            %	r1 = -norm(self.Lc)*5;
            
            %if (abs(self.Lc(1)) < 0.001 && abs(self.Lc(2)) < 0.001 && abs(self.Lc(3)) < 0.001)
             %   r1 = 2 ;
            %end
          
            dist = [self.dist1,self.dist2];
            rew = [r r1];
            
            
        end
        
        
        function[] = limit_ang(self)
            theta_lim_min = self.ang_limit(1,:);
            theta_lim_max = self.ang_limit(2,:);
            for ix = 1:25
                if(self.theta(ix) < theta_lim_min(ix))
                    self.theta(ix) = theta_lim_min(ix);
                elseif(self.theta(ix) > theta_lim_max(ix))
                    self.theta(ix) = theta_lim_max(ix);
                end
            end
        end
        
        function [in] = clip(self,in,fac)
            %out = zeros(1,length(in));
            for i=1:length(in)
                if(in(i) < self.action_bound(1)/fac)
                    in(i) = self.action_bound(1)/fac;
                elseif(in(i) > self.action_bound(2)/fac)
                    in(i) = self.action_bound(2)/fac;
                end
            end
        end
        
        function [S_next, r, done, dist] = step(self, action)
            %Resetting flags
            self.done1=0;
            self.done2=0;
            self.in_state=0;
            if(self.dist1<60 && self.dist2<60)
            	action = self.clip(action,1.3);
	    end
            tmp = self.theta;
            
            %Configuration and action
            self.theta(self.idx) = self.theta(self.idx) + deg2rad(action)*self.dt;
            
            %Limiting the angles
            self.limit_ang();
            
            self.dq(self.idx) = self.theta(self.idx) - tmp(self.idx);
            
            %Forward Kinematics
            self.pos = forwardKinematics(self.robot,self.theta);
            [self.CoM, self.M, cmm] = centerOfMass(self.robot,self.theta);
            
            %Calculation of centroidal L
            dcom = self.CoM - self.prev_CoM;
            N = self.M*self.CoM(1)*0.1;
            h = cmm*self.dq'/N;
            self.Lc = h(1:3);
            self.Pc = h(4:6);
            self.prev_CoM = self.CoM;
            %ZMP Calculation
            [self.tau,self.ho] = inverseDynamics(self.robot,self.theta,self.dq);
            [self.zmp, self.h_prev] = cal_ZMP(self.M, self.CoM, self.ho, self.h_prev);
            
            %ZMP Check
            stability_check(self)
            %draw(self) 
            %Dynamics
            % ddq = forwardDynamics(self.robot,self.theta,self.dq,tau);
            
            %Collision check
            col_body = collision_data(self.pos);
            self.cf = collision_check(col_body);
            
            %Getting reward
            [r, dist] = self.reward();
            
            %Getting state vector
            S_next = self.get_state();

            %Done
            done = [self.done1, self.done2];
        end
        
    end
end
