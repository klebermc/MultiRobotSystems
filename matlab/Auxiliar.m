classdef Auxiliar
    methods(Static)
        function [ output ] = sigma_norm( input )
           epslon=0.5;
           output=(1/epslon)*(sqrt(1+epslon*(norm(input,2)^2))-1);
        end

        function [rho] = rho_h (z)
            if z >= 0 && z < 0.5
                rho=1;
            elseif z >= 0.5 && z<1
                rho= 0.5 * (1 + cos( pi * ((z-0.5)/(1-0.5))));
            else 
                rho=0;
            end
        end

        function [sigma1] = sigma_1(z)
            sigma1=z/(sqrt(1+norm(z)^2));
        end

        function h = circle(x,y,r)
            th = 0:pi/50:2*pi;
            xunit = r * cos(th) + x;
            yunit = r * sin(th) + y;
            h = plot(xunit, yunit,'r','linewidth',1.5);
        end
        
        function h = block(x,y,colour)
            d=0.13;
            switch colour
                case 'grey'
                    colour=[0.8 0.8 0.8];
                case 'blue'
                    colour=[0.0 0.0 0.8];
                case 'red'
                    colour=[0.8 0.0 0.0];
                case 'green'
                    colour=[0.0 0.8 0.0];
                case 'magenta'
                    colour=[0.8 0.0 0.8];
            end        
            h=rectangle('Position',[x-d/2,y-d/2,d,d],'FaceColor',colour,'EdgeColor','k','LineWidth',1.5);
        end
        
        function h = block3d(x,y,z,colour)
            switch colour
                case 'grey'
                    colour=[0.8 0.8 0.8];
                case 'blue'
                    colour=[0.0 0.0 0.8];
                case 'red'
                    colour=[0.8 0.0 0.0];
                case 'green'
                    colour=[0.0 0.8 0.0];
                case 'magenta'
                    colour=[0.8 0.0 0.8];
            end
            d=0.4;
            a = -pi : pi/2 : pi;                                % Define Corners
            ph = pi/4;                                          % Define Angular Orientation (‘Phase’)
            X = [cos(a+ph); cos(a+ph)]/cos(ph); X = X*d/2 + x;
            Y = [sin(a+ph); sin(a+ph)]/sin(ph); Y = Y*d/2 + y;
            Z = [-ones(size(a)); ones(size(a))];Z = Z*d/2 + z;
            h=surf(X, Y, Z, 'FaceColor',colour,'EdgeColor','k'); % Plot Cube
            h=[h,patch(X', Y', Z', colour)]; % Make Cube Appear Solid
        end
        

    end
end
