clear
clc
close all

function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
    %{ 
    inputs:
        time: 1 x N time vector
        a/c state array: 12 x N state matrix
        control input array: 4 x N control vector
        fig: 6x1 vector figures
        col: plot line spec
    %}

    if nargin < 5
        col = 'b';
    end

    %% FIGURE 1 Inertial Position x y z
    figure(fig(1))

    subplot(3,1,1)
    plot(time, aircraft_state_array(1,:), col)
    hold on
    xlabel('t (s)')
    ylabel('x (m)')
    title('Inertial Position x vs Time')

    subplot(3,1,2)
    plot(time, aircraft_state_array(2,:), col)
    hold on
    xlabel('t (s)')
    ylabel('y (m)')
    title('Inertial Position y vs Time')

    subplot(3,1,3)
    plot(time, aircraft_state_array(3,:), col)
    hold on
    xlabel('t (s)')
    ylabel('z (m)')
    title('Inertial Position z vs Time')

    grid on

    %% FIGURE 2 Euler Angles phi theta psi
    figure(fig(2))

    subplot(3,1,1)
    plot(time, rad2deg(aircraft_state_array(4,:)), col)
    hold on
    xlabel('t (s)')
    ylabel('\phi (deg)')
    title('Euler Angle \phi')

    subplot(3,1,2)
    plot(time, rad2deg(aircraft_state_array(5,:)), col)
    hold on
    xlabel('t (s)')
    ylabel('\theta (deg)')
    title('Euler Angle \theta')

    subplot(3,1,3)
    plot(time, rad2deg(aircraft_state_array(6,:)), col)
    hold on
    xlabel('t (s)')
    ylabel('\psi (deg)')
    title('Euler Angle \psi')

    grid on

    %% FIGURE 3 Inertial Velocity in Body Frame u v w
    figure(fig(3))

    subplot(3,1,1)
    plot(time, aircraft_state_array(7,:), col)
    hold on
    xlabel('t (s)')
    ylabel('u (m/s)')
    title('Body Velocity u')

    subplot(3,1,2)
    plot(time, aircraft_state_array(8,:), col)
    hold on
    xlabel('t (s)')
    ylabel('v (m/s)')
    title('Body Velocity v')

    subplot(3,1,3)
    plot(time, aircraft_state_array(9,:), col)
    hold on
    xlabel('t (s)')
    ylabel('w (m/s)')
    title('Body Velocity w')

    grid on

    %% FIGURE 4 Angular Velocity p q r
    figure(fig(4))

    subplot(3,1,1)
    plot(time, rad2deg(aircraft_state_array(10,:)), col)
    hold on
    xlabel('t (s)')
    ylabel('p (deg/s)')
    title('Angular Velocity p')

    subplot(3,1,2)
    plot(time, rad2deg(aircraft_state_array(11,:)), col)
    hold on
    xlabel('t (s)')
    ylabel('q (deg/s)')
    title('Angular Velocity q')

    subplot(3,1,3)
    plot(time, rad2deg(aircraft_state_array(12,:)), col)
    hold on
    xlabel('t (s)')
    ylabel('r (deg/s)')
    title('Angular Velocity r')

    grid on

    %% FIGURE 5 Control Inputs
    figure(fig(5))

    subplot(4,1,1)
    plot(time, control_input_array(1,:), col)
    hold on
    xlabel('t (s)')
    ylabel('Zc (N)')
    title('Control Zc')

    subplot(4,1,2)
    plot(time, control_input_array(2,:), col)
    hold on
    xlabel('t (s)')
    ylabel('Lc (Nm)')
    title('Control Lc')
    
    subplot(4,1,3)
    plot(time, control_input_array(3,:), col)
    hold on
    xlabel('t (s)')
    ylabel('Mc (Nm)')
    title('Control Mc')

    subplot(4,1,4)
    plot(time, control_input_array(4,:), col)
    hold on
    xlabel('t (s)')
    ylabel('Nc (Nm)')
    title('Control Nc')

    grid on

    %% FIGURE 6 3D Path -> z positive up
    figure(fig(6))

    plot3(aircraft_state_array(1,:), aircraft_state_array(2,:), -aircraft_state_array(3,:), col)
    hold on
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)')
    title('3D Path (Start = Green, Finish = Red)')
    startPt = [aircraft_state_array(1,1), aircraft_state_array(2,1), -aircraft_state_array(3,1)];
    endPt = [aircraft_state_array(1,end), aircraft_state_array(2,end), -aircraft_state_array(3,end)];
    plot3(startPt(1), startPt(2), startPt(3), 'go', 'MarkerFaceColor', 'g')
    plot3(endPt(1), endPt(2), endPt(3), 'ro', 'MarkerFaceColor', 'r')
    grid on
    axis equal
end