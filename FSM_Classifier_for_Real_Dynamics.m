%-----------------------------------------------------------------------
% This code is a finite state machine created using switch case statements, designed to classify the
% joint state of a tactile sensor to determine whether the sensor is actively in contact or not. 
% The transition thresholds are tuned to real world sensor dynamics and can be used in real-time on a robot
% or to classify existing sensor joint state data.
% By Adam Gronewold
%----------------------------------------------------------------------


if i <= sensor.classifier_window
    sensor.current_state="Unknown State";
elseif i > sensor.classifier_window      
    % Determine state based on conditions
    switch sensor.current_state
        case "Unknown State"
            if all(abs(sensor.ydot_angle_h(i-sensor.classifier_window:i)) < 40)
                sensor.current_state= "Steady State Angle";
            elseif sensor.ydot_angle_h(i) < 0 && sensor.yddot_angle_h(i) < -10000
                sensor.current_state = "In Contact";
            elseif all(sensor.ydot_angle_h(i-sensor.classifier_window:i) > 0)
                sensor.current_state= "SHM Increasing";
            elseif all(sensor.ydot_angle_h(i-sensor.classifier_window:i) < 0) && all(sensor.y_angle_h(i-sensor.classifier_window:i) > 0)
                sensor.current_state= "SHM Decreasing";
            end

        case "Steady State Angle"
            if sensor.ydot_angle_h(i) < -25 && sensor.yddot_angle_h(i) < -500 
                sensor.current_state = "In Contact";
            elseif all(sensor.ydot_angle_h(i-sensor.classifier_window:i) > 1)
                sensor.current_state= "SHM Increasing";
            end
            
        case "SHM Increasing"
            if all(abs(sensor.ydot_angle_h(i-sensor.classifier_window:i)) < 40)
                sensor.current_state= "Steady State Angle";
            elseif sensor.y_angle_h(i) > 0 && abs(sensor.ydot_angle_h(i)) < 300 && sensor.yddot_angle_h(i) < 0
                sensor.current_state= "SHM Decreasing";
            elseif sensor.ydot_angle_h(i) < -30 && sensor.yddot_angle_h(i) > 0
                sensor.current_state = "In Contact";
            end

        case "SHM Decreasing"
            if all(abs(sensor.ydot_angle_h(i-sensor.classifier_window:i)) < 40)
                sensor.current_state= "Steady State Angle";
            elseif sensor.ydot_angle_h(i) < 0 && sensor.yddot_angle_h(i) < -10000
                sensor.current_state = "In Contact";
            elseif sensor.y_angle_h(i) < 0 && abs(sensor.ydot_angle_h(i)) < 100 && sensor.yddot_angle_h(i) > 0
                sensor.current_state= "SHM Increasing";
            end
            
        case "In Contact"
            if all(sensor.ydot_angle_h(i-sensor.classifier_window:i) > 0)
                sensor.current_state= "SHM Increasing";
            elseif all(abs(sensor.ydot_angle_h(i-sensor.classifier_window:i)) < 8) ...
                    && all(abs(sensor.yddot_angle_h(i-sensor.classifier_window:i)) < 1000)
                    sensor.current_state= "Steady State Angle";
            elseif sensor.y_angle_h(i)<-75
                    sensor.current_state= "Nearing Crash";
            end

        case "Nearing Crash"
            %if sensor.ydot_angle_h(i) < 0 && sensor.y_angle_h(i) < 0
            %    sensor.f_y_angle_classifier_h(i)h(i) = "In Contact";
            if all(sensor.ydot_angle_h(i-sensor.classifier_window:i) > 0)
                sensor.current_state= "SHM Increasing";
            elseif all(abs(sensor.ydot_angle_h(i-sensor.classifier_window:i)) < 8) ...
                    && all(abs(sensor.yddot_angle_h(i-sensor.classifier_window:i)) < 1000)
                sensor.current_state= "Steady State Angle";
            end
            
        otherwise
            sensor.current_state= "Unknown State";
    end
end
