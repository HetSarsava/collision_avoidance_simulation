function V2V_MPC_Dynamic_Safety_Zones()
    % 1. Setup Figure
    hFig = figure('Color',[0.15 0.15 0.15], 'Name', 'V2V MPC: Dynamic Safety Zones', ...
                  'Position', [100 100 1200 650]); 
    
    % --- LEFT PANEL: SIMULATION ---
    axSim = subplot(1, 2, 1);
    set(axSim, 'Color', [0.15 0.15 0.15], 'Position', [0.13 0.2 0.33 0.7]); 
    axis(axSim, 'equal', 'off'); hold(axSim, 'on');
    title(axSim, 'Dynamic Safety Zones (Scales with Speed)', 'Color', 'w');
    
    % --- RIGHT PANEL: VELOCITY PLOT ---
    axPlot = subplot(1, 2, 2);
    set(axPlot, 'Color', [0.2 0.2 0.2], 'XColor', 'w', 'YColor', 'w', ...
                 'Position', [0.57 0.2 0.33 0.7]); 
    hold(axPlot, 'on');
    title(axPlot, 'Velocity Profiles', 'Color', 'w');
    xlabel(axPlot, 'Time (s)'); ylabel(axPlot, 'Speed (units/s)');
    grid(axPlot, 'on');
    ylim(axPlot, [0 45]); 
    
    hLine1 = animatedline('Parent', axPlot, 'Color', [1 0.8 0], 'LineWidth', 2); % Orange
    hLine2 = animatedline('Parent', axPlot, 'Color', [0 1 1], 'LineWidth', 2);   % Cyan
    legend(axPlot, [hLine1, hLine2], {'Car 1 (East)', 'Car 2 (North)'}, ...
           'TextColor', 'w', 'Color', [0.1 0.1 0.1], 'Location', 'SouthWest');

    % --- SPEED CONTROL ---
    sim_delay = 0.05; 
    hSpeedLbl = uicontrol('Parent', hFig, 'Style', 'text', ...
        'String', sprintf('Simulation Speed: %.0f ms delay', sim_delay*1000), ...
        'Units', 'normalized', 'Position', [0.3 0.08 0.4 0.04], ...
        'BackgroundColor', [0.15 0.15 0.15], 'ForegroundColor', 'w', ...
        'FontSize', 10, 'FontWeight', 'bold');
    
    uicontrol('Parent', hFig, 'Style', 'slider', ...
        'Min', 0.0, 'Max', 0.2, 'Value', sim_delay, ...
        'Units', 'normalized', 'Position', [0.3 0.04 0.4 0.03], ...
        'Callback', @update_speed);
        
    function update_speed(src, ~)
        sim_delay = get(src, 'Value');
        set(hSpeedLbl, 'String', sprintf('Delay: %.0f ms', sim_delay*1000));
        set(hFig, 'CurrentObject', []); 
    end

    % --- 2. MAP SETUP ---
    map_size = 200; 
    xlim(axSim, [0 map_size]); ylim(axSim, [0 map_size]);
    
    fill(axSim, [0, 200, 200, 0], [92, 92, 108, 108], [0.3 0.3 0.3], 'EdgeColor', 'none');
    fill(axSim, [92, 92, 108, 108], [0, 200, 200, 0], [0.3 0.3 0.3], 'EdgeColor', 'none');
    fill(axSim, [92, 108, 108, 92], [92, 92, 108, 108], [0.2 0.2 0.2], 'EdgeColor', 'w'); 
    
    % --- 3. PARAMETERS ---
    dt = 0.1; Np = 20;                
    w_v = 10; w_da = 100; w_a = 0.1;  
    MAX_SPEED = 40; MAX_ACCEL = 10; MIN_ACCEL = -20; MAX_JERK  = 20;         
    
    car_w = 4; car_l = 6;
    cars = struct('id', {}, 'pos', {}, 'angle', {}, 'speed', {}, 'accel', {}, ...
                  'h', {}, 'hBody', {}, 'hHitbox', {}, 'state_color', {}); 
    
    sim_time = 0;
    
    % --- 4. MAIN LOOP ---
    while ishandle(hFig)
        sim_time = sim_time + dt;
        
        if isempty(cars)
            clearpoints(hLine1); clearpoints(hLine2);
            offset = 4; 
            
            % Car 1: Eastbound
            c1 = create_car(axSim, 1, [0, 100+offset], 0, [1 0.5 0], car_w, car_l, 35);
            % Car 2: Northbound
            c2 = create_car(axSim, 2, [100-offset, 0], pi/2, [0 1 1], car_w, car_l, 35);
            
            cars = [c1, c2];
        end
        
        % B. LOGIC & PHYSICS
        for i = 1:length(cars)
            me = cars(i);
            others = cars([1:length(cars)] ~= i);
            
            % Solve Math using DYNAMIC Box Length
            [opt_accel, is_overlap, box_len] = solve_mpc_dynamic_hitbox(me, others, dt, Np, w_v, w_da, w_a, ...
                                      MAX_SPEED, MAX_ACCEL, MIN_ACCEL, MAX_JERK, car_w, car_l);
            
            next_accel = me.accel + opt_accel; 
            me.speed = me.speed + next_accel * dt;
            if me.speed < 0, me.speed = 0; end
            if me.speed > MAX_SPEED, me.speed = MAX_SPEED; end
            
            me.pos = me.pos + [cos(me.angle), sin(me.angle)] * me.speed * dt;
            me.accel = next_accel;
            
            % Update Color based on status
            if is_overlap
                set(me.hHitbox, 'EdgeColor', 'r', 'LineWidth', 2);
            else
                set(me.hHitbox, 'EdgeColor', 'g', 'LineWidth', 1);
            end
            
            % Update Box Visual Length (Dynamic!)
            update_hitbox_visual(me.hHitbox, box_len, car_w, car_l);

            cars(i) = me;
        end
        
        % C. UPDATE VISUALS
        for i = 1:length(cars)
            c = cars(i);
            M = makehgtform('translate', [c.pos(1) c.pos(2) 0], 'zrotate', c.angle);
            set(c.h, 'Matrix', M);
            
            if c.id == 1, addpoints(hLine1, sim_time, c.speed); 
            elseif c.id == 2, addpoints(hLine2, sim_time, c.speed);
            end
            
            if c.pos(1)>map_size+20 || c.pos(2)>map_size+20
               delete(c.h); cars(i) = []; break; 
            end
        end
        drawnow; pause(sim_delay); 
    end
end

% --- SOLVER (Dynamic Safety Box) ---
function [u_opt, is_overlap, extension_len] = solve_mpc_dynamic_hitbox(me, others, dt, Np, w_v, w_da, w_a, ...
                                           v_max, a_max, a_min, j_max, cw, cl)
    H = eye(Np) * w_da;  
    v_ref = v_max; 
    Tril = tril(ones(Np)); M_v = dt * Tril; 
    H = H + (M_v' * eye(Np) * w_v * M_v);
    v_base = me.speed + (me.accel * dt * (1:Np)'); 
    f = M_v' * eye(Np) * w_v * (v_base - v_ref); 

    is_overlap = false;
    
    % --- DYNAMIC LENGTH CALCULATION ---
    % Stop Distance approx v^2 / 2a plus reaction buffer.
    % Clamp braking accel to avoid divide-by-zero.
    brake_accel = max(abs(a_min), 1);
    reaction_time = 0.75;
    extension_len = cl + (me.speed * reaction_time) + (me.speed^2 / (2 * brake_accel));
    
    % Get My Dynamic Box
    [my_poly_x, my_poly_y] = get_world_hitbox(me.pos, me.angle, cw, cl, extension_len);
    
    if ~isempty(others)
        should_yield = false;
        for idx = 1:length(others)
            other = others(idx);

            % Get Their Dynamic Box (They also project forward)
            other_brake_accel = max(abs(a_min), 1);
            other_ext = cl + (other.speed * reaction_time) + (other.speed^2 / (2 * other_brake_accel));
            [other_poly_x, other_poly_y] = get_world_hitbox(other.pos, other.angle, cw, cl, other_ext);

            if check_poly_overlap(my_poly_x, my_poly_y, other_poly_x, other_poly_y)
                is_overlap = true;

                % --- STRICT PRIORITY LOGIC ---

                % 1. Rear End Check (Am I truly behind?)
                % Vector from me to him
                vec_to = other.pos - me.pos;
                % Project onto my heading
                dist_forward = dot(vec_to, [cos(me.angle), sin(me.angle)]);

                % If he is essentially "parallel" to me and in front -> I Yield
                angle_diff = abs(angdiff(me.angle, other.angle));
                is_parallel = angle_diff < pi/4;

                if is_parallel && dist_forward > 0
                    should_yield = true;
                else
                    % 2. Crossing / Intersection (Perpendicular)
                    % STRICT ID CHECK. Do NOT check "who is in front" because
                    % in a cross, both think the other is in front.
                    if me.id > other.id
                        should_yield = true; % I yield to Lower ID
                    end
                end
            end
        end

        if should_yield
            v_ref = 0;
            f = M_v' * eye(Np) * w_v * (v_base - 0);
        end
    end

    A_ineq = []; b_ineq = [];
    A_acc = Tril;
    b_acc_max = (a_max - me.accel) * ones(Np, 1);
    b_acc_min = (me.accel - a_min) * ones(Np, 1);
    A_ineq = [A_ineq; A_acc; -A_acc]; b_ineq = [b_ineq; b_acc_max; b_acc_min];
    
    A_vel = M_v;
    b_vel_max = v_max - v_base; b_vel_min = v_base - 0; 
    A_ineq = [A_ineq; A_vel; -A_vel]; b_ineq = [b_ineq; b_vel_max; b_vel_min];
    
    A_ineq = [A_ineq; eye(Np); -eye(Np)]; b_ineq = [b_ineq; j_max * ones(Np,1); j_max * ones(Np,1)];
    
    [u_seq, status] = hildreth_qp_solver(H, f, A_ineq, b_ineq);
    if status == -1, u_opt = -5; else, u_opt = u_seq(1); end
end

% --- GEOMETRY HELPERS ---

function [px, py] = get_world_hitbox(pos, angle, w, l, ext_len)
    % Dynamic Extension
    dx = [-l/2,  l/2 + ext_len,  l/2 + ext_len, -l/2];
    dy = [-w/2, -w/2,            w/2,           w/2];
    
    R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
    coords = R * [dx; dy];
    
    px = coords(1,:) + pos(1);
    py = coords(2,:) + pos(2);
end

function update_hitbox_visual(hPatch, ext_len, w, l)
    % Updates the local coordinates of the patch to match speed
    dx = [-l/2,  l/2 + ext_len,  l/2 + ext_len, -l/2];
    dy = [-w/2, -w/2,            w/2,           w/2];
    set(hPatch, 'XData', dx, 'YData', dy);
end

function overlap = check_poly_overlap(x1, y1, x2, y2)
    overlap = false;
    if any(inpolygon(x1, y1, x2, y2)), overlap = true; return; end
    if any(inpolygon(x2, y2, x1, y1)), overlap = true; return; end
end

% --- STANDARD HELPERS ---
function [x, status] = hildreth_qp_solver(H, f, A, b)
    H_reg = H + eye(size(H))*1e-6;
    H_inv = H_reg \ eye(size(H_reg));
    P = A * H_inv * A';
    d = b + A * H_inv * f;
    lambda = zeros(size(A,1), 1);
    for iter = 1:100
        lambda_p = lambda;
        for i = 1:length(lambda)
            w = P(i,:)*lambda - P(i,i)*lambda(i);
            if abs(P(i,i)) < 1e-12
                continue;
            end
            lambda(i) = max(0, -(d(i)+w)/P(i,i));
        end
        if norm(lambda - lambda_p) < 1e-6, break; end
    end
    x = -H_inv * (f + A' * lambda); status = 1; 
end

function car = create_car(ax, id, pos, angle, color, w, l, init_speed)
    hGroup = hgtransform('Parent', ax);
    
    % Body
    bx = [-l/2, l/2, l/2, -l/2]; 
    by = [-w/2, -w/2, w/2, w/2];
    hBody = fill(bx, by, color, 'EdgeColor', 'k', 'Parent', hGroup);
    
    % Hitbox (Initial small size)
    hx = [-l/2, l/2+l, l/2+l, -l/2];
    hy = [-w/2, -w/2, w/2, w/2];
    
    hHitbox = fill(hx, hy, 'w', 'FaceColor', 'none', 'EdgeColor', 'g', ...
                   'LineWidth', 1, 'Parent', hGroup);
    
    set(hGroup, 'Matrix', makehgtform('translate', [pos(1) pos(2) 0], 'zrotate', angle));
    
    car = struct('id', id, 'pos', pos, 'angle', angle, ...
                 'speed', init_speed, 'accel', 0, 'h', hGroup, 'hBody', hBody, ...
                 'hHitbox', hHitbox, 'mpc_plan_v', [], 'state_color', color);
end
