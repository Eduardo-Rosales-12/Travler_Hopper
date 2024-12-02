# Place this snippet inside the main execution block before the movement logic
if __name__ == "__main__":
    nodes = [0, 1]
    
    bus = can.interface.Bus("can0", interface="socketcan")
    
    while not (bus.recv(timeout=0) is None):
        pass

    Motor0, Motor1 = nodes[0], nodes[1]

    set_closed_loop_control(Motor0)
    set_closed_loop_control(Motor1)
    set_velocity_control_mode(Motor0)
    set_velocity_control_mode(Motor1)

    # Setup data log
    data_log = []

    # Start logging data
    New_Time = time.perf_counter()
    Elapsed_Start_Time = time.perf_counter()
    
    print("Data logging started. Idling for 2 seconds...")
    time.sleep(2)  # Introduce a 2-second idle period

    target_position = 0.1
    target_vel = -9
    current_position_0, current_velocity_0 = encoder_estimates(Motor0)
    current_position_1, current_velocity_1 = encoder_estimates(Motor1)
    
    motor0_tor = get_torque_estimate(Motor0)
    motor1_tor = get_torque_estimate(Motor1)
    latch = 0
    latch2 = 0

    try:
        
        while True:
            print(current_position_0, current_position_1)
            New_Time = time.perf_counter()  # Update time inside loop
            current_position_0, current_velocity_0 = encoder_estimates(Motor0)
            current_position_1, current_velocity_1 = encoder_estimates(Motor1)
            motor0_tor = get_torque_estimate(Motor0)
            motor1_tor = get_torque_estimate(Motor1)
            
            elapsed_time = New_Time - Elapsed_Start_Time
            current_time = time.time()  
            global_time = datetime.now()
            
            formatted_global_time = global_time.strftime("%Y-%m-%d %H:%M:%S.%f")
            data_log.append([formatted_global_time, elapsed_time, current_position_0, current_position_1, motor0_tor, motor1_tor])

            phi_1, phi_2, phi_1_vel, phi_2_vel = get_state_variables(current_position_0, current_position_1, current_velocity_0, current_velocity_1)
            
            if (current_position_0 > target_position or current_position_1 > target_position) and (latch == 0):
                set_velocity(Motor0, target_vel)
                set_velocity(Motor1, target_vel)
                
            elif (current_position_0 < target_position or current_position_1 < target_position) and (latch2 == 0):
                last_pos0 = current_position_0
                last_pos1 = current_position_1
                latch = 1
                latch2 = 1
                
            if(latch == 1 and latch2 == 1):
                set_position_control_mode(Motor0)
                set_position_control_mode(Motor1)
                set_position(Motor0,  last_pos0-0.09)
                set_position(Motor1,  last_pos1-0.09)
    
    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Setting nodes to idle...")
        for node_id in nodes:
            set_idle(node_id)
        print("Nodes set to idle. Exiting program.")

    # Save data logic remains unchanged...
