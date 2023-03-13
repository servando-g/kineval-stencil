function update_pendulum_state(numerical_integrator, pendulum, dt, gravity) {
    // integrate pendulum state forward in time by dt
    // please use names 'pendulum.angle', 'pendulum.angle_previous', etc. in else codeblock between line 28-30

    if (typeof numerical_integrator === "undefined")
        numerical_integrator = "none";

    if (numerical_integrator === "euler") {

    // STENCIL: a correct Euler integrator is REQUIRED for assignment

        pendulum.angle_previous = pendulum.angle
        accel = pendulum_acceleration(pendulum, gravity)

        pendulum.angle = pendulum.angle_previous + (pendulum.angle_dot * dt)
        pendulum.angle_dot = pendulum.angle_dot + (accel * dt)

    }
    else if (numerical_integrator === "verlet") {

    // STENCIL: basic Verlet integration

    }
    else if (numerical_integrator === "velocity verlet") {

    // STENCIL: a correct velocity Verlet integrator is REQUIRED for assignment

        pendulum.angle_previous = pendulum.angle
        accel2 = pendulum_acceleration(pendulum, gravity)

        pendulum.angle = pendulum.angle + pendulum.angle_dot*dt + ((1/2) * accel2 * (dt**2))
        pendulum.angle_dot = pendulum.angle_dot + ((accel2 + pendulum_acceleration(pendulum,gravity))/2)*dt
    
    }
    else if (numerical_integrator === "runge-kutta") {

    // STENCIL: Runge-Kutta 4 integrator
    } 
    else {
        pendulum.angle_previous = pendulum.angle;
        pendulum.angle = (pendulum.angle+Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot = (pendulum.angle-pendulum.angle_previous)/dt;
        numerical_integrator = "none";
    }

    return pendulum;
}

function pendulum_acceleration(pendulum, gravity) {
    // STENCIL: return acceleration(s) system equation(s) of motion 
    acceleration = -(gravity/pendulum.length)*Math.sin(pendulum.angle)+pendulum.control/(pendulum.mass * (pendulum.length**2));

    return acceleration;
    
}

function init_verlet_integrator(pendulum, t, gravity) {
    // STENCIL: for verlet integration, a first step in time is needed
    // return: updated pendulum state and time

    return [pendulum, t];
}

function set_PID_parameters(pendulum) {
    // STENCIL: change pid parameters
<<<<<<< HEAD:project_pendularm/update_pendulum_state.js
    pendulum.servo = {kp:1500, kd:200, ki:500};  // no control
=======
    pendulum.servo = {kp:100, kd:50, ki:10};  // no control
>>>>>>> 7a78386440ea3790e88c8e30b6e92014a6f0f6de:kineval-stencil/project_pendularm/update_pendulum_state.js
    return pendulum;
}

function PID(pendulum, accumulated_error, dt) {
    // STENCIL: implement PID controller
    // return: updated output in pendulum.control and accumulated_error

    pendulum.servo.error = pendulum.desired - pendulum.angle;
    pendulum.servo.previous_error = pendulum.desired-pendulum.angle_previous;

    pendulum.control = pendulum.servo.kp*pendulum.servo.error + pendulum.servo.ki*accumulated_error + pendulum.servo.kd*(pendulum.servo.error-pendulum.servo.previous_error)/dt;
    accumulated_error = pendulum.servo.error + accumulated_error;


    return [pendulum, accumulated_error];
}