import spacecraft as sc
import launch_utils as utils
import mission
import controllers
import time
import plotting_utils as pu
import matplotlib.pyplot as plt
import steering_logic as steering
from digitalfilter import low_pass_filter as LPF
import numpy as np
from math import radians, sin, cos, sqrt, atan2, degrees

import threading
from queue import Queue

from launch_utils import DcxControlMode, enter_control_mode
from telemetry import KSPTelemetry

telem_viz = KSPTelemetry()
telem_viz.start_metrics_server()
telem_viz.register_enum_metric(utils.CONTROL_MODE, "The enumerated control mode of the flight computer",
                               [mode.name for mode in DcxControlMode])
telem_viz.register_gauge_metric('distance_to_pad', 'distance_to_pad')
telem_viz.register_gauge_metric('current_horizontal_velocity', 'current_horizontal_velocity')
telem_viz.register_gauge_metric('distance_pitch', 'distance_pitch')
telem_viz.register_gauge_metric('pitch_input', 'pitch_input')
telem_viz.register_gauge_metric('heading_error', 'heading_error')
telem_viz.register_gauge_metric('heading', 'heading')
telem_viz.register_gauge_metric('roll', 'roll')
telem_viz.register_gauge_metric('roll_input', 'roll_input')
telem_viz.register_gauge_metric('pitch', 'pitch (mode 3)')
telem_viz.register_gauge_metric('throttle', 'throttle')
telem_viz.register_gauge_metric('time_to_burn', 'time_to_burn')
telem_viz.register_gauge_metric('vert_vel_setpoint', 'vert_vel_setpoint')
telem_viz.register_gauge_metric('alt_controller_setpoint', 'alt_controller_setpoint')

telem_viz.register_counter_metric('gnc_frame_count', 'gnc_frame_count')
telem_viz.register_counter_metric('gnc_overrun_count', 'gnc_overrun_count')

telem_viz.register_histogram_metric('calculate_landing_burn_time', 'calculate_landing_burn_time')
telem_viz.register_histogram_metric('calculate_heading', 'calculate_heading')
telem_viz.register_histogram_metric('haversine_distance', 'haversine_distance')
telem_viz.register_histogram_metric('publish_gnc_metrics_b', 'publish_gnc_metrics_b')
telem_viz.register_histogram_metric('get_flight_path_angle', 'get_flight_path_angle')
telem_viz.register_histogram_metric('calc_lb_final_math', 'calc_lb_final_math')

enter_control_mode(DcxControlMode.PAD, telem_viz)

# this code is testing the newly implemented CascadingController for the pad targeting control loop

def euler_step(vessel, h, v, dt):
    h = h + v * dt
    r = vessel.orbit.body.equatorial_radius + h 
    Ft = 0 # coasting, no thrust
    rho = 1.7185*np.exp(-2e-04*h)
    Cd_A = 3.5 # kg/m^3
    Fd =  0.5*rho*v**2*Cd_A
    Fg = vessel.orbit.body.gravitational_parameter * vessel.mass / r**2
    a = (Ft - Fd - Fg) / vessel.mass
    v = v + a * dt

    return h, v

def compute_los_angle(current_position, target_position):
    delta_x = target_position[0] - current_position[0]  # Change in latitude
    delta_y = target_position[1] - current_position[1]   # Change in longitude
    # print("target_position", target_position, "current_position:", current_position)
    # print("delta_x:", delta_x, "delta_y:", delta_y)
    los_angle = np.arctan2(-delta_y, delta_x)
    # print("los_angle (radians):", los_angle)
    return los_angle

def corrected_compute_los_angle(current_position, target_position):
    delta_y = target_position[0] - current_position[0]  # Change in latitude
    delta_x = target_position[1] - current_position[1]   # Change in longitude
    # print("target_position", target_position, "current_position:", current_position)
    # print("delta_x:", delta_x, "delta_y:", delta_y)
    los_angle = np.arctan2(delta_y, delta_x)
    # print("los_angle (radians):", los_angle)
    return los_angle

def compute_los_rate(current_velocity, current_position, target_position):
    los_angle = compute_los_angle(current_position, target_position)
    los_angle = normalize_angle(np.degrees(los_angle))
    vel_angle = np.arctan2(current_velocity[0], current_velocity[1])
    vel_angle = normalize_angle(np.degrees(vel_angle))

    # Compute shortest angle difference
    los_rate = vel_angle - los_angle
    # if los_rate >= 180: ******
    #     los_rate -= 360
    # elif los_rate < -180:
    #     los_rate += 360

    return los_rate

def compass_heading(angle):
    offset = 90
    if angle > 180:
        heading = normalize_angle(angle-offset)
    else:
        heading = normalize_angle(offset-angle)
    
    return heading

def normalize_angle(angle):
    return angle % 360

@telem_viz.get_histogram_metric('haversine_distance').time()
def haversine_distance(lat1, lon1, lat2, lon2):
    r = 600000
    phi1 = radians(lat1)
    phi2 = radians(lat2)
    delta_phi = radians(lat2-lat1)
    delta_lambda = radians(lon2-lon1)
    a = sin(delta_phi / 2)**2 + cos(phi1) * cos(phi2) * sin(delta_lambda / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    distance = r * c
    return distance

def compute_heading_error(current_heading, target_heading):
    error = target_heading - current_heading
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    return error

# constants
CLOCK_RATE = 30  # refresh rate [Hz]
TELEM_RATE = 1.0  # refresh rate for telemetry aquistion [Hz]
target_alt = 62000
root_vessel = "DCX"
is_abort_installed = False
abort_criteria = 20.0  # maximum off-angle before automated abort triggered

upper_stage_LF = 0
payload_LF = 0

meco_condition_multiplier = 0.06  # 0 to ignore condition, otherwise set to desired
#    1st stage liquid fuel percentage at MECO
v_stage = 100000  # velocity target for 45 degree pitch over

conn, vessel = utils.initialize()
mission_params = mission.MissionParameters(root_vessel,
                                           state="init",
                                           target_inc=0,
                                           target_roll=180,
                                           altimeter_bias=71,
                                           grav_turn_end=85000,
                                           max_q=16000,
                                           max_g=3.0)
dcx = sc.launch_vehicle(vessel, CLOCK_RATE, root_vessel,
                        mission_params.altimeter_bias, v_stage, upper_stage_LF,
                        payload_LF, meco_condition_multiplier)
mission_params.target_heading = utils.set_azimuth(vessel,
                                                  mission_params.target_inc,
                                                  dcx.bref)
starting_LF = vessel.resources.amount("LiquidFuel")
target_lat = vessel.flight().latitude
target_lon = vessel.flight().longitude
landing_site = (target_lat, target_lon)

# Create the hybrid reference frame
body = vessel.orbit.body
ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
    position=body.reference_frame,
    rotation=vessel.surface_reference_frame,
    velocity=body.reference_frame
)

vessel_mass = conn.add_stream(getattr, vessel, 'mass')
vessel_available_thrust = conn.add_stream(getattr, vessel, 'available_thrust')
vessel_max_thrust = conn.add_stream(getattr, vessel, 'max_thrust')
vessel_throttle = conn.add_stream(getattr, vessel.control, 'throttle')
s_g = vessel.orbit.body.surface_gravity # is this the same as g calculated later in the script?

# Pre-Launch
utils.launch_countdown(1)
enter_control_mode(DcxControlMode.IGNITION, telem_viz)
vessel.control.activate_next_stage()  # Engine Ignition

if vessel.available_thrust < 10:
    for engine in vessel.parts.engines:
        engine.active = True

vessel.auto_pilot.engage()
vessel.auto_pilot.auto_tune = True
throttle_limit = utils.throttle_from_twr(vessel_mass(), s_g, vessel_max_thrust(), 1.5)
vessel.control.throttle = throttle_limit
telem_viz.publish_gauge_metric('throttle', throttle_limit, False)
vessel.control.sas = True
vessel.control.rcs = True
vessel, telem = utils.check_active_vehicle(conn, vessel,
                                           mission_params.root_vessel)
telem_viz.start_telem_publish(telem)
vessel.control.toggle_action_group(1)
vessel.auto_pilot.target_pitch_and_heading(87.0, mission_params.target_heading)
vessel.auto_pilot.wait()
vessel.control.gear = False

# Wait until target alititude is achieved
dt = 0.5
enter_control_mode(DcxControlMode.BURN_TO_ALTITUDE, telem_viz)
expected_frame_time = (1/dcx.CLOCK_RATE) * 1e9
def dynamic_sleep(actual_frame_time: float):
    if actual_frame_time > expected_frame_time:
        if telem_viz.gnc_debug:
            print(f"Overrun! Time={actual_frame_time:.2f} seconds")
        telem_viz.increment_counter_metric('gnc_overrun_count')
    else:
        to_sleep = expected_frame_time - actual_frame_time
        time.sleep(to_sleep / 1e9)
while True:
    #predict future apoapsis
    frame_start = time.time_ns()
    telem_viz.increment_counter_metric('gnc_frame_count')
    h_future = vessel.flight().mean_altitude
    v_future = telem.vertical_vel() # np.linalg.norm(np.array(vessel.flight().velocity))
    for _ in range(int(vessel.orbit.time_to_apoapsis/dt)):
        h_old = h_future
        h_future, v_future = euler_step(vessel, h_future, v_future, dt)
        if h_future < h_old:
            break
        if h_future > target_alt:
            print("BREAK")
            break

    if h_future > target_alt:
        vessel.control.throttle = 0.0
        telem_viz.publish_gauge_metric('throttle', 0.0, False)
        enter_control_mode(DcxControlMode.COAST_TO_ALTITUDE, telem_viz)
        break
    else:
        frame_end = time.time_ns()
        dynamic_sleep(frame_end - frame_start)

    # tweak = 0.90 # fudge factor for strength of drag correction, probably somewhere in the 0.9 - 1.0 range is most accurate
    # scale_h = 5600.0 * vessel.orbit.body.atmosphere_depth / 70000.0 # rough atmosphere 1/e scale height in m
    # drag = vessel.flight().drag
    # d_loss = tweak * scale_h * sqrt(drag[0]*drag[0] + drag[1]*drag[1] + drag[2]*drag[2]) # energy loss to drag in J
    # h_future = 1.0/(1.0/vessel.orbit.apoapsis + d_loss/(vessel.orbit.body.gravitational_parameter * vessel.mass)) - vessel.orbit.body.equatorial_radius

    # print(h_future)

while telem.vertical_vel() > 2:
    telem_viz.increment_counter_metric('gnc_frame_count')
    time.sleep(10/dcx.CLOCK_RATE)
    pass

enter_control_mode(DcxControlMode.VERTICAL_HOLD, telem_viz)
print("Passed %i, entering vertical velocity hold ..." % target_alt)
new_throttle_limit = utils.throttle_from_twr(vessel_mass(), s_g, vessel_max_thrust(), 0.0)
vessel.control.throttle = new_throttle_limit
telem_viz.publish_gauge_metric('throttle', new_throttle_limit, False)
# Tu = 300
# Ku = 2.5
# Kp = 0.2*Ku
# Ki = 2.0*Kp/Tu
# Kd = 2.0*Kp/Tu
vert_vel_controller = controllers.PID(0.0, 0.25, 0.01, 0, new_throttle_limit, 1.0)
vert_vel_controller.set_point = -0.02  # vertical velocity target, m/s
alt_controller = controllers.PID(0.0, 0.5, 0.05, 0.05, min_output=-20, max_output=10, velocity_form=False)
alt_controller.set_point = telem.apoapsis()
slam_controller = controllers.PID(0.0, 1, 0.1, 0.0, 0.0, 1.0)
slam_controller.set_point = 0.3
roll_controller = controllers.PID(0.0, 0.5, 0.01, 0.0,-20, 20, 1.0)
dist_controller = controllers.PID(0.0, 0.09, 0.0, 0.01, -250.0, 250.0, velocity_form=False)
hvel_controller = controllers.PID(0.0, 0.3, 0.3, 0.1, -40.0, 60.0)
pad_targeting_pid = controllers.CascadeController(dist_controller, hvel_controller, True)

for thruster in vessel.parts.rcs:
    thruster.enabled = True

enter_control_mode(DcxControlMode.MODE_1, telem_viz)
mode = 1
status = vessel.situation.landed
starting_time = time.time()
throttle_update = starting_time
burn_flag = False
burn_start_flag = False
throttle_datastream = pu.data_stream_plot()
altitude_datastream = pu.data_stream_plot()
vertvel_datastream = pu.data_stream_plot()
distance_to_pad = 10000
prev_dist = 0
vel_sign = -1
expected_frame_time = (1/dcx.CLOCK_RATE) * 1e9
def dynamic_sleep(actual_frame_time: float):
    if actual_frame_time > expected_frame_time:
        if telem_viz.gnc_debug:
            print(f"Overrun! Time={actual_frame_time:.2f} seconds")
        telem_viz.increment_counter_metric('gnc_overrun_count')
    else:
        to_sleep = expected_frame_time - actual_frame_time
        time.sleep(to_sleep / 1e9)

# performance optimization: calculate static values only once to avoid many RPC calls to KSP game
g = vessel.orbit.body.gravitational_parameter / (vessel.orbit.body.equatorial_radius * vessel.orbit.body.equatorial_radius)

# start async, queue based metrics publisher. this avoids blocking calls to publish metrics from impacting GNC performance
metric_queue = Queue()
publisher_thread = threading.Thread(target=telem_viz.metric_publisher, args=(metric_queue,), daemon=True)
publisher_thread.start()

# performance optimization: setup streams for vehicle telemetry to avoid lots of RPC calls when reading the latest values
vessel_flight = vessel.flight()
vessel_latitude = conn.add_stream(getattr, vessel_flight, 'latitude')
vessel_longitude = conn.add_stream(getattr, vessel_flight, 'longitude')
vessel_heading = conn.add_stream(getattr, vessel_flight, 'heading')
vessel_roll = conn.add_stream(getattr, vessel_flight, 'roll')
vessel_surface_altitude = conn.add_stream(getattr, vessel_flight, 'surface_altitude')

vessel_flight_orbit_body_refframe = vessel.flight(vessel.orbit.body.reference_frame)
horizontal_speed_orbit_body_refframe = conn.add_stream(getattr, vessel_flight_orbit_body_refframe, 'horizontal_speed')

vessel_surface_ref = vessel.flight(ref_frame)
vertical_speed_surface_refframe  = conn.add_stream(getattr, vessel_surface_ref, 'vertical_speed')
horizontal_speed_surface_refframe  = conn.add_stream(getattr, vessel_surface_ref, 'horizontal_speed')
velocity_surface_refframe  = conn.add_stream(getattr, vessel_surface_ref, 'velocity')
heading_surface_refframe  = conn.add_stream(getattr, vessel_surface_ref, 'heading')

srf_frame = vessel.orbit.body.reference_frame
vessel_position_srf = conn.add_stream(vessel.position, srf_frame)
vessel_velocity_srf = conn.add_stream(vessel.velocity, srf_frame)
vessel_speed_srf  = conn.add_stream(getattr, vessel.flight(srf_frame), 'speed')

while vessel.situation != status:
    telem_viz.increment_counter_metric('gnc_frame_count')
    frame_start_time = time.time_ns()
    elapsed_time = time.time() - starting_time

    # Mode 1 is hover at target altitude
    if mode == 1:
        enter_control_mode(DcxControlMode.MODE_1, telem_viz, False)
        #vessel.control.throttle = vert_vel_controller.update(telem.vertical_vel())
        vert_vel_setpoint = alt_controller.update(telem.surface_altitude())
        vert_vel_controller.set_point = vert_vel_setpoint
        vessel.control.throttle = vert_vel_controller.update(telem.vertical_vel())
        telem_viz.publish_gauge_metric('throttle', vessel.control.throttle, False)
        if int(elapsed_time) > 1:
            vessel.control.throttle = 0.0
            metric_queue.put({'name': 'throttle', 'type': 'gauge', 'value': vessel.control.throttle})
            enter_control_mode(DcxControlMode.MODE_2, telem_viz)
            mode = 2
            vessel.auto_pilot.stopping_time = (0.3, 0.3, 0.3)
            vessel.control.toggle_action_group(1)
            vessel.auto_pilot.target_pitch = -10.0
            vessel.auto_pilot.target_roll = 45.0
            print("Exiting altitude hold ...")
            while telem.vertical_vel() > -10:
                telem_viz.increment_counter_metric('gnc_frame_count')
                dynamic_sleep(0)
                pass
            vessel.auto_pilot.stopping_time = (0.5, 0.3, 0.5)
    
    # Mode 2 is descent and landing burn start calculation
    if mode == 2:
        with telem_viz.get_histogram_metric('calculate_landing_burn_time').time():
            engine_offset = \
              (vessel.parts.with_name(vessel.parts.engines[0].part.name)[0].position(vessel.reference_frame))[1]
            tb = steering.calculate_landing_burn_time(
                vessel_position_srf(), vessel_velocity_srf(), vessel_mass(), vessel_speed_srf(),
                vessel_surface_altitude(), vessel_available_thrust(),
                telem_viz, g, engine_offset)
        if telem.surface_altitude() < 1500 or distance_to_pad < 50 or (tb < 0.5 and telem.surface_altitude() < 3000):
            enter_control_mode(DcxControlMode.MODE_3, telem_viz, False)
            mode = 3
            pad_targeting_pid.set_max_output(20)
            pad_targeting_pid.set_min_output(-20)
            pad_targeting_pid.reset()

        # Compute line of sight angle to landing pad
        current_position = (vessel_latitude(), vessel_longitude())
        with telem_viz.get_histogram_metric('calculate_heading').time():
            heading = compass_heading(np.degrees(corrected_compute_los_angle(current_position, landing_site)))

        # Compute pitch parameter inputs
        distance_to_pad = haversine_distance(current_position[0], current_position[1], landing_site[0], landing_site[1])
        current_horizontal_velocity = vel_sign*horizontal_speed_orbit_body_refframe()
        
        # Apply velocity sign based on whether moving toward/away from pad
        if distance_to_pad < prev_dist:
            current_horizontal_velocity *= -1  # Moving away from pad
        prev_dist = distance_to_pad

        pitch_input = pad_targeting_pid.update(distance_to_pad, current_horizontal_velocity)

        if pitch_input > 0:
            vessel.control.brakes = True
        else:
            vessel.control.brakes = False

        # Update control input
        vessel.auto_pilot.target_heading = heading
        if telem.horizontal_vel() > 50 and distance_to_pad < 1500:
            vessel.auto_pilot.target_pitch = pitch_input
        else:
            vessel.auto_pilot.target_pitch = pitch_input

        # Roll control implementation
        current_heading = vessel.flight().heading
        heading_error = steering.compute_heading_error(current_heading, heading)
        roll_input = -1*roll_controller.update(heading_error)
        if telem.surface_altitude() > 8000:
            vessel.auto_pilot.target_roll = 45.0
        else:
            vessel.auto_pilot.target_roll = 0.0

        if telem_viz.gnc_debug:
            print(f"Mode 2 Outputs: {distance_to_pad:.2f}, {current_horizontal_velocity:.2f}, {pitch_input:.2f}, {tb:.2f}")
        with telem_viz.get_histogram_metric('publish_gnc_metrics_b').time():
            metric_queue.put({'name':'distance_to_pad', 'type': 'gauge', 'value': distance_to_pad})
            metric_queue.put({'name':'current_horizontal_velocity', 'type': 'gauge', 'value': current_horizontal_velocity})
            metric_queue.put({'name':'pitch_input', 'type': 'gauge', 'value': pitch_input})
            metric_queue.put({'name':'time_to_burn', 'type': 'gauge', 'value': tb})
            metric_queue.put({'name': 'heading_error', 'type': 'gauge', 'value': heading_error})
            metric_queue.put({'name': 'heading', 'type': 'gauge', 'value': heading})
            metric_queue.put({'name': 'roll', 'type': 'gauge', 'value': vessel_roll()})
            metric_queue.put({'name': 'roll_input', 'type': 'gauge', 'value': roll_input})

    if mode == 3:
        with telem_viz.get_histogram_metric('calculate_landing_burn_time').time():
            engine_offset = \
                (vessel.parts.with_name(vessel.parts.engines[0].part.name)[0].position(vessel.reference_frame))[1]
            tb = steering.calculate_landing_burn_time(
                vessel_position_srf(), vessel_velocity_srf(), vessel_mass(), vessel_speed_srf(),
                vessel_surface_altitude(), vessel_available_thrust(),
                telem_viz, g, engine_offset)

        for thruster in vessel.parts.rcs:
            thruster.enabled = True

        # Compute line of sight angle to landing pad
        current_position = (vessel_latitude(), vessel_longitude())
        heading = compass_heading(np.degrees(corrected_compute_los_angle(current_position, landing_site)))

        # Compute pitch parameter inputs
        distance_to_pad = haversine_distance(current_position[0], current_position[1], landing_site[0], landing_site[1])
        current_horizontal_velocity = vel_sign*horizontal_speed_orbit_body_refframe()
        
        # Apply velocity sign based on whether moving toward/away from pad
        if distance_to_pad < prev_dist:
            current_horizontal_velocity *= -1  # Moving away from pad
        prev_dist = distance_to_pad

        # Update cascade controller
        pitch_input = pad_targeting_pid.update(distance_to_pad, current_horizontal_velocity)

        # Update heading based on engine thrust status
        if burn_flag is True and telem.velocity() > 120:
            heading += 180
        elif burn_flag is False:
            heading += 180
            
        if heading > 360:
                heading -= 360

        vessel.auto_pilot.target_heading = heading
        vessel.auto_pilot.target_pitch = 90+pitch_input
        vessel.auto_pilot.target_roll = float('NaN')
        current_heading = vessel_heading()
        heading_error = steering.compute_heading_error(current_heading, heading)

        if telem_viz.gnc_debug:
            print(f"Mode 3 Outputs: {distance_to_pad:.2f}, {current_horizontal_velocity:.2f}, {pitch_input:.2f}, {telem.velocity():.2f}")
        with telem_viz.get_histogram_metric('publish_gnc_metrics_b').time():
            metric_queue.put({'name':'distance_to_pad', 'type': 'gauge', 'value': distance_to_pad})
            metric_queue.put({'name':'current_horizontal_velocity', 'type': 'gauge', 'value': current_horizontal_velocity})
            metric_queue.put({'name':'pitch_input', 'type': 'gauge', 'value': pitch_input})
            metric_queue.put({'name':'heading', 'type': 'gauge', 'value': heading})
            metric_queue.put({'name':'heading_error', 'type': 'gauge', 'value': heading_error})
            # note velocity already published by background thread

        if (tb < 0.5 or telem.surface_altitude() < 1500) and burn_flag is False:
            vessel.control.throttle = slam_controller.update(tb)
            burn_flag = True
            vessel.auto_pilot.stopping_time = (1.2, 0.2, 0.2)

        if telem.surface_altitude() >= 50 and burn_flag is True:
                vessel.control.throttle = slam_controller.update(tb)

        if telem.surface_altitude() < 120 or telem.vertical_vel() > -5:
            mode = 4
            enter_control_mode(DcxControlMode.MODE_4, telem_viz)
            # Tu = 225
            # Ku = 2.5
            # Kp = 0.2*Ku
            # Ki = 2.0*Kp/Tu
            # Kd = 0.0*Kp/Tu
            vert_vel_controller.set_gains(0.25, 0.1, 0)
            alt_controller.set_point = 10.0
            alt_controller.set_max_output(10.0)
            alt_controller.set_min_output(-10.0)
            hvel_controller.set_max_output(15.0)
            hvel_controller.set_min_output(-15.0)
            hvel_controller.set_gains(0.5, 0.5, 0.1)
            dist_controller.set_gains(0.1, 0.0, 0.0)

    # Mode 4 is null horizontal velocity and slew to pad
    if mode == 4:
        if vessel.resources.amount("LiquidFuel") < starting_LF*0.045:
            mode = 5
            enter_control_mode(DcxControlMode.MODE_5, telem_viz)
            vert_vel_controller.set_gains(0.25, 0.1, 0)
            alt_controller.set_point = -10.0
            alt_controller.set_max_output(-4.0)
            alt_controller.set_min_output(-10.0)
            hvel_controller.set_max_output(10.0)
            hvel_controller.set_min_output(-10.0)
            hvel_controller.set_gains(0.05, 0.15, 0.05)
        vert_vel_setpoint = alt_controller.update(telem.surface_altitude())
        vert_vel_controller.set_point = vert_vel_setpoint
        vessel.control.throttle = vert_vel_controller.update(telem.vertical_vel())
        # Compute line of sight angle to landing pad
        current_position = (vessel_latitude(), vessel_longitude())
        heading = compass_heading(np.degrees(corrected_compute_los_angle(current_position, landing_site)))

        # Compute pitch parameter inputs
        distance_to_pad = haversine_distance(current_position[0], current_position[1], landing_site[0], landing_site[1])

        if distance_to_pad > 75:
            current_horizontal_velocity = horizontal_speed_orbit_body_refframe()
            
            # Apply velocity sign based on whether moving toward/away from pad
            if distance_to_pad < prev_dist:
                current_horizontal_velocity *= -1  # Moving away from pad
            prev_dist = distance_to_pad

            # Update cascade controller
            pitch_input = pad_targeting_pid.update(distance_to_pad, current_horizontal_velocity)
            vessel.auto_pilot.target_heading = heading
            vessel.auto_pilot.target_pitch = 90+pitch_input
            vessel.auto_pilot.target_roll = float('NaN')

        else:
            # Get the vessel's velocity relative to the surface
            v_vec = velocity_surface_refframe()
            v_mag = np.linalg.norm(v_vec)

            if horizontal_speed_surface_refframe() > 1:
                # Calculate the direction to apply thrust
                retro_vec = -np.array(v_vec) / v_mag
                # Use the PID controller to adjust the pointing vector
                pitch_input = hvel_controller.update(horizontal_speed_surface_refframe())
                # Calculate the heading using atan2 and convert to 0-360 degrees
                heading_raw = degrees(atan2(retro_vec[2], retro_vec[1]))
                if heading_raw < 0:
                    heading = heading_raw + 360
                else:
                    heading = heading_raw
                
                # Set the target direction for the autopilot
                vessel.auto_pilot.target_pitch = 90+pitch_input
                vessel.auto_pilot.target_heading = heading
                vessel.auto_pilot.target_roll = float('NaN')
            else:
                pitch_input = 0
                heading = vessel_heading()
                vessel.auto_pilot.target_pitch = 90
                vessel.auto_pilot.target_heading = heading
                vessel.auto_pilot.target_roll = float('NaN')
                mode = 5
                enter_control_mode(DcxControlMode.MODE_5, telem_viz)
                vert_vel_controller.set_gains(0.25, 0.1, 0)
                alt_controller.set_point = -10.0
                alt_controller.set_max_output(-4.0)
                alt_controller.set_min_output(-10.0)
                hvel_controller.set_max_output(10.0)
                hvel_controller.set_min_output(-10.0)
                hvel_controller.set_gains(0.05, 0.15, 0.05)

        current_heading = vessel_heading()
        heading_error = steering.compute_heading_error(current_heading, heading)

        if telem_viz.gnc_debug:
            print(f"Mode 4 Outputs: {distance_to_pad:.2f}, {current_horizontal_velocity}, {pitch_input:.2f}")
        with telem_viz.get_histogram_metric('publish_gnc_metrics_b').time():
            metric_queue.put({'name':'distance_to_pad', 'type': 'gauge', 'value': distance_to_pad})
            metric_queue.put({'name':'current_horizontal_velocity', 'type': 'gauge', 'value': current_horizontal_velocity})
            metric_queue.put({'name':'pitch_input', 'type': 'gauge', 'value': pitch_input})
            metric_queue.put({'name': 'vert_vel_setpoint', 'type': 'gauge', 'value': vert_vel_controller.set_point})
            metric_queue.put({'name': 'alt_controller_setpoint', 'type': 'gauge', 'value': alt_controller.set_point})
            metric_queue.put({'name': 'heading', 'type': 'gauge', 'value': heading})
            metric_queue.put({'name': 'heading_error', 'type': 'gauge', 'value': heading_error})

    # constant rate of descent mode
    if mode == 5:
        vert_vel_setpoint = alt_controller.update(telem.surface_altitude())
        vert_vel_controller.set_setpoint(vert_vel_setpoint)
        vessel.control.throttle = vert_vel_controller.update(telem.vertical_vel())

        # Get the vessel's velocity relative to the surface
        v_vec = vessel.flight(ref_frame).velocity
        v_mag = np.linalg.norm(v_vec)

        if vessel.flight(ref_frame).horizontal_speed > 1:
            # Calculate the direction to apply thrust
            retro_vec = -np.array(v_vec) / v_mag
            # Use the PID controller to adjust the pointing vector
            pitch = hvel_controller.update(vessel.flight(ref_frame).horizontal_speed)
            # Calculate the heading using atan2 and convert to 0-360 degrees
            heading_raw = degrees(atan2(retro_vec[2], retro_vec[1]))
            if heading_raw < 0:
                heading = heading_raw + 360
            else:
                heading = heading_raw
            
            # Set the target direction for the autopilot
            vessel.auto_pilot.target_pitch = 90+pitch
            vessel.auto_pilot.target_heading = heading
            vessel.auto_pilot.target_roll = float('NaN')

        else:
            pitch = 0
            heading = vessel.flight(ref_frame).heading
            vessel.auto_pilot.target_pitch = 90
            vessel.auto_pilot.target_heading = heading
            vessel.auto_pilot.target_roll = float('NaN')

        if telem_viz.gnc_debug:
            print(f"Mode 5 Outputs: {distance_to_pad:.2f}, {telem.horizontal_vel():2f}, {pitch:.2f}, {vert_vel_setpoint:.2}, {alt_controller.set_point:.2f}")
        with telem_viz.get_histogram_metric('publish_gnc_metrics_b').time():
            metric_queue.put({'name':'distance_to_pad', 'type': 'gauge', 'value': distance_to_pad})
            metric_queue.put({'name': 'current_horizontal_velocity', 'type': 'gauge', 'value': telem.horizontal_vel()})
            metric_queue.put({'name':'pitch', 'type': 'gauge', 'value': pitch})
            metric_queue.put({'name':'vert_vel_setpoint', 'type': 'gauge', 'value': vert_vel_setpoint})
            metric_queue.put({'name':'alt_controller_setpoint', 'type': 'gauge', 'value': alt_controller.set_point})
            metric_queue.put({'name':'heading', 'type': 'gauge', 'value': heading})

    if telem.surface_altitude() < 30:
        vessel.control.gear = True

    # if int(time.time()) - int(throttle_update) > 5:
    #     if telem.surface_altitude()-target_alt < 50 and telem.vertical_vel() < -10:
    #         continue
    #         new_throttle_limit = utils.throttle_from_twr(vessel, 0.25)
    #     elif mode == 5:
    #         new_throttle_limit = utils.throttle_from_twr(vessel, 0.5)
    #         vert_vel_controller.set_max_output(utils.throttle_from_twr(vessel, 1.1))
    #     else:
    #         new_throttle_limit = utils.throttle_from_twr(vessel, 0.0)
        
    #     vert_vel_controller.set_min_output(new_throttle_limit)
    #     print("Current Machine State %i" % mode)
    #     throttle_update = time.time()

    # Plot states
    telem_viz.publish_gauge_metric('throttle', vessel.control.throttle, False)
    throttle_datastream.update_data_stream(elapsed_time, vessel.control.throttle)
    altitude_datastream.update_data_stream(elapsed_time, telem.altitude())
    vertvel_datastream.update_data_stream(elapsed_time, telem.vertical_vel())

    frame_end_time = time.time_ns()
    frame_time = frame_end_time - frame_start_time
    dynamic_sleep(frame_time)

enter_control_mode(DcxControlMode.LANDED, telem_viz)
vessel.control.throttle = 0.0
vessel.auto_pilot.disengage()
time.sleep(10/dcx.CLOCK_RATE)
vessel.control.rsc = False
time.sleep(10/dcx.CLOCK_RATE)
vessel.control.sas = True
# throttle_datastream.plot()
# altitude_datastream.plot()
# vertvel_datastream.plot()
# plt.show()
enter_control_mode(DcxControlMode.SHUTDOWN, telem_viz)
time.sleep(1)