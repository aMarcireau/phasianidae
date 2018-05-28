import krpc
import math
import scipy.interpolate
import time

def clamp(minimum, value, maximum):
    if minimum > value:
        return minimum
    if value > maximum:
        return maximum
    return value

def launch(connection, craft_directory, name, launch_site, target_altitude, pitch_profile, undershoot, precedence, precedence_hysteresis, apoapsis_k, eccentricity_k):
    print('craft_directory: {}\nname: {}\nlaunch_site: {}\ntarget_altitude: {} m\npitch_profile: {}\nundershoot: {} m\nprecedence: {} s\nprecedence hysteresis: {}s\napoapsis_k: {} m^-1\neccentricity_k: {}'.format(craft_directory, name, launch_site, target_altitude, pitch_profile, undershoot, precedence, precedence_hysteresis, apoapsis_k, eccentricity_k))
    connection.space_center.launch_vessel(craft_directory, name, launch_site)
    vessel = connection.space_center.active_vessel
    flight = vessel.flight()
    vessel.auto_pilot.target_pitch_and_heading(90, 90)
    vessel.auto_pilot.engage()
    vessel.control.throttle = 1

    # launch
    vessel.control.activate_next_stage()
    print('stage')

    phase = 0
    thrust = vessel.available_thrust
    eccentricity = 1
    while True:

        # manage stagging
        new_thrust = vessel.available_thrust
        if abs(new_thrust - thrust) / thrust > 0.1:
            print('stage')
            vessel.control.activate_next_stage()
            thrust = vessel.available_thrust
        else:
            thrust = new_thrust

        # manage thrust and throttle
        altitude = flight.mean_altitude
        if phase == 0:
            if altitude > 70000:
                vessel.auto_pilot.target_pitch = 0
                vessel.auto_pilot.throttle = 0
                phase = 1
            else:
                vessel.auto_pilot.target_pitch = clamp(
                    0,
                    sum(coefficient * altitude ** index for index, coefficient in enumerate(pitch_profile)),
                    90)
                vessel.control.throttle = clamp(
                    0,
                    (target_altitude - undershoot - vessel.orbit.apoapsis_altitude) * apoapsis_k,
                    1)
        elif phase == 1:
            if vessel.orbit.time_to_apoapsis < precedence:
                eccentricity = vessel.orbit.eccentricity
                vessel.control.throttle = 1
                phase = 2
        elif phase == 2:
            if vessel.orbit.time_to_apoapsis > precedence:
                vessel.control.throttle = 0
                phase = 3
            else:
                new_eccentricity = vessel.orbit.eccentricity
                if new_eccentricity < 0.001 or new_eccentricity > eccentricity:
                    vessel.control.throttle = 0
                    break
                else:
                    eccentricity = new_eccentricity
                    vessel.control.throttle = clamp(0, eccentricity * eccentricity_k, 1)
        elif phase == 3:
            connection.space_center.warp_to(connection.space_center.ut + vessel.orbit.time_to_apoapsis - (precedence - precedence_hysteresis))
            eccentricity = vessel.orbit.eccentricity
            vessel.control.throttle = clamp(0, eccentricity * eccentricity_k, 1)
            phase = 4
        elif phase == 4:
            if vessel.orbit.time_to_apoapsis > precedence:
                vessel.control.throttle = 0
                phase = 3
            else:
                new_eccentricity = vessel.orbit.eccentricity
                if new_eccentricity < 0.001 or new_eccentricity > eccentricity:
                    vessel.control.throttle = 0
                    break
                else:
                    eccentricity = new_eccentricity
                    vessel.control.throttle = clamp(0, eccentricity * eccentricity_k, 1)
        print('phase {}, altitude: {} m, apoapsis: {} m, eccentricity: {}, pitch: {}, throttle: {}'.format(phase, altitude, vessel.orbit.apoapsis_altitude, vessel.orbit.eccentricity, vessel.auto_pilot.target_pitch, vessel.control.throttle))
        time.sleep(0.1)
    vessel.auto_pilot.disengage()

launch(
    connection=krpc.connect(),
    craft_directory='VAB',
    name='tarsier',
    launch_site='LaunchPad',
    target_altitude=80000,
    pitch_profile=list(reversed(scipy.interpolate.lagrange([0, 20000, 70000], [90, 45, 0]).coefficients.tolist())),
    undershoot=2000,
    precedence=60,
    precedence_hysteresis=30,
    apoapsis_k=1.0 / 10000,
    eccentricity_k=1.0 / 0.1)
