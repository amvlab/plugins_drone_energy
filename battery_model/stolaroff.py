# model from DOI: 10.1038/s41467-017-02411-5

# Energy use and life cycle greenhouse gas emissions of drones for commercial package delivery

import json
import os
import numpy as np
import pandas as pd
import itertools
import matplotlib.pyplot as plt

# Load configuration from JSON
config_path = os.path.join(os.path.dirname(__file__), 'config.json')
with open(config_path, 'r') as f:
    cfg = json.load(f)

airspeeds = np.arange(1, 30.5, 0.5)
drone_masses = np.arange(1, 10.5, 0.5)
payload_masses = np.arange(0, 10.5, 0.5)
battery_masses = np.arange(1, 10.5, 0.5)

# Test

drone_masses = [7]
battery_masses = [10]
payload_masses = [7]

config_combinations = list(itertools.product(*[airspeeds, drone_masses, payload_masses, battery_masses]))


# inputs
rho0 = 1.225
g0 = 9.807

drone_mass = 7
battery_mass = 10
payload_mass = 7

drone_Cd = 1.49
battery_Cd = 1
payload_Cd = 2.2

drone_area = 0.224
battery_area = 0.015
payload_area = 0.0929

n_rotors = 8
rotor_diameter       = 0.432



def stolaroff_vi(force_drag, force_grav, angle_of_attack, tas, n_rotors, chord_diameter, max_iter=1000, tol=0.001):

    # Calculate induced airspeed numerically
    vi = 0
    exit_condition = True
    iteration = 0

    while exit_condition:

        # Calculate vi
        trig_term = np.sqrt(np.square(tas * np.cos(angle_of_attack)) + np.square(tas*np.sin(angle_of_attack) + vi))
        vi_new = 2*(force_grav+force_drag) / (np.pi*np.square(chord_diameter)*n_rotors*cfg['environment']['rho0']*trig_term) 


        # Exit condition if difference is lower than tolerance
        if abs(vi_new - vi) < tol: 
            exit_condition = False 

        elif iteration > max_iter:
            print(f'Could not converge')
            print(f'Airspeed {tas}')
            print('----------------')
            exit_condition = False

        # assign vi to vi_new to restart the loop
        vi = vi_new

        # step iteration
        iteration+=1

    return vi

# Make the dataframe
cols = ['airspeed', 'drone_mass', 'payload_mass', 'battery_mass']
df = pd.DataFrame(config_combinations, columns=cols)

# lists with payload
induced_speeds_1 = list()
thrusts_1 = list()
epms_1 = list()
flight_ranges_1 = list()
power_expended_1 = list()


# lists without payload
induced_speeds_2 = list()
thrusts_2 = list()
epms_2 = list()
flight_ranges_2 = list()
power_expended_2 = list()

flight_range_4 =list()

for row in df.itertuples():

    # Get values for payload

    # calculate drag and weight
    force_drag = 0.5*rho0*(drone_area*drone_Cd + battery_area*battery_Cd + payload_area*payload_Cd)*np.square(row.airspeed)
    force_grav = g0*(drone_mass + battery_mass + payload_mass)
    
    angle_of_attack = np.arctan((force_drag/force_grav))

    vi = stolaroff_vi(
                    force_drag, 
                    force_grav, 
                    angle_of_attack, 
                    row.airspeed, 
                    n_rotors, 
                    rotor_diameter, 
                    max_iter=100000, 
                    tol=0.00001
                )


    # induced speed
    induced_speeds_1.append(vi)

    # Now calculate thrust
    thrust = force_drag + force_grav
    thrusts_1.append(thrust)

    # calcualte power
    power_exp = thrust* ((row.airspeed*np.sin(angle_of_attack) + vi) / (cfg['battery']['batt_eff']))
    epm_a = power_exp / (row.airspeed)
    epms_1.append(epm_a)
    power_expended_1.append(power_exp)

    # range
    batt_energy = cfg['battery']['batt_senergy'] *  battery_mass  # J
    flight_range = (0.25/1.2)*(batt_energy / epm_a)/1000
    flight_ranges_1.append(flight_range)

    # Get values without
    
    # calculate drag and weight
    force_drag = 0.5*rho0*(drone_area*drone_Cd + battery_area*battery_Cd)*np.square(row.airspeed)
    force_grav = g0*(drone_mass + battery_mass )
    
    angle_of_attack = np.arctan((force_drag/force_grav))

    vi = stolaroff_vi(
                    force_drag, 
                    force_grav, 
                    angle_of_attack, 
                    row.airspeed, 
                    n_rotors, 
                    rotor_diameter, 
                    max_iter=100000, 
                    tol=0.00001
                )
    # induced speed
    induced_speeds_2.append(vi)

    # Now calculate thrust
    thrust = force_drag + force_grav
    thrusts_2.append(thrust)

    # The last thing to calculate is energy consumption
    power_exp = thrust* ((row.airspeed*np.sin(angle_of_attack) + vi) / (cfg['battery']['batt_eff']))
    epm_b = power_exp / (row.airspeed)
    epms_2.append(epm_b)
    power_expended_2.append(power_exp)

    # range
    batt_energy = cfg['battery']['batt_senergy'] *  battery_mass   # J
    flight_range = (0.25/1.2)*(batt_energy / epm_b)/1000
    flight_ranges_2.append(flight_range)

    # range
    flight_range = (0.5/1.2)*(batt_energy / (epm_a+epm_b))/1000
    flight_range_4.append(flight_range)

    
# Add the new columns
df['induced_speed_payload'] = induced_speeds_1
df['thrust_payload'] = thrusts_1
df['epm_payload'] = epms_1
df['flight_range_payload'] = flight_ranges_1
df['power_payload'] = power_expended_1

# Add the new columns
df['induced_speed_no_payload'] = induced_speeds_2
df['thrust_no_payload'] = thrusts_2
df['epm_no_payload'] = epms_2
df['flight_range_no_payload'] = flight_ranges_2
df['power_no_payload'] = power_expended_2

df['avg_range'] = flight_range_4

# calculate averages
df['epm_3'] = (df['epm_payload'] + df['epm_no_payload'])*0.5


# save as dataframe
output_dir = os.path.join(os.path.dirname(__file__), '..', 'plugins', 'drone_performance_data')
os.makedirs(output_dir, exist_ok=True)
output_path = os.path.join(output_dir, 'energy.csv')
df.to_csv(output_path, index=False)


# load csv
#df_1 = pd.read_csv('energy.csv')
#print(df_1)

# Plot
ax = df.plot(x='airspeed', y='epm_3', label='loaded')
#df.plot(ax=ax, x='airspeed', y='epm_no_payload', label='unloaded')

ax.set_ylim([0,1000])
ax.set_xlim([0,25])
ax.set_ylabel('Energy/distance (J/m)')
ax.set_xlabel('airspeed (m/s)')

plt.show()

# # thrusts
# df['epm_2'] = epms
# df['flight_range_2'] = flight_ranges

# df['epm_3'] = (df['epm'] + df['epm_2'])*0.5


# Plot
# df.plot(x='airspeed', y='epm_2', ax=ax)
ax = df.plot(x='airspeed', y='flight_range_no_payload')



ax.set_ylim([0,12])
ax.set_xlim([0,25])
plt.show()


ax = df.plot(x='airspeed', y='flight_range_payload')



ax.set_ylim([0,12])
ax.set_xlim([0,25])
plt.show()

ax = df.plot(x='airspeed', y='avg_range')



ax.set_ylim([0,12])
ax.set_xlim([0,25])
plt.show()