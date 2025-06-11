import pandas as pd
import numpy as np
import scipy
import matplotlib.pyplot as plt

motor_numbers = [115, 116]
trial_numbers = [0, 1]

linear_velocity = 15  # range that we consider linear
velocity_threshold = 0.5  # threshold for velocity to be considered zero
negative_speeds = np.arange(-linear_velocity, 0, 0.1)
positive_speeds = np.arange(0, linear_velocity, 0.1)

coefficients = np.zeros((len(motor_numbers) * len(trial_numbers), 4))  # to store coefficients for each motor per trial

fig, axs = plt.subplots(2, figsize=(10, 20))  # Create subplots
fig.subplots_adjust(hspace=0.5)  # Adjust space between subplots
for ax in axs:
    ax.grid(True)

# Columns: [negative_slope, negative_intercept, positive_slope, positive_intercept]
average_coefficients = np.zeros((len(motor_numbers), 4))  # to store average coefficients for each motor

for i in range(len(motor_numbers)):
    # NEW: per-motor temp storage
    nonzero_vel_mins = []

    for j in range(len(trial_numbers)):
        name = 'friction_id_KM' + str(motor_numbers[i]) + '_' + str(trial_numbers[j]) + '.csv'
        df = pd.read_csv(name)
        print(f"Processing file: {name}")
        df_cleaned = df[(df['velocity'] >= -linear_velocity) & (df['velocity'] <= linear_velocity)]
        df_negative = df_cleaned[df_cleaned['velocity'] < -velocity_threshold]
        df_positive = df_cleaned[df_cleaned['velocity'] > velocity_threshold]

        # Print largest velocity in df_negative and smallest in df_positive with their torque values
        if not df_negative.empty:
            max_neg_idx = df_negative['velocity'].idxmax()
            max_neg_velocity = df_negative.loc[max_neg_idx, 'velocity']
            max_neg_torque = df_negative.loc[max_neg_idx, 'torque']
            nonzero_vel_mins.append((max_neg_velocity, max_neg_torque))  # Store the absolute value of the max negative velocity
            # print(f"Largest velocity in df_negative: {max_neg_velocity:.4f}, torque: {max_neg_torque:.4f}")
        else:
            print("df_negative is empty.")

        if not df_positive.empty:
            min_pos_idx = df_positive['velocity'].idxmin()
            min_pos_velocity = df_positive.loc[min_pos_idx, 'velocity']
            min_pos_torque = df_positive.loc[min_pos_idx, 'torque']
            nonzero_vel_mins.append((min_pos_velocity, min_pos_torque))  # Store the absolute value of the min positive velocity
            # print(f"Smallest velocity in df_positive: {min_pos_velocity:.4f}, torque: {min_pos_torque:.4f}")
        else:
            print("df_positive is empty.")

        # Only perform regression if there are at least 2 points
        if len(df_negative) >= 2:
            negative_friction = scipy.stats.linregress(df_negative['velocity'], df_negative['torque'])
            neg_slope = negative_friction.slope
            neg_intercept = negative_friction.intercept
        else:
            neg_slope = np.nan
            neg_intercept = np.nan

        if len(df_positive) >= 2:
            positive_friction = scipy.stats.linregress(df_positive['velocity'], df_positive['torque'])
            pos_slope = positive_friction.slope
            pos_intercept = positive_friction.intercept
        else:
            pos_slope = np.nan
            pos_intercept = np.nan

        coefficients[3 * i + j, :] = [neg_slope, neg_intercept, pos_slope, pos_intercept]
        axs[i].plot(df['velocity'], df['torque'], 'bo', label='Motor' + str(motor_numbers[i]) + ' Trial ' + str(trial_numbers[j]))   

    # Average the coefficients across trials for each motor
    average_coefficients[i,:] = np.nanmean(coefficients[3*i:3*i+len(trial_numbers)], axis=0)

    axs[i].plot(negative_speeds, average_coefficients[i,0] * negative_speeds + average_coefficients[i,1], 'r-', label=('Neg Viscous Friction:' + str(average_coefficients[i,0])+ ' N-m/rad-s  Neg Stiction:' + str(average_coefficients[i,1]) + 'N-m'), linewidth=3.5)
    axs[i].plot(positive_speeds, average_coefficients[i,2] * positive_speeds + average_coefficients[i,3], 'g-', label=('Pos Viscous Friction:' + str(average_coefficients[i,2])+ 'vN-m/rad-s  Pos Stiction:' + str(average_coefficients[i,3]) + 'N-m'), linewidth=3.5)
    axs[i].set_xlabel('Velocity (rad/s)')
    axs[i].set_ylabel('Torque (N-m)')
    axs[i].set_title(f'KM{motor_numbers[i]} \nm_neg:{average_coefficients[i,0]:.3f} N-m/rad-s, b_neg:{average_coefficients[i,1]:.3f} N-m, m_pos:{average_coefficients[i,2]:.3f} N-m/rad-s, b_pos:{average_coefficients[i,3]:.3f} N-m')

    # NEW: Calculate a slope for near zero case with the smallest non-zero velocity
    min_pair = min(nonzero_vel_mins, key=lambda x: abs(x[0]))
    slope = (abs(min_pair[1])) / (abs(min_pair[0]))  # slope near zero

    # Plot a solid orange line between -min_vel and min_vel with slope slopes_near_zero[i]
    x_vals = np.array([-abs(min_pair[0]), abs(min_pair[0])])
    y_vals = slope * x_vals
    axs[i].plot(x_vals, y_vals, color='orange', linewidth=3, label='Slope near zero')

    # NEW: Print summary for each motor
    print(f"Motor KM{motor_numbers[i]}:")
    print(f"Smallest non-zero velocity: {min_pair[0]:.4f} rad/s, Torque: {min_pair[1]:.4f} N-m")
    print(f"Slope near zero: {slope:.4f} N-m/rad/s")

plt.show()


