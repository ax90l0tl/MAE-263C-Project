import pandas as pd
import numpy as np
import scipy
import matplotlib.pyplot as plt

motor_numbers = [115, 116, 121]
trial_numbers = [0, 1, 2]

linear_velocity = 15  # range that we consider linear
velocity_threshold = 0.5  # threshold for velocity to be considered zero
negative_speeds = np.arange(-linear_velocity, 0, 0.1)
positive_speeds = np.arange(0, linear_velocity, 0.1)

coefficients = np.zeros((len(motor_numbers) * len(trial_numbers), 4))  # to store coefficients for each motor per trial

fig, axs = plt.subplots(3, figsize=(10, 15))  # Create subplots
fig.subplots_adjust(hspace=0.5)  # Adjust space between subplots
for ax in axs:
    ax.grid(True)
# Columns: [negative_slope, negative_intercept, positive_slope, positive_intercept]
average_coefficients = np.zeros((len(motor_numbers), 4))  # to store average coefficients for each motor
for i in range(len(motor_numbers)):
    for j in range(len(trial_numbers)):
        name = 'friction_id_KM' + str(motor_numbers[i]) + '_' + str(trial_numbers[j]) + '.csv'
        dir = r"C:\Users\c_mku\Documents\MAE263C\MAE-263C-Project"
        filepath = dir + "\\" + name
        df = pd.read_csv(filepath)
        print(f"Processing file: {name}")
        df_cleaned = df[(df['velocity'] >= -linear_velocity) & (df['velocity'] <= linear_velocity)]
        df_negative = df_cleaned[df_cleaned['velocity'] < -velocity_threshold]
        df_positive = df_cleaned[df_cleaned['velocity'] > velocity_threshold]
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




