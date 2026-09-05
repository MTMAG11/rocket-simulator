import matplotlib.pyplot as plt

def plot_results(times, altitudes, velocities, accelerations, thrusts, twrs):
    # Create subplots
    fig, axs = plt.subplots(3, 2, figsize=(12, 10))
    fig.suptitle('Rocket Simulation Results', fontsize=16)

    # Plot Altitude vs Time
    axs[0, 0].plot(times, altitudes, color='blue')
    axs[0, 0].set_title('Altitude vs Time')
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel('Altitude (m)')
    axs[0, 0].grid()

    # Plot Velocity vs Time
    axs[0, 1].plot(times, velocities, color='green')
    axs[0, 1].set_title('Velocity vs Time')
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Velocity (m/s)')
    axs[0, 1].grid()

    # Plot Acceleration vs Time
    axs[1, 0].plot(times, accelerations, color='red')
    axs[1, 0].set_title('Acceleration vs Time')
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_ylabel('Acceleration (m/s²)')
    axs[1, 0].grid()

    # Plot Thrust vs Time
    axs[1, 1].plot(times, thrusts, color='orange')
    axs[1, 1].set_title('Thrust vs Time')
    axs[1, 1].set_xlabel('Time (s)')
    axs[1, 1].set_ylabel('Thrust (N)')
    axs[1, 1].grid()

    # Plot TWR vs Time
    axs[2, 0].plot(times, twrs, color='purple')
    axs[2, 0].set_title('Thrust-to-Weight Ratio vs Time')
    axs[2, 0].set_xlabel('Time (s)')
    axs[2, 0].set_ylabel('TWR')
    axs[2, 0].grid()

    # Hide the empty subplot
    fig.delaxes(axs[2][1])

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()