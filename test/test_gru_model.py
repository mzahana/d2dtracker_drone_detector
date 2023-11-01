import numpy as np
import torch
import torch.nn as nn
import matplotlib.pyplot as plt
import sys
import time

# Check if GPU is available
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Trajectory Predictor Model Definition
class TrajectoryPredictor(nn.Module):
    def __init__(self, input_dim, hidden_dim, output_dim, num_layers):
        super(TrajectoryPredictor, self).__init__()

        self.gru1 = nn.GRU(input_dim, hidden_dim, num_layers, batch_first=True)
        self.gru2 = nn.GRU(hidden_dim, hidden_dim, num_layers, batch_first=True)
        self.fc = nn.Linear(hidden_dim, output_dim)

    def forward(self, x):
        out, h_n = self.gru1(x)
        dec_input = torch.zeros(x.size(0), 10, hidden_dim).to(x.device)
        out, _ = self.gru2(dec_input, h_n)
        out = self.fc(out)
        return out

# Velocity Predictor Model Definition
class VelocityPredictor(nn.Module):
    def __init__(self, input_dim, hidden_dim, output_dim, num_layers):
        super(VelocityPredictor, self).__init__()

        self.gru1 = nn.GRU(input_dim, hidden_dim, num_layers, batch_first=True)
        self.gru2 = nn.GRU(hidden_dim, hidden_dim, num_layers, batch_first=True)
        self.fc = nn.Linear(hidden_dim, output_dim)

    def forward(self, x):
        out, h_n = self.gru1(x)
        dec_input = torch.zeros(x.size(0), 10, hidden_dim).to(x.device)
        out, _ = self.gru2(dec_input, h_n)
        out = self.fc(out)
        return out

def predict_trajectory(model, sequence):
    sequence_tensor = torch.tensor(sequence, dtype=torch.float32).unsqueeze(0).to(device)
    
    with torch.no_grad():
        predicted_output = model(sequence_tensor)
        
    return predicted_output.squeeze(0).cpu().numpy()

def compute_velocity(position_sequence, dt):
    diff = np.diff(position_sequence, axis=0)
    velocity = diff / dt
    return velocity

def plot_on_ax(ax, inputs, predictions, actuals=None, title='', linestyle='-', color='blue'):
    ax.plot(inputs[:, 0], inputs[:, 1], inputs[:, 2], label='Input', linestyle=linestyle, color=color, linewidth=2)
    ax.plot(predictions[:, 0], predictions[:, 1], predictions[:, 2], label='Predicted', linestyle='--', color='red', linewidth=2)
    
    # Plot the actuals if provided
    if actuals is not None:
        ax.plot(actuals[:, 0], actuals[:, 1], actuals[:, 2], label='Actual', linestyle=':', color='green', linewidth=2)
    
    ax.set_title(title)
    
    min_x, max_x = min(inputs[:, 0].min(), predictions[:, 0].min(), (actuals[:, 0].min() if actuals is not None else predictions[:, 0].min())), max(inputs[:, 0].max(), predictions[:, 0].max(), (actuals[:, 0].max() if actuals is not None else predictions[:, 0].max()))
    min_y, max_y = min(inputs[:, 1].min(), predictions[:, 1].min(), (actuals[:, 1].min() if actuals is not None else predictions[:, 1].min())), max(inputs[:, 1].max(), predictions[:, 1].max(), (actuals[:, 1].max() if actuals is not None else predictions[:, 1].max()))
    min_z, max_z = min(inputs[:, 2].min(), predictions[:, 2].min(), (actuals[:, 2].min() if actuals is not None else predictions[:, 2].min())), max(inputs[:, 2].max(), predictions[:, 2].max(), (actuals[:, 2].max() if actuals is not None else predictions[:, 2].max()))
    
    ax.set_xlim(min_x - 5, max_x + 5)
    ax.set_ylim(min_y - 5, max_y + 5)
    ax.set_zlim(min_z - 5, max_z + 5)

def compute_predicted_positions(last_position, predicted_velocity, dt):
    predicted_positions = [last_position]
    for v in predicted_velocity:
        new_position = predicted_positions[-1] + v * dt
        predicted_positions.append(new_position)
    return np.array(predicted_positions)

if __name__ == '__main__':
    position_model_path = sys.argv[1]
    velocity_model_path = sys.argv[2]

    # Hyperparameters
    input_dim = 3
    hidden_dim = 64
    output_dim = 3
    num_layers = 2
    
    # Load the position model
    position_model = TrajectoryPredictor(input_dim, hidden_dim, output_dim, num_layers)
    position_model.load_state_dict(torch.load(position_model_path, map_location=device))
    position_model.to(device)
    position_model.eval()

    # Load the velocity model
    velocity_model = VelocityPredictor(input_dim, hidden_dim, output_dim, num_layers)
    velocity_model.load_state_dict(torch.load(velocity_model_path, map_location=device))
    velocity_model.to(device)
    velocity_model.eval()

    dt = 0.1
    speed = 7.0  # Speed along the trajectory in meters/second
    traj_type='line'
    inp_seq_len = 20
    out_seq_len = 10
    if traj_type=='line':
        # Define a straight line of 3D points
        velocity_x = speed  # constant velocity in the x-axis
        velocity_y = speed
        velocity_z = speed
        timesteps = np.arange(inp_seq_len+1+out_seq_len) * dt
        
        # Start from an arbitrary point, for example (5, 3, 2)
        start_position = np.array([-10, 5, 3])
        x_coords = start_position[0] + velocity_x * timesteps
        y_coords = start_position[1] + velocity_y * timesteps #np.full_like(timesteps, start_position[1])
        z_coords = start_position[2] + velocity_z * timesteps #np.full_like(timesteps, start_position[2])

        true_predicted_x=x_coords[-out_seq_len:]
        true_predicted_y=y_coords[-out_seq_len:]
        true_predicted_z=z_coords[-out_seq_len:]
        x_coords = x_coords[:inp_seq_len+1]
        y_coords = y_coords[:inp_seq_len+1]
        z_coords = z_coords[:inp_seq_len+1]

    elif traj_type=='circle':
        ####################### Define the arc trajectory #######################
        r = 10  # Radius
        h = 3  # Height
        xi, yi = 0, 0  # Starting coordinates in the XY-plane

        # Calculate the angle covered in each timestep
        distance_per_timestep = speed * dt
        theta_increment = distance_per_timestep / r
        
        # Generate angles for 20 timesteps
        timesteps = np.arange(inp_seq_len+1+out_seq_len) * theta_increment
        
        x_coords = xi + r * np.cos(timesteps)
        y_coords = yi + r * np.sin(timesteps)
        z_coords = np.full_like(timesteps, h)

        true_predicted_x=x_coords[-out_seq_len:]
        true_predicted_y=y_coords[-out_seq_len:]
        true_predicted_z=z_coords[-out_seq_len:]
        x_coords = x_coords[:inp_seq_len+1]
        y_coords = y_coords[:inp_seq_len+1]
        z_coords = z_coords[:inp_seq_len+1]
    else:
        print(f"traj_type {traj_type} is not valid")
        exit(1)
    
    sequence = np.vstack([x_coords, y_coords, z_coords]).T

    true_pred_sequence = np.vstack([true_predicted_x, true_predicted_y, true_predicted_z]).T

    # Compute velocity from the 21-point input position sequence
    input_velocity = compute_velocity(sequence, dt=0.1)

    # Predict the position trajectory using only the first 20 points
    predicted_position = predict_trajectory(position_model, sequence[:-1])

    # Predict the output velocity sequence using the input velocity
    predicted_velocity_output = predict_trajectory(velocity_model, input_velocity)
    
    # Compute the predicted position using the last point in the input sequence, predicted velocities and dt
    predicted_positions_from_velocity = compute_predicted_positions(sequence[-1], predicted_velocity_output, dt)

    # Instead of separate figures, use one figure with 3 subplots
    fig = plt.figure(figsize=(10, 15))
    ax1 = fig.add_subplot(311, projection='3d')
    ax2 = fig.add_subplot(312, projection='3d')
    ax3 = fig.add_subplot(313, projection='3d')
    
    # Plot on the subplots
    plot_on_ax(ax1, sequence[:-1], predicted_position, actuals=true_pred_sequence, title=traj_type+' Position Trajectory')
    plot_on_ax(ax2, input_velocity, predicted_velocity_output, title='Velocity Trajectory')
    plot_on_ax(ax3, sequence, predicted_positions_from_velocity, actuals=true_pred_sequence, title='Position from Velocity')

    # Set global legend for the figure
    handles, labels = ax1.get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper right')

    # Adjust layout and show the plot
    plt.tight_layout()
    plt.show()

    # Save the figure as a PDF
    fig.savefig(traj_type+'_trajectory.pdf', format='pdf')
