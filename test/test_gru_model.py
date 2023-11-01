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
    # Define the architecture of your velocity predictor model here
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

def plot_trajectories_with_matplotlib(inputs, predictions):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(inputs[:, 0], inputs[:, 1], inputs[:, 2], label='Input Sequence', color='blue', linewidth=2)
    ax.plot(predictions[:, 0], predictions[:, 1], predictions[:, 2], label='Predicted Trajectory', color='red', linestyle='--', linewidth=2)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-10, 10)
    plt.legend()
    plt.show()

def plot_velocity_in_3D(input_velocity, predicted_velocity):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plotting the input 3D velocity sequence
    ax.plot(input_velocity[:, 0], input_velocity[:, 1], input_velocity[:, 2], 
            label='Input Velocity Sequence', color='blue', linewidth=2)
    
    # Plotting the predicted 3D velocity sequence
    ax.plot(predicted_velocity[:, 0], predicted_velocity[:, 1], predicted_velocity[:, 2], 
            label='Predicted Velocity Sequence', color='red', linestyle='--', linewidth=2)
    
    ax.set_xlabel('Velocity in X')
    ax.set_ylabel('Velocity in Y')
    ax.set_zlabel('Velocity in Z')
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-10, 10)
    plt.legend()
    plt.show()

def compute_predicted_positions(last_position, predicted_velocity, dt):
    predicted_positions = [last_position]
    for v in predicted_velocity:
        new_position = predicted_positions[-1] + v * dt
        predicted_positions.append(new_position)
    return np.array(predicted_positions)

def plot_predicted_positions_using_velocity(input_sequence, predicted_positions_from_velocity):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(input_sequence[:, 0], input_sequence[:, 1], input_sequence[:, 2], 
            label='Input Position Sequence', color='blue', linewidth=2)
    
    ax.plot(predicted_positions_from_velocity[:, 0], predicted_positions_from_velocity[:, 1], predicted_positions_from_velocity[:, 2], 
            label='Predicted Position from Velocity', color='red', linestyle='--', linewidth=2)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-10, 10)
    plt.legend()
    plt.show()

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
    if traj_type=='line':
        # Define a straight line of 3D points
        velocity_x = speed  # constant velocity in the x-axis
        timesteps = np.arange(20) * dt
        
        # Start from an arbitrary point, for example (5, 3, 2)
        start_position = np.array([0, 0, 0])
        x_coords = start_position[0] + velocity_x * timesteps
        y_coords = np.full_like(timesteps, start_position[1])
        z_coords = np.full_like(timesteps, start_position[2])

    elif traj_type=='circle':
        ####################### Define the arc trajectory #######################
        r = 10  # Radius
        h = 3  # Height
        xi, yi = 0, 0  # Starting coordinates in the XY-plane

        # Calculate the angle covered in each timestep
        distance_per_timestep = speed * dt
        theta_increment = distance_per_timestep / r
        
        # Generate angles for 20 timesteps
        timesteps = np.arange(20) * theta_increment
        
        x_coords = xi + r * np.cos(timesteps)
        y_coords = yi + r * np.sin(timesteps)
        z_coords = np.full_like(timesteps, h)
    else:
        print(f"traj_type {traj_type} is not valid")
        exit(1)
    
    sequence = np.vstack([x_coords, y_coords, z_coords]).T

    # Compute velocity from the 21-point input position sequence
    input_velocity = compute_velocity(sequence, dt=0.1)

    # Predict the position trajectory using only the first 20 points
    predicted_position = predict_trajectory(position_model, sequence[:-1])

    # Predict the output velocity sequence using the input velocity
    predicted_velocity_output = predict_trajectory(velocity_model, input_velocity)

    # Print and plot results
    print("Predicted Position Trajectory:")
    print(predicted_position)

    print("\nPredicted Velocity Output:")
    print(predicted_velocity_output)

    plot_trajectories_with_matplotlib(sequence[:-1], predicted_position)
    plot_velocity_in_3D(input_velocity, predicted_velocity_output)

    # Compute the predicted position using the last point in the input sequence, predicted velocities and dt
    predicted_positions_from_velocity = compute_predicted_positions(sequence[-1], predicted_velocity_output, dt)

    # Plot the results
    plot_predicted_positions_using_velocity(sequence, predicted_positions_from_velocity)
