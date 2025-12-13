import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import glob


YLIM_CONFIG = {
    'cf1_static_hover_new': {
        'x': [0.0, 2.0],
        'y': [0.0, 2.0],
        'z': [0.0, 2.0]
    },
    'cf2_static_hover_new': {
        'x': [0.0, 2.0],
        'y': [0.0, 2.0],
        'z': [0.0, 2.0]
    },
     'cf1_dynamic_hover_new': {
        'x': [0.0, 3.0],
        'y':  [0.0, 3.0],
        'z':  [0.0, 3.0]
    },
    'cf1_dynamic_hover_new2': {
        'x':  [0.0, 3.0],
        'y':  [0.0, 3.0],
        'z':  [0.0, 3.0]
    },
    'cf2_dynamic_hover_new': {
        'x': [0.0, 3.0],
        'y': [0.0, 3.0],
        'z': [0.0, 3.0]
    },
    'cf2_dynamic_hover_new2': {
        'x': [0.0, 3.0],
        'y': [0.0, 3.0],
        'z': [0.0, 3.0]
    },
    'cf1_multiple_waypoint_new': {
        'x': [0.0, 3.0],
        'y': [0.0, 3.0],
        'z': [0.0, 3]
    },
    'cf2_multiple_waypoint_new': {
        'x': [0.0, 3.0],
        'y': [0.0, 3.0],
        'z': [0.0, 3.0]
    },
}

class FlightDataPlotter:
    def __init__(self, directory_path):
        """
        Initialize the plotter with the directory containing CSV files.
        
        Args:
            directory_path (str): Path to directory containing flight CSV files
        """
        self.directory_path = Path(directory_path)
        self.flight_data = []
        self.processed_data = {}

        dir_name = self.directory_path.name
        self.ylims = YLIM_CONFIG.get(dir_name, None)

        if self.ylims:
            print(f"loaded custom y-axis limits for '{dir_name}': {self.ylims}")
        else:
            print(f"No custom y-axis limits found for '{dir_name}', using auto-scaling")
        
    def file_log_reader(self, file_pattern="*.csv", sampling_rate=100.0):
        """
        Read all CSV files matching the pattern from the directory.
        
        Args:
            file_pattern (str): Pattern to match CSV files (default: "*.csv")
            sampling_rate (float): Sampling rate in Hz for creating time axis (default: 100 Hz)
        
        Returns:
            list: List of DataFrames, one for each CSV file
        """
        csv_files = list(self.directory_path.glob(file_pattern))
        
        if not csv_files:
            raise FileNotFoundError(f"No CSV files found in {self.directory_path}")
        
        self.flight_data = []
        
        for csv_file in csv_files:
            try:
                # Read CSV file
                df = pd.read_csv(csv_file)
                
                # Check if we have the expected columns
                # Your data has 'time_s', 'x', 'y', 'z'
                expected_columns = ['time_s', 'x', 'y', 'z']
                if not all(col in df.columns for col in expected_columns):
                    print(f"Warning: {csv_file.name} missing expected columns {expected_columns}. Found: {list(df.columns)}")
                    continue
                
                # Create proper time axis since time_s just contains repeated timestamps
                # Assume uniform sampling at the given rate
                num_samples = len(df)
                time_axis = np.arange(num_samples) / sampling_rate  # Time in seconds
                
                # Add the proper time column
                df['time'] = time_axis
                
                # Keep the original timestamp for reference
                df['start_timestamp'] = df['time_s'].iloc[0]
                
                # Add filename for reference
                df['filename'] = csv_file.name
                
                self.flight_data.append(df)
                print(f"Successfully loaded {csv_file.name} with {len(df)} data points")
                print(f"  Created time axis: 0 to {time_axis[-1]:.2f} seconds at {sampling_rate} Hz")
                
            except Exception as e:
                print(f"Error reading {csv_file.name}: {e}")
        
        print(f"Total files loaded: {len(self.flight_data)}")
        return self.flight_data
    
    def process_flight_data(self, axis='y'):
        """
        Process flight data to calculate averages and standard deviations.
        
        Args:
            axis (str): Axis to process ('x', 'y', or 'z')
        
        Returns:
            dict: Processed data with time, mean, and std arrays
        """
        if not self.flight_data:
            raise ValueError("No flight data loaded. Call file_log_reader() first.")
        
        # Find common time range across all flights
        all_times = []
        for df in self.flight_data:
            all_times.extend(df['time'].tolist())
        
        # Create a common time grid
        min_time = min(all_times)
        max_time = max(all_times)
        
        # Use the finest time resolution from all datasets
        all_dt = []
        for df in self.flight_data:
            if len(df) > 1:
                dt = np.diff(df['time']).min()
                all_dt.append(dt)
        
        if all_dt:
            time_step = min(all_dt)
        else:
            time_step = 0.1  # Default time step
        
        # Create common time array
        common_time = np.arange(min_time, max_time + time_step, time_step)
        
        # Interpolate all flights to common time grid
        interpolated_data = []
        
        for i, df in enumerate(self.flight_data):
            # Interpolate y-values to common time grid
            interpolated_values = np.interp(common_time, df['time'], df[axis])
            interpolated_data.append(interpolated_values)
            
        # Convert to numpy array for easier manipulation
        data_matrix = np.array(interpolated_data)
        
        # Calculate statistics across flights at each time point
        mean_values = np.mean(data_matrix, axis=0)
        std_values = np.std(data_matrix, axis=0)
        
        # Store processed data
        self.processed_data[axis] = {
            'time': common_time,
            'mean': mean_values,
            'std': std_values,
            'individual_flights': data_matrix
        }
        
        return self.processed_data[axis]
    
    def plot_flight_data(self, axes=['y'], figsize=(12, 8), main_title=None, title_pad=15, subplot_spacing=3.0, ylims=None):
        """
        Create subplot plots for specified axes.
        
        Args:
            axes (list): List of axes to plot ('x', 'y', 'z')
            figsize (tuple): Figure size (width, height)
            main_title (str): Main title for the entire figure (optional)
            title_pad (int): Padding above subplot titles (default: 15)
            subplot_spacing (float): Vertical spacing between subplots (default: 3.0)
        """
        if ylims is None:
            ylims = self.ylims
        
        n_plots = len(axes)
        
        # Create subplots
        fig, axs = plt.subplots(n_plots, 1, figsize=figsize, sharex=True)
        
        # Handle single subplot case
        if n_plots == 1:
            axs = [axs]
        
        colors = {'x': 'red', 'y': 'blue', 'z': 'green'}
        
        for i, axis in enumerate(axes):
            # Process data for this axis
            if axis not in self.processed_data:
                data = self.process_flight_data(axis)
            else:
                data = self.processed_data[axis]
            
            time = data['time']
            mean_vals = data['mean']
            std_vals = data['std']
            
            # Plot mean line (solid)
            axs[i].plot(time, mean_vals, 
                       color=colors.get(axis, 'blue'), 
                       linewidth=2, 
                       label=f'{axis.upper()}-axis Average',
                       solid_capstyle='round')
            
            # Plot standard deviation lines (lighter color)
            color_alpha = colors.get(axis, 'blue')
            axs[i].plot(time, mean_vals + std_vals, 
                       color=color_alpha, 
                       alpha=0.4, 
                       linewidth=1.5,
                       linestyle='--',
                       label=f'{axis.upper()}-axis +1σ')
            
            axs[i].plot(time, mean_vals - std_vals, 
                       color=color_alpha, 
                       alpha=0.4, 
                       linewidth=1.5,
                       linestyle='--',
                       label=f'{axis.upper()}-axis -1σ')
            
            # Fill between std deviation lines for better visualization
            axs[i].fill_between(time, 
                              mean_vals - std_vals, 
                              mean_vals + std_vals,
                              color=color_alpha, 
                              alpha=0.1)
            
            # Customize subplot
            axs[i].set_ylabel(f'{axis.upper()}-axis Value')
            axs[i].grid(True, alpha=0.3)
            # axs[i].legend(loc='lower right', fontsize='small', framealpha=0.9, )              
            # Clean subplot title with customizable positioning

            #axs[i] sets title for current subplot
            #
            axs[i].set_title(f'{axis.upper()}-axis', fontsize=12, pad=title_pad)
            if ylims and axis in ylims:
                axs[i].set_ylim(ylims[axis])
        
        # Set x-label only for bottom subplot
        axs[-1].set_xlabel('Time (seconds)')
        
        # Add main title if provided and adjust spacing
        if main_title:
            fig.suptitle(main_title, fontsize=16, fontweight='bold', y=0.96)
            # Adjust subplot spacing - more room between subplots and for main title
            plt.tight_layout(rect=[0, 0.03, 1, 0.93], h_pad=subplot_spacing, w_pad=2.0)
        else:
            # No main title, but still give good spacing between subplots
            plt.tight_layout(h_pad=subplot_spacing, w_pad=2.0)
        plt.show()
        
        return fig, axs
    
    def get_statistics_summary(self, axis='y'):
        """
        Get summary statistics for the processed data.
        
        Args:
            axis (str): Axis to get statistics for
            
        Returns:
            dict: Summary statistics
        """
        if axis not in self.processed_data:
            self.process_flight_data(axis)
        
        data = self.processed_data[axis]
        
        summary = {
            'num_flights': len(self.flight_data),
            'time_range': (data['time'].min(), data['time'].max()),
            'mean_range': (data['mean'].min(), data['mean'].max()),
            'avg_std': np.mean(data['std']),
            'max_std': np.max(data['std']),
            'min_std': np.min(data['std'])
        }
        
        return summary
    
    def analyze_steady_state_error(self, axes=['x', 'y', 'z'], 
                                  steady_state_start_time=10.0, 
                                  steady_state_end_time=None,
                                  target_position={'x': 2.0, 'y': 1.0, 'z': 3.0},   # DONT NEED TO CHANGE
                                  plot_results=True):
        """
        Analyze steady-state error for flight data after initial transients.
        
        Args:
            axes (list): List of axes to analyze ('x', 'y', 'z')
            steady_state_start_time (float): Start time for steady-state analysis (seconds)
            steady_state_end_time (float): End time for steady-state analysis (None = use all data after start)
            target_position (dict): Target position for each axis (used to calculate error)
            plot_results (bool): Whether to plot the steady-state analysis results
        
        Returns:
            dict: Steady-state statistics for each axis
        """
        
        if not self.flight_data:
            raise ValueError("No flight data loaded. Call file_log_reader() first.")
        
        steady_state_results = {}
        
        for axis in axes:
            # Process data if not already done
            if axis not in self.processed_data:
                self.process_flight_data(axis)
            
            data = self.processed_data[axis]
            time = data['time']
            mean_vals = data['mean']
            std_vals = data['std']
            individual_flights = data['individual_flights']
            
            # Find indices for steady-state window
            start_idx = np.argmin(np.abs(time - steady_state_start_time))
            
            if steady_state_end_time is not None:
                end_idx = np.argmin(np.abs(time - steady_state_end_time))
            else:
                end_idx = len(time)
            
            # Extract steady-state portion
            ss_time = time[start_idx:end_idx]
            ss_mean = mean_vals[start_idx:end_idx]
            ss_std = std_vals[start_idx:end_idx]
            ss_individual = individual_flights[:, start_idx:end_idx]
            
            # Get target for this axis
            target = target_position.get(axis, 0.0)
            
            # Calculate steady-state statistics
            ss_avg_position = np.mean(ss_mean)
            ss_std_position = np.std(ss_mean)  # Variation in the mean position during steady state
            
            # Calculate errors
            ss_avg_error = ss_avg_position - target
            ss_abs_avg_error = np.abs(ss_avg_error)
            
            # RMS error across time for the averaged data
            ss_rms_error = np.sqrt(np.mean((ss_mean - target) ** 2))
            
            # Average standard deviation during steady state (spread between flights)
            ss_avg_spread = np.mean(ss_std)
            
            # Peak-to-peak variation in steady state
            ss_peak_to_peak = np.max(ss_mean) - np.min(ss_mean)
            
            # Individual flight errors during steady state
            individual_errors = []
            for flight_data in ss_individual:
                flight_error = np.mean(flight_data - target)
                individual_errors.append(flight_error)
            
            # Store results
            steady_state_results[axis] = {
                'time_window': (steady_state_start_time, steady_state_end_time if steady_state_end_time else time[-1]),
                'num_samples': len(ss_time),
                'target_position': target,
                'avg_position': ss_avg_position,
                'position_std': ss_std_position,
                'avg_error': ss_avg_error,
                'abs_avg_error': ss_abs_avg_error,
                'rms_error': ss_rms_error,
                'avg_spread_between_flights': ss_avg_spread,
                'peak_to_peak': ss_peak_to_peak,
                'individual_flight_errors': individual_errors,
                'max_individual_error': max(np.abs(e) for e in individual_errors),
                'min_individual_error': min(np.abs(e) for e in individual_errors),
            }
        
        # Print summary
        print("\n" + "="*60)
        print(f"STEADY-STATE ERROR ANALYSIS")
        print(f"Time window: {steady_state_start_time:.1f}s to {steady_state_end_time if steady_state_end_time else 'end'}")
        print("="*60)
        
        for axis in axes:
            results = steady_state_results[axis]
            print(f"\n{axis.upper()}-AXIS STEADY-STATE ANALYSIS:")
            print(f"  Target Position: {results['target_position']:.3f}")
            print(f"  Average Position: {results['avg_position']:.3f}")
            print(f"  Average SS Error: {results['avg_error']:+.4f}")
            print(f"  Absolute Average Error: {results['abs_avg_error']:.4f}")
            print(f"  RMS Error: {results['rms_error']:.4f}")
            print(f"  Position Std Dev: {results['position_std']:.4f}")
            print(f"  Peak-to-Peak Variation: {results['peak_to_peak']:.4f}")
            print(f"  Avg Spread Between Flights: {results['avg_spread_between_flights']:.4f}")
            print(f"  Max Individual Flight Error: {results['max_individual_error']:.4f}")
            print(f"  Min Individual Flight Error: {results['min_individual_error']:.4f}")
        
        # Optional: Create visualization
        if plot_results:
            self._plot_steady_state_analysis(steady_state_results, axes, 
                                            steady_state_start_time, 
                                            steady_state_end_time)
        
        return steady_state_results

    def _plot_steady_state_analysis(self, ss_results, axes, start_time, end_time):
        """
        Create visualization of steady-state analysis.
        
        Args:
            ss_results (dict): Steady-state results from analyze_steady_state_error
            axes (list): List of axes analyzed
            start_time (float): Start time of steady-state window
            end_time (float): End time of steady-state window (None for end of data)
        """
        n_plots = len(axes)
        fig, axs = plt.subplots(n_plots, 2, figsize=(14, 4*n_plots))
        
        # Handle single axis case
        if n_plots == 1:
            axs = axs.reshape(1, -1)
        
        colors = {'x': 'red', 'y': 'blue', 'z': 'green'}
        
        for i, axis in enumerate(axes):
            data = self.processed_data[axis]
            results = ss_results[axis]
            target = results['target_position']
            
            # Left plot: Full time series with steady-state window highlighted
            time = data['time']
            mean_vals = data['mean']
            std_vals = data['std']
            
            # Plot full trajectory
            axs[i, 0].plot(time, mean_vals, 
                          color=colors.get(axis, 'blue'), 
                          linewidth=2, 
                          label=f'{axis.upper()}-axis Average')
            
            # Plot target line
            axs[i, 0].axhline(y=target, color='black', linestyle=':', 
                             linewidth=1.5, label=f'Target ({target:.2f})')
            
            # Highlight steady-state region
            if end_time is None:
                end_time_plot = time[-1]
            else:
                end_time_plot = end_time
                
            axs[i, 0].axvspan(start_time, end_time_plot, alpha=0.2, 
                             color='yellow', label='Steady-State Window')
            
            # Add standard deviation
            axs[i, 0].fill_between(time, 
                                  mean_vals - std_vals, 
                                  mean_vals + std_vals,
                                  color=colors.get(axis, 'blue'), 
                                  alpha=0.1)
            
            axs[i, 0].set_xlabel('Time (seconds)')
            axs[i, 0].set_ylabel(f'{axis.upper()}-axis Value')
            axs[i, 0].set_title(f'{axis.upper()}-axis: Full Trajectory')
            axs[i, 0].legend(loc='best')
            axs[i, 0].grid(True, alpha=0.3)
            
            # Right plot: Error distribution during steady state
            errors = results['individual_flight_errors']
            
            # Create histogram
            axs[i, 1].hist(errors, bins=15, color=colors.get(axis, 'blue'), 
                          alpha=0.7, edgecolor='black')
            
            # Add vertical lines for statistics
            axs[i, 1].axvline(x=results['avg_error'], color='red', 
                             linestyle='--', linewidth=2, 
                             label=f'Mean Error: {results["avg_error"]:.4f}')
            axs[i, 1].axvline(x=0, color='black', linestyle=':', 
                             linewidth=1.5, label='Zero Error')
            
            axs[i, 1].set_xlabel(f'{axis.upper()}-axis Error')
            axs[i, 1].set_ylabel('Frequency (# of flights)')
            axs[i, 1].set_title(f'{axis.upper()}-axis: Steady-State Error Distribution')
            axs[i, 1].legend(loc='best')
            axs[i, 1].grid(True, alpha=0.3)
        
        plt.suptitle(f'Steady-State Analysis (t = {start_time:.1f}s to {end_time_plot:.1f}s)', 
                    fontsize=14, fontweight='bold')
        plt.tight_layout(rect=[0, 0.03, 1, 0.96])
        plt.show()
        
        return fig, axs

    def find_steady_state_automatically(self, axis='y', window_size=5.0, 
                                       threshold_factor=0.1, min_steady_time=5.0):
        """
        Automatically detect when steady state is reached based on rate of change.
        
        Args:
            axis (str): Axis to analyze
            window_size (float): Window size in seconds for computing rate of change
            threshold_factor (float): Threshold as fraction of initial rate of change
            min_steady_time (float): Minimum time before considering steady state (seconds)
        
        Returns:
            float: Time when steady state is detected
        """
        if axis not in self.processed_data:
            self.process_flight_data(axis)
        
        data = self.processed_data[axis]
        time = data['time']
        mean_vals = data['mean']
        
        # Calculate rolling rate of change
        dt = time[1] - time[0]  # Assuming uniform sampling
        window_samples = int(window_size / dt)
        
        # Compute moving standard deviation as measure of variation
        moving_std = []
        for i in range(window_samples, len(mean_vals)):
            window_data = mean_vals[i-window_samples:i]
            moving_std.append(np.std(window_data))
        
        # Find where variation drops below threshold
        if moving_std:
            initial_variation = np.max(moving_std[:int(len(moving_std)*0.2)])  # Max in first 20%
            threshold = initial_variation * threshold_factor
            
            # Find first point after min_steady_time where variation stays low
            min_idx = int(min_steady_time / dt)
            for i in range(min_idx, len(moving_std)):
                if all(std < threshold for std in moving_std[i:min(i+window_samples, len(moving_std))]):
                    steady_state_time = time[i + window_samples]
                    print(f"Steady state detected at t = {steady_state_time:.2f}s for {axis}-axis")
                    return steady_state_time
        
        # Default to a reasonable value if detection fails
        default_time = 10.0
        print(f"Could not detect steady state for {axis}-axis, using default t = {default_time}s")
        return default_time
    
    def analyze_time_to_peak(self, axis='y', start_time=0.0, end_time=None, 
                         plot_results=True, peak_type='max'):
        """
        Calculate time to peak for a specified axis within a time window.
    
        Args:
        axis (str): Axis to analyze ('x', 'y', or 'z')
        start_time (float): Start time of the window (seconds)
        end_time (float): End time of the window (None = use all data after start)
        plot_results (bool): Whether to plot the analysis
        peak_type (str): 'max' for maximum value, 'min' for minimum, 'abs' for absolute maximum
    
        Returns:
        dict: Analysis results including time to peak, peak value, etc.
        """
    
    # Process data if not already done
        if axis not in self.processed_data:
            self.process_flight_data(axis)
        
        data = self.processed_data[axis]
        time = data['time']
        mean_vals = data['mean']
        
        # Find indices for the window
        start_idx = np.argmin(np.abs(time - start_time))
        
        if end_time is not None:
            end_idx = np.argmin(np.abs(time - end_time))
        else:
            end_idx = len(time)
        
        # Extract window data
        window_time = time[start_idx:end_idx]
        window_vals = mean_vals[start_idx:end_idx]
        
        if len(window_vals) == 0:
            raise ValueError(f"No data in specified time window ({start_time} to {end_time})")
        
        # Find peak based on type
        if peak_type == 'max':
            peak_idx = np.argmax(window_vals)
            peak_value = window_vals[peak_idx]
        elif peak_type == 'min':
            peak_idx = np.argmin(window_vals)
            peak_value = window_vals[peak_idx]
        elif peak_type == 'abs':
            abs_vals = np.abs(window_vals - window_vals[0])  # Relative to start
            peak_idx = np.argmax(abs_vals)
            peak_value = window_vals[peak_idx]
        else:
            raise ValueError(f"Unknown peak_type: {peak_type}. Use 'max', 'min', or 'abs'")
        
        # Calculate time to peak
        peak_time = window_time[peak_idx]
        time_to_peak = peak_time - start_time
        
        # Calculate rise metrics
        start_value = window_vals[0]
        value_change = peak_value - start_value
        
        # Find 10-90% rise time (common metric)
        ten_percent_val = start_value + 0.1 * value_change
        ninety_percent_val = start_value + 0.9 * value_change
        
        # Find when these thresholds are crossed
        ten_percent_idx = None
        ninety_percent_idx = None
        
        if value_change > 0:  # Rising
            for i, val in enumerate(window_vals):
                if ten_percent_idx is None and val >= ten_percent_val:
                    ten_percent_idx = i
                if ninety_percent_idx is None and val >= ninety_percent_val:
                    ninety_percent_idx = i
                    break
        else:  # Falling
            for i, val in enumerate(window_vals):
                if ten_percent_idx is None and val <= ten_percent_val:
                    ten_percent_idx = i
                if ninety_percent_idx is None and val <= ninety_percent_val:
                    ninety_percent_idx = i
                    break
        
        rise_time_10_90 = None
        if ten_percent_idx is not None and ninety_percent_idx is not None:
            rise_time_10_90 = window_time[ninety_percent_idx] - window_time[ten_percent_idx]
        
        # Calculate overshoot if there's a settling value
        if end_time is not None and (end_idx - start_idx) > 50:
            # Use last 20% of window as settling region
            settling_start = int(0.8 * len(window_vals))
            settling_value = np.mean(window_vals[settling_start:])
            overshoot_percent = 100 * (peak_value - settling_value) / abs(settling_value - start_value) if settling_value != start_value else 0
        else:
            settling_value = None
            overshoot_percent = None
        
        # Store results
        results = {
            'axis': axis,
            'time_window': (start_time, end_time if end_time else time[-1]),
            'peak_type': peak_type,
            'time_to_peak': time_to_peak,
            'peak_time': peak_time,
            'peak_value': peak_value,
            'start_value': start_value,
            'value_change': value_change,
            'rise_time_10_90': rise_time_10_90,
            'settling_value': settling_value,
            'overshoot_percent': overshoot_percent
        }
        
        # Print results
        print(f"\n{'='*60}")
        print(f"TIME TO PEAK ANALYSIS - {axis.upper()}-AXIS")
        print(f"{'='*60}")
        print(f"Time Window: {start_time:.2f}s to {end_time if end_time else 'end':.2f}s")
        print(f"Peak Type: {peak_type}")
        print(f"Time to Peak: {time_to_peak:.4f} seconds")
        print(f"Peak Time: {peak_time:.4f}s")
        print(f"Peak Value: {peak_value:.4f}")
        print(f"Start Value: {start_value:.4f}")
        print(f"Value Change: {value_change:+.4f}")
        if rise_time_10_90:
            print(f"10-90% Rise Time: {rise_time_10_90:.4f} seconds")
        if overshoot_percent is not None:
            print(f"Overshoot: {overshoot_percent:.1f}%")
        
        # Plot if requested
        if plot_results:
            self._plot_time_to_peak_analysis(results, data, start_idx, end_idx)
        
        return results

    def _plot_time_to_peak_analysis(self, results, data, start_idx, end_idx):
        """
        Create visualization for time to peak analysis.
        """
        fig, ax = plt.subplots(1, 1, figsize=(12, 6))
        
        time = data['time']
        mean_vals = data['mean']
        std_vals = data['std']
        
        # Plot full trajectory with transparency
        ax.plot(time, mean_vals, 'b-', alpha=0.3, linewidth=1, label='Full Trajectory')
        
        # Highlight analysis window
        window_time = time[start_idx:end_idx]
        window_vals = mean_vals[start_idx:end_idx]
        ax.plot(window_time, window_vals, 'b-', linewidth=2, label='Analysis Window')
        
        # Mark the peak
        ax.plot(results['peak_time'], results['peak_value'], 'ro', 
            markersize=10, label=f'Peak ({results["peak_time"]:.2f}s, {results["peak_value"]:.3f})')
        
        # Mark the start
        ax.plot(results['time_window'][0], results['start_value'], 'go', 
            markersize=8, label=f'Start ({results["time_window"][0]:.2f}s, {results["start_value"]:.3f})')
        
        # Add vertical line for time to peak
        ax.vlines(results['peak_time'], results['start_value'], results['peak_value'],
                colors='r', linestyles='--', alpha=0.5)
        
        # Add horizontal line from start
        ax.hlines(results['start_value'], results['time_window'][0], results['peak_time'],
                colors='g', linestyles='--', alpha=0.5)
        
        # Annotate time to peak
        mid_time = results['time_window'][0] + results['time_to_peak']/2
        mid_value = (results['start_value'] + results['peak_value'])/2
        ax.annotate(f'Time to Peak:\n{results["time_to_peak"]:.3f}s', 
                xy=(mid_time, mid_value),
                xytext=(10, 10), textcoords='offset points',
                bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.7),
                fontsize=10)
        
        # If settling value exists, show it
        if results['settling_value'] is not None:
            ax.axhline(y=results['settling_value'], color='purple', linestyle=':', 
                    linewidth=1.5, label=f'Settling Value: {results["settling_value"]:.3f}')
        
        # Add standard deviation
        ax.fill_between(time[start_idx:end_idx], 
                        mean_vals[start_idx:end_idx] - std_vals[start_idx:end_idx],
                        mean_vals[start_idx:end_idx] + std_vals[start_idx:end_idx],
                        color='blue', alpha=0.1)
        
        ax.set_xlabel('Time (seconds)')
        ax.set_ylabel(f'{results["axis"].upper()}-axis Value')
        ax.set_title(f'Time to Peak Analysis - {results["axis"].upper()}-axis\n'
                    f'Time to Peak: {results["time_to_peak"]:.3f}s | '
                    f'Peak Value: {results["peak_value"]:.3f}')
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
        return fig, ax

    def analyze_waypoint_transitions(self, waypoint_times, axes=['x', 'y', 'z'], 
                                    margin=0.5, plot_summary=True):
        """
        Analyze time to peak for multiple waypoint transitions.
        
        Args:
            waypoint_times (list): List of tuples (start_time, end_time) for each waypoint transition
            axes (list): List of axes to analyze
            margin (float): Time margin before/after transition (seconds)
            plot_summary (bool): Whether to create summary plot
        
        Returns:
            dict: Results for each waypoint and axis
        """
        
        all_results = {}
        
        for wp_idx, (start, end) in enumerate(waypoint_times):
            wp_results = {}
            
            # Add margin to capture full transition
            analysis_start = max(0, start - margin)
            analysis_end = end + margin if end is not None else None
            
            print(f"\n{'='*60}")
            print(f"WAYPOINT {wp_idx + 1} TRANSITION ANALYSIS")
            print(f"Transition: {start:.1f}s to {end if end else 'end':.1f}s")
            print(f"{'='*60}")
            
            for axis in axes:
                results = self.analyze_time_to_peak(
                    axis=axis,
                    start_time=analysis_start,
                    end_time=analysis_end,
                    plot_results=False,  # We'll create a summary plot instead
                    peak_type='abs'  # Use absolute for waypoint changes
                )
                wp_results[axis] = results
            
            all_results[f'waypoint_{wp_idx+1}'] = wp_results
        
        # Create summary statistics
        if plot_summary:
            self._plot_waypoint_summary(all_results, waypoint_times)
        
        return all_results
    def detect_waypoint_transitions(self, axis='z', threshold=0.3, min_separation=5.0):
        """
        Automatically detect waypoint transitions by finding sudden changes in commanded position.
        
        Args:
            axis (str): Axis to analyze for waypoint detection
            threshold (float): Minimum position change to consider a waypoint (meters)
            min_separation (float): Minimum time between waypoints (seconds)
        
        Returns:
            list: List of detected waypoint times
        """
        if axis not in self.processed_data:
            self.process_flight_data(axis)
        
        data = self.processed_data[axis]
        time = data['time']
        mean_vals = data['mean']
        
        # Calculate derivative (rate of change)
        dt = time[1] - time[0]
        derivative = np.gradient(mean_vals, dt)
        
        # Find peaks in absolute derivative (sudden changes)
        abs_derivative = np.abs(derivative)
        
        # Threshold for detecting significant changes
        derivative_threshold = threshold / 1.0  # threshold per second
        
        waypoint_times = []
        last_waypoint_time = -min_separation
        
        for i in range(len(time)):
            if abs_derivative[i] > derivative_threshold:
                # Check if enough time has passed since last waypoint
                if time[i] - last_waypoint_time >= min_separation:
                    waypoint_times.append(time[i])
                    last_waypoint_time = time[i]
        
        print(f"\nDetected {len(waypoint_times)} waypoint transitions on {axis}-axis:")
        for i, t in enumerate(waypoint_times):
            print(f"  Waypoint {i+1}: t = {t:.2f}s")
        
        return waypoint_times
        



def analyze_static_hover_with_steady_state(axes=['x', 'y', 'z'], sampling_rate=100.0):
    """
    Analyze static hover flight data with steady-state error analysis
    """
    # directory_path = "/Users/connorbishop/Desktop/crazyflie_codebase/final_flight_data/cf2_static_hover_new"
    directory_path = "/Users/connorbishop/Desktop/crazyflie_codebase/final_flight_data/cf2_dynamic_hover_new2"
    # directory_path = "/Users/connorbishop/Desktop/crazyflie_codebase/final_flight_data/cf2_multiple_waypoint_new_ind"

    print("=== Static Hover Flight Analysis with Steady-State Error ===")
    plotter = FlightDataPlotter(directory_path)
    
    # Read and process data
    flight_data = plotter.file_log_reader("cf2_tuning_*.csv", sampling_rate=sampling_rate)
    
    if flight_data:
        # Create standard plots first
        #NOTE: THIS IS THE PLOT
        # fig, axs = plotter.plot_flight_data(axes=axes, main_title="Static Hold at With CF2 SSSS")
        
        # Analyze steady-state error (from 10 seconds onward based on your plot)
        ss_resultsx = plotter.analyze_steady_state_error(
            axes=axes,
            steady_state_start_time=15,  # Adjust based on when your system settles
            steady_state_end_time=None,     # None means go to end of data
            target_position={'x': 1.0, 'y': 1.0, 'z': 1.0},  # Your setpoint NOTE: CHANGE HEREEEEE
            plot_results= True  # This will create additional analysis plots
        )
        ss_resultsz = plotter.analyze_steady_state_error(
            axes=axes,
            steady_state_start_time=15,  # Adjust based on when your system settles
            steady_state_end_time=None,     # None means go to end of data
            target_position={'x': 1.0, 'y': 1.0, 'z': 1.0},  # Your setpoint NOTE: CHANGE HEREEEEE
            plot_results= False  # This will create additional analysis plots
        )
        
        # Print statistics
        for axis in axes:
            stats = plotter.get_statistics_summary(axis)
            print(f"\n{axis.upper()}-axis: {stats['num_flights']} flights, avg std: {stats['avg_std']:.3f}")
    
    return plotter

def analyze_dynamic_hover_with_steady_state(axes=['x', 'y', 'z'], sampling_rate=100.0):
    """
    Analyze dynamic hover flight data with steady-state error analysis
    """
    directory_path = "/Users/connorbishop/Desktop/crazyflie_codebase/final_flight_data/cf2_multiple_waypoint"
    
    print("=== Dynamic Hover Flight Analysis with Steady-State Error ===")
    plotter = FlightDataPlotter(directory_path)
    
    # Read and process data
    flight_data = plotter.file_log_reader("cf2_tuning_*.csv", sampling_rate=sampling_rate)
    
    if flight_data:
        # Create standard plots
        fig, axs = plotter.plot_flight_data(axes=axes, main_title="Multiple Waypoint Navigation With CF2 SSSSSSSS")
        
        # For waypoint navigation, you might want to analyze different segments
        # Example: if you hover at different waypoints at different times
        waypoint_1_results = plotter.analyze_steady_state_error(
            axes=axes,
            steady_state_start_time=12.5,
            steady_state_end_time= None,
            target_position={'x': 1.0, 'y': 1.0, 'z': 3.0},  # First waypoint
            plot_results=False
        )
        
        # You can analyze multiple segments for different waypoints
        
    return plotter



# Analysis Functions - Choose which one to call
def analyze_static_hover(axes=['x', 'y', 'z'], sampling_rate=100.0):
    """Analyze static hover flight data"""
    directory_path = "/Users/connorbishop/Desktop/crazyflie_codebase/final_flight_data/cf1_static_hover_new"
    
    print("=== Static Hover Flight Analysis ===")
    plotter = FlightDataPlotter(directory_path)
    
    # Read and process data
    flight_data = plotter.file_log_reader("cf1_tuning_*.csv", sampling_rate=sampling_rate)
    
    if flight_data:
        # Create plots with main title
        fig, axs = plotter.plot_flight_data(axes=axes, main_title="Static Hold at () With CF111")
        
        # Print statistics
        for axis in axes:
            stats = plotter.get_statistics_summary(axis)
            print(f"\n{axis.upper()}-axis: {stats['num_flights']} flights, avg std: {stats['avg_std']:.3f}")
    
    return plotter

def analyze_dynamic_hover(axes=['x', 'y', 'z'], sampling_rate=100.0):
    """Analyze dynamic hover flight data"""
    directory_path = "/Users/connorbishop/Desktop/crazyflie_codebase/final_flight_data/cf1_dynamic_hover_new"
    # directory_path = "/Users/connorbishop/Desktop/crazyflie_codebase/final_flight_data/cf1_multiple_waypoint_new"

    
    print("=== Dynamic Hover Flight Analysis ===")
    plotter = FlightDataPlotter(directory_path)
    
    # Read and process data
    flight_data = plotter.file_log_reader("cf1_tuning_*.csv", sampling_rate=sampling_rate)
    
    if flight_data:
        # Create plots with main title
        fig, axs = plotter.plot_flight_data(axes=axes, main_title="dynamic waypoint From fasdfadsfa With CF11111")
        
        # Print statistics
        for axis in axes:
            stats = plotter.get_statistics_summary(axis)
            print(f"\n{axis.upper()}-axis: {stats['num_flights']} flights, avg std: {stats['avg_std']:.3f}")
    
    return plotter

def analyze_custom_path(directory_path, axes=['x', 'y', 'z'], file_pattern="cf1_tuning_*.csv", sampling_rate=100.0, main_title=None):
    """Analyze flight data from a custom directory path"""
    print(f"=== Flight Analysis: {directory_path.split('/')[-1]} ===")
    plotter = FlightDataPlotter(directory_path)
    
    # Read and process data
    flight_data = plotter.file_log_reader(file_pattern, sampling_rate=sampling_rate)
    
    if flight_data:
        # Create plots with optional main title
        if main_title is None:
            main_title = directory_path.split('/')[-1].replace('_', ' ').title()
        fig, axs = plotter.plot_flight_data(axes=axes, main_title=main_title)
        
        # Print statistics
        for axis in axes:
            stats = plotter.get_statistics_summary(axis)
            print(f"\n{axis.upper()}-axis: {stats['num_flights']} flights, avg std: {stats['avg_std']:.3f}")
    
    return plotter

# Example usage for analyzing waypoint transitions
def analyze_waypoint_flight_with_timing():
    """
    Analyze waypoint navigation with time to peak calculations
    """
    directory_path = "/Users/connorbishop/Desktop/crazyflie_codebase/final_flight_data/cf2_multiple_waypoint_new_ind"
    
    plotter = FlightDataPlotter(directory_path)
    flight_data = plotter.file_log_reader("cf2_tuning_*.csv", sampling_rate=100.0)
    
    if flight_data:
        # First, plot the full trajectory to identify waypoint transitions
        fig, axs = plotter.plot_flight_data(axes=['x', 'y', 'z'], 
                                           main_title="Waypoint Navigation Analysis")
        
          # AUTOMATIC DETECTION - detect waypoints on z-axis
        # waypoint_times_z = plotter.detect_waypoint_transitions(
        #     axis='z', 
        #     threshold=0.5,  # Detect changes > 0.5m
        #     min_separation=5.0  # At least 5s apart
        # )
        
        # # Create waypoint windows (start at detected time, end 5s later)
        # waypoint_transitions = []
        # for wp_time in waypoint_times_z:
        #     waypoint_transitions.append((wp_time, wp_time + 5.0))
        
        # Analyze specific waypoint transitions
        # Example: First waypoint transition from t=5s to t=15s
        results_x = plotter.analyze_time_to_peak(
            axis='x',
            start_time=0.0,   # Start of maneuver
            end_time=10.0,    # End of maneuver
            plot_results=True,
            peak_type='abs'   # or 'min' or 'abs'
        )
        results_z = plotter.analyze_time_to_peak(
            axis='z',

            start_time=15.1,   # Start of maneuver
            end_time=20.0,    # End of maneuver
            plot_results=True,
            peak_type='abs'   # or 'min' or 'abs'
        )
        
        # Analyze multiple waypoints
        waypoint_transitions = [
            (0.0, 5.0),   # First waypoint
            (15.1, 20.0),  # Second waypoint
            # (25.0, 35.0),  # Third waypoint
        ]
        
        all_results = plotter.analyze_waypoint_transitions(
            waypoint_times=waypoint_transitions,
            axes=['x', 'y', 'z'],
            margin=0.0,
            plot_summary=True
        )
        
        # Print summary
        for wp, wp_results in all_results.items():
            print(f"\n{wp.upper()} Summary:")
            for axis, results in wp_results.items():
                print(f"  {axis}-axis: Time to Peak = {results['time_to_peak']:.3f}s")
    
    return plotter



# Main execution
if __name__ == "__main__":
    # Choose which analysis to run by commenting/uncommenting:

    #NOTE: TIME TO PEAK HERE
    # analyze_waypoint_flight_with_timing()
    
    #NOTE: PLOTSSS Call Here
    # Analyze static hover data (all axes) - will show "Static Hold" as main title
    # analyze_static_hover()
    #
    
    # Analyze dynamic hover data (all axes) - will show "Dynamic Hold" as main title
    # analyze_dynamic_hover()
    
    # Analyze just Y-axis for dynamic hover
    # analyze_dynamic_hover(axes=['y'])
    
    # Analyze static hover with custom sampling rate
    # analyze_static_hover(axes=['x', 'y', 'z'], sampling_rate=50.0)
    
    # Analyze custom directory with custom title
    # analyze_custom_path("/path/to/your/custom/directory", 
    #                    axes=['y'], 
    #                    main_title="Custom Flight Analysis")

    # Quick usage examples for your specific data:
    """
    To use this code with your crazyflie data:

    1. Uncomment the analysis function you want at the bottom
    2. Run: python3 sd_flightplots.py

    EXAMPLES:

    # Static hover with main title "Static Hold" and subplot titles "X-axis", "Y-axis", "Z-axis":
    analyze_static_hover()

    # Dynamic hover with main title "Dynamic Hold":
    analyze_dynamic_hover()

    # Just Y-axis for dynamic hover:
    analyze_dynamic_hover(axes=['y'])

    # Custom directory with custom main title:
    analyze_custom_path("/path/to/data", 
                axes=['x', 'y', 'z'], 
                main_title="My Custom Analysis")

    # Custom sampling rate:
    analyze_static_hover(sampling_rate=50.0)

    The plots will show:
    - Main title at the top (e.g., "Static Hold")
    - Individual subplot titles (e.g., "X-axis", "Y-axis", "Z-axis") 
    - Solid lines for averages
    - Dashed lines and shaded areas for standard deviation
    """

    # Now you have options:

    # Option 1: Use the original analysis (without steady-state)
    # analyze_static_hover()

    # Option 2: Use the new analysis with steady-state error calculation
    analyze_static_hover_with_steady_state()

    # Option 3: Analyze dynamic hover with steady-state
    # analyze_dynamic_hover_with_steady_state()




