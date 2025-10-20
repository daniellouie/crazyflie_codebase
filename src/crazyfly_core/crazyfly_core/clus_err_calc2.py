import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import glob
import os
from pathlib import Path
from enum import Enum
from abc import ABC, abstractmethod

# COORDINATE SYSTEM NOTE:
# Y-axis is vertical (altitude)
# X and Z are horizontal planes
plt.ion()
class FlightType(Enum):
    """Enumeration of supported flight types"""
    HOVER = "hover"
    DYNAMIC = "dynamic"
    P_MANEUVER = "p_maneuver"
    ROTATING = "rotating"

class FlightAnalyzer(ABC):
    """Abstract base class for flight analyzers"""
    
    @abstractmethod
    def identify_analysis_period(self, df):
        """Identify the period to analyze for this flight type"""
        pass
    
    @abstractmethod
    def get_description(self):
        """Return a description of this flight type"""
        pass
    
    def validate_data(self, df):
        """Common validation for all flight types"""
        required_cols = ['Cur_Cluster_Y', 'Des_Cluster_Y', 'Cur_P', 'Des_P']
        for col in required_cols:
            if col not in df.columns:
                raise ValueError(f"Required column {col} not found in data")
        return True

class HoverAnalyzer(FlightAnalyzer):
    """Analyzer for static hovering flights"""
    
    def get_description(self):
        return "Static Hovering Flight"
    
    def identify_analysis_period(self, df, y_col='Cur_Cluster_Y', threshold=0.02, min_duration=30):
        """
        Identify hovering period when Y position is stable at hover altitude.
        """
        # Get both current and desired Y positions
        y_position = df[y_col].values.copy()
        des_y_col = 'Des_' + y_col.split('_')[-2] + '_Y'
        des_y_position = df[des_y_col].values if des_y_col in df.columns else None
        
        # Find the hover altitude from desired position if available, otherwise use current position
        if des_y_position is not None:
            # Use the most common desired altitude
            hist, bins = np.histogram(des_y_position, bins=30)
            mode_bin = np.argmax(hist)
            hover_altitude = (bins[mode_bin] + bins[mode_bin + 1]) / 2
        else:
            # Fallback to current position analysis
            high_threshold = np.percentile(y_position, 70)
            high_values = y_position[y_position > high_threshold]
            if len(high_values) > 0:
                hist, bins = np.histogram(high_values, bins=30)
                mode_bin = np.argmax(hist)
                hover_altitude = (bins[mode_bin] + bins[mode_bin + 1]) / 2
            else:
                hover_altitude = np.max(y_position) * 0.95
        
        # Find periods within ±2% of hover altitude (tighter than before)
        altitude_band = 0.02 * hover_altitude
        within_band = np.abs(y_position - hover_altitude) <= altitude_band
        
        # Find continuous segments within the band
        segments = []
        in_segment = False
        start_idx = 0
        
        for i in range(len(within_band)):
            if within_band[i] and not in_segment:
                start_idx = i
                in_segment = True
            elif not within_band[i] and in_segment:
                if i - start_idx >= min_duration:
                    segments.append((start_idx, i-1))
                in_segment = False
        
        # Handle case where segment extends to end
        if in_segment and len(within_band) - start_idx >= min_duration:
            segments.append((start_idx, len(within_band)-1))
        
        if segments:
            # Choose the longest segment
            longest_segment = max(segments, key=lambda x: x[1] - x[0])
            start, end = longest_segment
            
            # Calculate rolling statistics to find stable period
            window_size = 10
            stability_threshold = 0.02  # 2cm variation
            
            # Get velocity if available (looking for near-zero velocity during hover)
            y_velocity = None
            if 'Cur_Cluster_Ydot' in df.columns:
                y_velocity = df['Cur_Cluster_Ydot'].iloc[start:end+1]
            
            # Check position stability
            y_window_std = df[y_col].iloc[start:end+1].rolling(window=window_size).std()
            
            # Find the most stable region
            for i in range(len(y_window_std)):
                if y_window_std.iloc[i] < stability_threshold:
                    if y_velocity is None or abs(y_velocity.iloc[i]) < 0.05:  # 5cm/s velocity threshold
                        start = start + i
                        break
            
            # Additional check for P stability in hovering
            if 'Cur_P' in df.columns and 'Des_P' in df.columns:
                p_error = np.abs(df['Cur_P'].iloc[start:end+1] - df['Des_P'].iloc[start:end+1])
                # Find when P stabilizes (error < 0.2m)
                for i in range(len(p_error)):
                    if p_error.iloc[i] < 0.2:
                        start = start + i
                        break
                        
            # Ensure minimum duration after all stability checks
            if end - start < min_duration:
                print("Warning: Stable period too short after all checks")
                return int(len(df) * 0.2), int(len(df) * 0.8)
            
            return start, end
        else:
            # Fallback
            print("Warning: Could not identify clear hovering period")
            return int(len(df) * 0.2), int(len(df) * 0.8)

class DynamicAnalyzer(FlightAnalyzer):
    """Analyzer for dynamic trajectory-following flights"""
    
    def get_description(self):
        return "Dynamic Trajectory Following"
    
    def identify_analysis_period(self, df, startup_trim=20, landing_trim=20):
        """
        For dynamic flights, analyze only the stable hovering period.
        """
        start = startup_trim
        end = len(df) - landing_trim
        
        # Ensure valid range
        if end <= start:
            print("Warning: Flight too short after trimming, using full data")
            start = 0
            end = len(df) - 1
        
        # Find when tracking begins (errors stabilize)
        if 'Cur_Cluster_Y' in df.columns and 'Des_Cluster_Y' in df.columns:
            y_error = np.abs(df['Cur_Cluster_Y'] - df['Des_Cluster_Y'])
            y_position = df['Cur_Cluster_Y']
            
            # Find when tracking error becomes reasonable at startup
            for i in range(start, min(start + 100, end)):
                window_error = y_error.iloc[i:i+10].mean()
                if window_error < 0.1:  # Within 10cm
                    start = i
                    break
            
            # Find when descent begins (looking for significant drop in Y position)
            window_size = 10
            for i in range(start + window_size, end):
                window_avg = y_position.iloc[i-window_size:i].mean()
                if y_position.iloc[i] < (window_avg * 0.9):  # 10% drop indicates descent
                    end = i
                    break
        
        return start, end

class PManeuverAnalyzer(FlightAnalyzer):
    """Analyzer for flights with P (inter-drone distance) changes"""
    
    def get_description(self):
        return "P-Parameter Maneuver (Distance Changes)"
    
    def identify_analysis_period(self, df, min_p_change=0.3):
        """
        Identify period where P changes significantly (actual maneuver).
        """
        if 'Des_P' not in df.columns:
            # Fallback to dynamic analyzer
            return DynamicAnalyzer().identify_analysis_period(df)
        
        des_p = df['Des_P'].values
        
        # Find where desired P starts changing
        p_diff = np.abs(np.diff(des_p))
        maneuver_mask = p_diff > 0.01  # P changing more than 1cm per sample
        
        # Find start of maneuver
        start = 0
        for i in range(len(maneuver_mask)):
            if maneuver_mask[i]:
                start = max(0, i - 10)  # Include a bit before the maneuver
                break
        
        # Find end of maneuver (when P stabilizes again)
        end = len(df) - 1
        for i in range(len(df) - 1, start, -1):
            if i >= 10:
                window_std = np.std(des_p[i-10:i])
                if window_std < 0.02:  # P stable
                    end = min(i + 10, len(df) - 1)
                    break
        
        # Ensure we capture actual P changes
        actual_p_change = np.max(des_p[start:end]) - np.min(des_p[start:end])
        if actual_p_change < min_p_change:
            print(f"Warning: Small P change detected ({actual_p_change:.2f}m), using full flight")
            return DynamicAnalyzer().identify_analysis_period(df)
        
        return start, end

class RotatingAnalyzer(FlightAnalyzer):
    """Analyzer for rotating cluster flights"""
    
    def get_description(self):
        return "Rotating Cluster Maneuver"
    
    def identify_analysis_period(self, df, rotation_threshold=0.1):
        """
        Identify period where cluster is rotating (alpha/beta changing).
        """
        # Check for rotation in alpha or beta
        rotation_detected = False
        start, end = 0, len(df) - 1
        
        for angle_col in ['Des_Alpha', 'Des_Beta']:
            if angle_col in df.columns:
                angle = df[angle_col].values
                angle_change = np.abs(np.diff(angle))
                
                if np.max(angle_change) > rotation_threshold:
                    rotation_detected = True
                    # Find rotation period
                    rotating = angle_change > 0.01
                    
                    # Find start
                    for i in range(len(rotating)):
                        if rotating[i]:
                            start = max(0, i - 10)
                            break
                    
                    # Find end
                    for i in range(len(rotating) - 1, start, -1):
                        if rotating[i]:
                            end = min(i + 10, len(df) - 1)
                            break
                    break
        
        if not rotation_detected:
            print("Warning: No rotation detected, using dynamic analysis")
            return DynamicAnalyzer().identify_analysis_period(df)
        
        return start, end

class ClusterFlightAnalyzer:
    """Main analyzer that delegates to specific flight type analyzers"""
    
    def __init__(self, flight_type=FlightType.HOVER):
        self.flight_type = flight_type
        self.analyzer = self._get_analyzer(flight_type)
    
    def _get_analyzer(self, flight_type):
        """Factory method to get appropriate analyzer"""
        analyzers = {
            FlightType.HOVER: HoverAnalyzer(),
            FlightType.DYNAMIC: DynamicAnalyzer(),
            FlightType.P_MANEUVER: PManeuverAnalyzer(),
            FlightType.ROTATING: RotatingAnalyzer()
        }
        return analyzers.get(flight_type, DynamicAnalyzer())
    
    def analyze_flight(self, df):
        """Analyze a single flight based on its type"""
        self.analyzer.validate_data(df)
        start, end = self.analyzer.identify_analysis_period(df)
        return start, end
    
    def collect_errors(self, filepath):
        """Process a single CSV file and collect error data"""
        df = pd.read_csv(filepath)
        
        # Get analysis period based on flight type
        start, end = self.analyze_flight(df)
        
        # Extract analysis period data
        analysis_df = df.loc[start:end].copy()
        
        # Define variables to analyze
        variables = {
            'X_c (m)': ('Cur_Cluster_X', 'Des_Cluster_X'),
            'Y_c (m)': ('Cur_Cluster_Y', 'Des_Cluster_Y'),
            'Z_c (m)': ('Cur_Cluster_Z', 'Des_Cluster_Z'),
            'α (rad)': ('Cur_Alpha', 'Des_Alpha'),
            'β (rad)': ('Cur_Beta', 'Des_Beta'),
            'P (m)': ('Cur_P', 'Des_P'),
            'φ₁ (rad)': ('Cur_Phi1', 'Des_Phi1'),
            'φ₂ (rad)': ('Cur_Phi2', 'Des_Phi2')
        }
        
        errors = {}
        for var_name, (cur_col, des_col) in variables.items():
            if cur_col in analysis_df.columns and des_col in analysis_df.columns:
                error = analysis_df[cur_col] - analysis_df[des_col]
                errors[var_name] = error.values
            else:
                errors[var_name] = np.array([])
        
        return errors, start, end, len(analysis_df), len(df)

def calculate_statistics(all_errors):
    """Calculate aggregate statistics across all flights"""
    results = []
    
    for var_name, error_list in all_errors.items():
        if error_list and any(len(e) > 0 for e in error_list):
            all_errors_concat = np.concatenate([e for e in error_list if len(e) > 0])
            
            # Calculate statistics
            avg_error = np.mean(all_errors_concat)
            std_dev = np.std(all_errors_concat)
            max_error = np.max(np.abs(all_errors_concat))
            rmse = np.sqrt(np.mean(all_errors_concat**2))
            
            results.append({
                'Variable': var_name,
                'Average Error': avg_error,
                'Std Dev': std_dev,
                'Max Error': max_error,
                'RMSE': rmse
            })
    
    return pd.DataFrame(results)

def plot_all_flights_overview(flight_data):
    """Create a summary plot showing hovering periods across all flights."""
    n_flights = len(flight_data)
    
    if n_flights == 0:
        print("No flight data to plot")
        return None
    
    # Determine subplot layout
    n_cols = min(3, n_flights)
    n_rows = (n_flights + n_cols - 1) // n_cols
    
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(5*n_cols, 4*n_rows))
    
    if n_flights == 1:
        axes = [axes]
    elif n_rows == 1 or n_cols == 1:
        axes = axes.flatten()
    else:
        axes = axes.flatten()
    
    for idx, (filepath, data) in enumerate(flight_data.items()):
        if idx >= len(axes):
            break
            
        ax = axes[idx]
        df = data['df']
        hover_start = data['analysis_start']
        hover_end = data['analysis_end']
        
        # Plot Y position (vertical) as it's most indicative of flight phase
        if 'Cur_Cluster_Y' in df.columns:
            ax.plot(df.index, df['Cur_Cluster_Y'], 'b-', alpha=0.7, label='Current Y', linewidth=2)
        if 'Des_Cluster_Y' in df.columns:
            ax.plot(df.index, df['Des_Cluster_Y'], 'r--', alpha=0.7, label='Desired Y')
        
        # Highlight hovering period with green
        ax.axvspan(hover_start, hover_end, alpha=0.3, color='green', label='Analysis Period')
        
        # Mark the boundaries clearly
        ax.axvline(x=hover_start, color='green', linestyle=':', alpha=0.8, linewidth=2)
        ax.axvline(x=hover_end, color='green', linestyle=':', alpha=0.8, linewidth=2)
        
        # Add text annotations
        y_lim = ax.get_ylim()
        ax.text(hover_start, y_lim[1]*0.95, 'Start', color='green', fontweight='bold')
        ax.text(hover_end, y_lim[1]*0.95, 'End', color='green', fontweight='bold')
        
        ax.set_title(f'Flight: {os.path.basename(filepath)}\nHover: samples {hover_start}-{hover_end}', fontsize=10)
        ax.set_xlabel('Sample Index')
        ax.set_ylabel('Y Position (m) - Vertical')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
    
    # Hide any unused subplots
    for idx in range(n_flights, len(axes)):
        axes[idx].axis('off')
    
    plt.suptitle('Flight Analysis Periods (Y=Vertical)', fontsize=14)
    plt.tight_layout()
    return fig

def plot_cluster_variables_and_errors(flight_data):
    """Create comprehensive plots showing all cluster variables and their errors during hovering."""
    # Define variables to plot
    variables = [
        ('Cluster_X', 'X Position (m) - Horizontal'),
        ('Cluster_Y', 'Y Position (m) - Vertical'),
        ('Cluster_Z', 'Z Position (m) - Horizontal'),
        ('Alpha', 'α (rad)'),
        ('Beta', 'β (rad)'),
        ('P', 'P (m)'),
        ('Phi1', 'φ₁ (rad)'),
        ('Phi2', 'φ₂ (rad)')
    ]
    
    # Create figure with subplots for each variable
    fig, axes = plt.subplots(4, 2, figsize=(15, 12))
    axes = axes.flatten()
    
    for idx, (var_base, label) in enumerate(variables):
        ax = axes[idx]
        
        # Plot data from all flights
        for flight_idx, (filepath, data) in enumerate(flight_data.items()):
            df = data['df']
            start = data['analysis_start']
            end = data['analysis_end']
            hover_df = df.loc[start:end]
            
            cur_col = f'Cur_{var_base}'
            des_col = f'Des_{var_base}'
            
            if cur_col in hover_df.columns and des_col in hover_df.columns:
                # Calculate error during hovering
                time = np.arange(len(hover_df))
                error = hover_df[cur_col].values - hover_df[des_col].values
                
                # Plot error for this flight
                flight_name = os.path.basename(filepath).replace('.csv', '')
                ax.plot(time, error, alpha=0.7, label=f'Flight {flight_idx+1}', linewidth=0.8)
        
        ax.set_title(f'{label} Error During Analysis')
        ax.set_xlabel('Time (samples)')
        ax.set_ylabel('Error')
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        
        if idx == 0:  # Only show legend on first plot
            ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
    
    plt.suptitle('Cluster Variable Errors During Analysis Period', fontsize=14)
    plt.tight_layout()
    return fig

def plot_error_distributions(flight_data):
    """Create box plots showing error distributions for each variable across all flights."""
    # Collect all errors
    variables = {
        'X_c': ('Cur_Cluster_X', 'Des_Cluster_X'),
        'Y_c': ('Cur_Cluster_Y', 'Des_Cluster_Y'),
        'Z_c': ('Cur_Cluster_Z', 'Des_Cluster_Z'),
        'P': ('Cur_P', 'Des_P'),
        'α': ('Cur_Alpha', 'Des_Alpha'),
        'β': ('Cur_Beta', 'Des_Beta'),
        'φ₁': ('Cur_Phi1', 'Des_Phi1'),
        'φ₂': ('Cur_Phi2', 'Des_Phi2')
    }
    
    error_data = {var: [] for var in variables.keys()}
    
    for filepath, data in flight_data.items():
        df = data['df']
        start = data['analysis_start']
        end = data['analysis_end']
        hover_df = df.loc[start:end]
        
        for var_name, (cur_col, des_col) in variables.items():
            if cur_col in hover_df.columns and des_col in hover_df.columns:
                errors = hover_df[cur_col].values - hover_df[des_col].values
                error_data[var_name].extend(errors)
    
    # Create box plots
    fig, axes = plt.subplots(2, 4, figsize=(16, 8))
    axes = axes.flatten()
    
    for idx, (var_name, errors) in enumerate(error_data.items()):
        if idx < len(axes):
            ax = axes[idx]
            if errors:
                ax.boxplot(errors, vert=True)
                ax.set_title(f'{var_name}')
                ax.set_ylabel('Error')
                ax.grid(True, alpha=0.3)
                ax.axhline(y=0, color='r', linestyle='--', alpha=0.5)
                
                # Add statistics text
                mean_err = np.mean(errors)
                std_err = np.std(errors)
                max_err = np.max(np.abs(errors))
                ax.text(0.98, 0.98, f'μ={mean_err:.4f}\nσ={std_err:.4f}\nmax={max_err:.4f}',
                       transform=ax.transAxes, ha='right', va='top',
                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
                       fontsize=8)
    
    plt.suptitle('Error Distributions for Cluster Variables (All Flights Combined)', fontsize=14)
    plt.tight_layout()
    return fig

def plot_analysis_periods(flight_data, flight_type):
    """Plot showing identified analysis periods for each flight"""
    n_flights = len(flight_data)
    if n_flights == 0:
        return None
    
    n_cols = min(3, n_flights)
    n_rows = (n_flights + n_cols - 1) // n_cols
    
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(5*n_cols, 4*n_rows))
    if n_flights == 1:
        axes = [axes]
    else:
        axes = np.array(axes).flatten()
    
    for idx, (filepath, data) in enumerate(flight_data.items()):
        if idx >= len(axes):
            break
        
        ax = axes[idx]
        df = data['df']
        start = data['analysis_start']
        end = data['analysis_end']
        
        # Plot Y position
        ax.plot(df.index, df['Cur_Cluster_Y'], 'b-', alpha=0.7, label='Current Y')
        ax.plot(df.index, df['Des_Cluster_Y'], 'r--', alpha=0.7, label='Desired Y')
        
        # Highlight analysis period
        ax.axvspan(start, end, alpha=0.3, color='green', label='Analysis Period')
        ax.axvline(x=start, color='green', linestyle=':', alpha=0.8)
        ax.axvline(x=end, color='green', linestyle=':', alpha=0.8)
        
        ax.set_title(f'{os.path.basename(filepath)}\nAnalysis: {start}-{end}')
        ax.set_xlabel('Sample Index')
        ax.set_ylabel('Y Position (m)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
    
    # Hide unused subplots
    for idx in range(n_flights, len(axes)):
        axes[idx].axis('off')
    
    plt.suptitle(f'{flight_type.value.title()} Flight Analysis Periods', fontsize=14)
    plt.tight_layout()
    return fig

def process_flights(file_pattern='cluster_data*.csv', directory='.', 
                    flight_type=FlightType.HOVER, debug=False):
    """Process multiple flight files with specified flight type"""
    
    # Initialize analyzer
    analyzer = ClusterFlightAnalyzer(flight_type)
    
    # Find all CSV files
    search_path = os.path.join(directory, file_pattern)
    csv_files = glob.glob(search_path)
    
    if not csv_files:
        print(f"No files found matching: {search_path}")
        return None, None
    
    print(f"\nProcessing {len(csv_files)} files as {flight_type.value} flights")
    print(f"Analyzer: {analyzer.analyzer.get_description()}")
    print("-" * 60)
    
    # Initialize error collection
    all_errors = {
        'X_c (m)': [], 'Y_c (m)': [], 'Z_c (m)': [],
        'α (rad)': [], 'β (rad)': [], 'P (m)': [],
        'φ₁ (rad)': [], 'φ₂ (rad)': []
    }
    
    flight_data = {}
    total_analysis_samples = 0
    total_samples = 0
    
    # Process each file
    for filepath in csv_files:
        filename = os.path.basename(filepath)
        print(f"Processing {filename}...")
        
        try:
            df = pd.read_csv(filepath)
            errors, start, end, analysis_samples, total = analyzer.collect_errors(filepath)
            
            if debug:
                print(f"  Analysis period: {start} to {end} ({analysis_samples} samples)")
                print(f"  Total samples: {total}")
                print(f"  Percentage analyzed: {analysis_samples/total*100:.1f}%")
            
            # Collect errors
            for var_name in all_errors.keys():
                if var_name in errors:
                    all_errors[var_name].append(errors[var_name])
            
            flight_data[filepath] = {
                'df': df,
                'analysis_start': start,
                'analysis_end': end,
                'analysis_samples': analysis_samples
            }
            
            total_analysis_samples += analysis_samples
            total_samples += total
            
        except Exception as e:
            print(f"  Error: {e}")
    
    print(f"\nTotal samples analyzed: {total_analysis_samples}/{total_samples} "
          f"({total_analysis_samples/total_samples*100:.1f}%)")
    
    # Calculate statistics
    error_stats = calculate_statistics(all_errors)
    
    return error_stats, flight_data

def main(directory='.', flight_type=FlightType.HOVER, file_pattern='cluster_data*.csv', 
         save_results=True, show_plots=True, debug=False):
    """
    Main analysis function with flight type selection
    
    Parameters:
    -----------
    directory : str
        Directory containing CSV files
    flight_type : FlightType
        Type of flight to analyze (HOVER, DYNAMIC, P_MANEUVER, ROTATING)
    file_pattern : str
        Pattern to match CSV files
    save_results : bool
        Whether to save results to CSV
    show_plots : bool
        Whether to display plots
    debug : bool
        Whether to print debug information
    
    Returns:
    --------
    error_stats : DataFrame
        Statistics for each variable
    flight_data : dict
        Detailed flight data
    """
    
    # Process flights
    error_stats, flight_data = process_flights(
        file_pattern=file_pattern,
        directory=directory,
        flight_type=flight_type,
        debug=debug
    )
    
    if error_stats is None:
        return None, None
    
    # Display results
    print("\n" + "="*70)
    print(f"ERROR STATISTICS - {flight_type.value.upper()} FLIGHTS")
    print(f"Analyzed {len(flight_data)} flights")
    print("="*70)
    
    formatted_df = error_stats.set_index('Variable').round(4)
    print(formatted_df.to_string())
    print("="*70)
    
    # Save results
    if save_results:
        output_file = f'{flight_type.value}_error_statistics.csv'
        error_stats.to_csv(output_file, index=False)
        print(f"\nResults saved to {output_file}")
    
    # Generate plots
    if show_plots:
        fig = plot_analysis_periods(flight_data, flight_type)
        plt.show()
    
    return error_stats, flight_data


# Example usage
if __name__ == "__main__":
    # Set the directory path for your data
    directory = os.path.expanduser('~/Desktop/crazyflie_codebase/cluster_data/cluster_dynamic')
    
    # Process the data and generate visualizations
    error_stats, flight_data = main( directory, debug=True)
    
    if error_stats is not None and flight_data is not None:
        print("\nGenerating separate plots...")
        
        # Create each plot in a new figure
        plt.figure('Flight Overview')
        fig1 = plot_all_flights_overview(flight_data)
        
        plt.figure('Error Time Series')
        fig2 = plot_cluster_variables_and_errors(flight_data)
        
        plt.figure('Error Distributions')
        fig3 = plot_error_distributions(flight_data)
        
        print("\nDisplaying all plots. Close all windows to exit.")
        plt.show(block=True)
        
        print("\nAnalysis complete!")