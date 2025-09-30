import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import glob
import os
from pathlib import Path



def identify_hovering_period(df, y_col='Cur_Cluster_Y', threshold=0.02, min_duration=30):
    """
    Identify the hovering period by finding when the CURRENT Y position (vertical) is stable at hover altitude.
    """
    
    if y_col not in df.columns:
        print(f"Warning: {y_col} not found, trying Des_Cluster_Y")
        y_col = 'Des_Cluster_Y'
    
    # Get the current Y position
    y_position = df[y_col].values.copy()
    
    # Find the hover altitude using robust statistics
    high_threshold = np.percentile(y_position, 70)
    high_values = y_position[y_position > high_threshold]
    
    if len(high_values) > 0:
        # Create histogram to find the mode
        hist, bins = np.histogram(high_values, bins=30)
        mode_bin = np.argmax(hist)
        hover_altitude = (bins[mode_bin] + bins[mode_bin + 1]) / 2
    else:
        hover_altitude = np.max(y_position) * 0.95
    
    # Start with a slightly more relaxed band (±5% instead of ±3%)
    altitude_band = 0.05 * hover_altitude
    within_band = np.abs(y_position - hover_altitude) <= altitude_band
    
    # Find all continuous segments within the band
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
        # For multiple segments, check if they can be merged
        # This handles cases where brief exits from the band split the hovering
        if len(segments) > 1:
            merged_segments = []
            current_start = segments[0][0]
            current_end = segments[0][1]
            
            for i in range(1, len(segments)):
                # If segments are close (within 20 samples), merge them
                if segments[i][0] - current_end < 20:
                    current_end = segments[i][1]
                else:
                    merged_segments.append((current_start, current_end))
                    current_start = segments[i][0]
                    current_end = segments[i][1]
            
            merged_segments.append((current_start, current_end))
            segments = merged_segments
        
        # Choose the longest segment
        longest_segment = max(segments, key=lambda x: x[1] - x[0])
        hover_start, hover_end = longest_segment
        
        # Don't apply aggressive fine-tuning that might cut off valid hovering
        # Just trim obvious landing at the end
        for i in range(hover_end, max(hover_start, hover_end - 10), -1):
            if i + 5 < len(y_position):
                if y_position[i] - y_position[min(i+5, len(y_position)-1)] > 0.1:
                    hover_end = i
                    break
        
        return hover_start, hover_end

# COORDINATE SYSTEM NOTE:
# Y-axis is vertical (altitude)
# X and Z are horizontal planes


def collect_hover_errors_from_file(filepath):
    """
    Process a single CSV file and collect error data from hovering period.
    
    Returns a dictionary with error arrays for each variable.
    """
    
    # Read the CSV file
    df = pd.read_csv(filepath)
    
    # Identify hovering period
    hover_start, hover_end = identify_hovering_period(df)
    
    # Extract hovering data
    hover_df = df.loc[hover_start:hover_end].copy()
    
    # Define the variables to analyze
    variables = {
        'X_c (m)': ('Cur_Cluster_X', 'Des_Cluster_X'),  # Horizontal
        'Y_c (m)': ('Cur_Cluster_Y', 'Des_Cluster_Y'),  # Vertical
        'Z_c (m)': ('Cur_Cluster_Z', 'Des_Cluster_Z'),  # Horizontal
        'α (rad)': ('Cur_Alpha', 'Des_Alpha'),
        'β (rad)': ('Cur_Beta', 'Des_Beta'),
        'P (m)': ('Cur_P', 'Des_P'),
        'φ₁ (rad)': ('Cur_Phi1', 'Des_Phi1'),
        'φ₂ (rad)': ('Cur_Phi2', 'Des_Phi2')
    }
    
    errors = {}
    
    for var_name, (cur_col, des_col) in variables.items():
        if cur_col in hover_df.columns and des_col in hover_df.columns:
            # Calculate error: Current - Desired
            error = hover_df[cur_col] - hover_df[des_col]
            errors[var_name] = error.values
        else:
            print(f"Warning: Columns {cur_col} or {des_col} not found in {filepath}")
            errors[var_name] = np.array([])
    
    return errors, hover_start, hover_end, len(hover_df), len(df)

def calculate_aggregate_statistics(all_errors):
    """
    Calculate aggregate statistics across all flights.
    
    Parameters:
    - all_errors: Dictionary where keys are variable names and values are lists of error arrays
    
    Returns a DataFrame with average error, standard deviation, and max error.
    """
    
    results = []
    
    for var_name, error_list in all_errors.items():
        # Concatenate all error arrays for this variable across all flights
        if error_list and any(len(e) > 0 for e in error_list):
            all_errors_concat = np.concatenate([e for e in error_list if len(e) > 0])
            
            # Calculate statistics across all flights
            avg_error = np.mean(all_errors_concat)
            std_dev = np.std(all_errors_concat)
            max_error = np.max(np.abs(all_errors_concat))
            
            results.append({
                'Variable': var_name,
                'Average Error': avg_error,
                'Error Standard Deviation': std_dev,
                'Max Error': max_error
            })
        else:
            print(f"No valid data for {var_name}")
    
    return pd.DataFrame(results)

def plot_all_flights_overview(flight_data):
    """
    Create a summary plot showing hovering periods across all flights.
    Y is the vertical axis in this coordinate system.
    """
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
        hover_start = data['hover_start']
        hover_end = data['hover_end']
        
        # Plot Y position (vertical) as it's most indicative of flight phase
        if 'Cur_Cluster_Y' in df.columns:
            ax.plot(df.index, df['Cur_Cluster_Y'], 'b-', alpha=0.7, label='Current Y', linewidth=2)
        if 'Des_Cluster_Y' in df.columns:
            ax.plot(df.index, df['Des_Cluster_Y'], 'r--', alpha=0.7, label='Desired Y')
        
        # Highlight hovering period with green
        ax.axvspan(hover_start, hover_end, alpha=0.3, color='green', label='Identified Hovering')
        
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
    
    plt.suptitle('Hovering Period Identification Across All Flights (Y=Vertical)', fontsize=14)
    plt.tight_layout()
    return fig

def plot_cluster_variables_and_errors(flight_data):
    """
    Create comprehensive plots showing all cluster variables and their errors during hovering.
    """
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
            hover_start = data['hover_start']
            hover_end = data['hover_end']
            hover_df = df.loc[hover_start:hover_end]
            
            cur_col = f'Cur_{var_base}'
            des_col = f'Des_{var_base}'
            
            if cur_col in hover_df.columns and des_col in hover_df.columns:
                # Calculate error during hovering
                time = np.arange(len(hover_df))
                error = hover_df[cur_col].values - hover_df[des_col].values
                
                # Plot error for this flight
                flight_name = os.path.basename(filepath).replace('.csv', '')
                ax.plot(time, error, alpha=0.7, label=f'Flight {flight_idx+1}', linewidth=0.8)
        
        ax.set_title(f'{label} Error During Hovering')
        ax.set_xlabel('Time (samples)')
        ax.set_ylabel('Error')
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        
        if idx == 0:  # Only show legend on first plot
            ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
    
    plt.suptitle('Cluster Variable Errors During Hovering (All Flights)', fontsize=14)
    plt.tight_layout()
    return fig

def plot_error_distributions(flight_data):
    """
    Create box plots showing error distributions for each variable across all flights.
    """
    # Collect all errors
    variables = {
        'X_c': ('Cur_Cluster_X', 'Des_Cluster_X'),
        'Y_c': ('Cur_Cluster_Y', 'Des_Cluster_Y'),
        'Z_c': ('Cur_Cluster_Z', 'Des_Cluster_Z'),
        'α': ('Cur_Alpha', 'Des_Alpha'),
        'β': ('Cur_Beta', 'Des_Beta'),
        'P': ('Cur_P', 'Des_P'),
        'φ₁': ('Cur_Phi1', 'Des_Phi1'),
        'φ₂': ('Cur_Phi2', 'Des_Phi2')
    }
    
    error_data = {var: [] for var in variables.keys()}
    
    for filepath, data in flight_data.items():
        df = data['df']
        hover_start = data['hover_start']
        hover_end = data['hover_end']
        hover_df = df.loc[hover_start:hover_end]
        
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

def process_multiple_flights(file_pattern='cluster_data*.csv', directory='.', debug=False):
    """
    Process multiple flight CSV files and calculate aggregate error statistics.
    
    Parameters:
    - file_pattern: Pattern to match CSV files (default: 'cluster_data*.csv')
    - directory: Directory containing the CSV files (default: current directory)
    - debug: If True, print detailed debugging information
    
    Returns:
    - DataFrame with aggregate error statistics
    - Dictionary with detailed flight information
    """
    
    # Find all matching CSV files
    search_path = os.path.join(directory, file_pattern)
    csv_files = glob.glob(search_path)
    
    if not csv_files:
        print(f"No files found matching pattern: {search_path}")
        return None, None
    
    print(f"Found {len(csv_files)} flight files to process:")
    for f in csv_files:
        print(f"  - {os.path.basename(f)}")
    print()
    
    # Initialize dictionary to collect all errors by variable
    all_errors = {
        'X_c (m)': [],  # Horizontal
        'Y_c (m)': [],  # Vertical  
        'Z_c (m)': [],  # Horizontal
        'α (rad)': [],
        'β (rad)': [],
        'P (m)': [],
        'φ₁ (rad)': [],
        'φ₂ (rad)': []
    }
    
    # Store flight data for visualization
    flight_data = {}
    
    # Process statistics
    total_hover_samples = 0
    total_samples = 0
    
    # Process each file
    for filepath in csv_files:
        filename = os.path.basename(filepath)
        print(f"Processing {filename}...")
        
        try:
            # Read the file for visualization
            df = pd.read_csv(filepath)
            
            # Collect errors from this file
            errors, hover_start, hover_end, hover_samples, total = collect_hover_errors_from_file(filepath)
            
            if debug:
                y_data = df['Cur_Cluster_Y'].values if 'Cur_Cluster_Y' in df.columns else df['Des_Cluster_Y'].values
                print(f"  Debug info:")
                print(f"    - Total samples: {total}")
                print(f"    - Y range: {y_data.min():.3f} to {y_data.max():.3f}")
                print(f"    - Hover Y altitude: ~{y_data[hover_start:hover_end+1].mean():.3f}")
                print(f"    - Hover period: {hover_start} to {hover_end} ({hover_samples} samples)")
                print(f"    - Percentage hovering: {hover_samples/total*100:.1f}%")
            
            # Add to aggregate collection
            for var_name in all_errors.keys():
                if var_name in errors:
                    all_errors[var_name].append(errors[var_name])
            
            # Store flight data
            flight_data[filepath] = {
                'df': df,
                'hover_start': hover_start,
                'hover_end': hover_end,
                'hover_samples': hover_samples
            }
            
            total_hover_samples += hover_samples
            total_samples += total
            
            if not debug:
                print(f"  - Hovering period: samples {hover_start} to {hover_end} ({hover_samples} samples)")
            
        except Exception as e:
            print(f"  - Error processing file: {e}")
    
    print(f"\nTotal hovering samples analyzed: {total_hover_samples}")
    print(f"Total samples across all flights: {total_samples}")
    print(f"Percentage in hover: {total_hover_samples/total_samples*100:.1f}%\n")
    
    # Calculate aggregate statistics
    print("Calculating aggregate statistics across all flights...")
    error_stats = calculate_aggregate_statistics(all_errors)
    
    return error_stats, flight_data

def main(file_pattern='cluster_data*.csv', directory='.', debug=False):
    """
    Main function to process multiple cluster data files and generate aggregate error statistics.
    
    Note: This code assumes Y is the vertical axis (altitude) in the coordinate system.
    
    Parameters:
    - file_pattern: Pattern to match CSV files (default: 'cluster_data*.csv')
    - directory: Directory containing the CSV files (default: current directory)
    - debug: If True, print detailed debugging information
    """
    
    # Process all flights
    error_stats, flight_data = process_multiple_flights(file_pattern, directory, debug=debug)
    
    if error_stats is None:
        return None, None
    
    # Format the table for display
    print("\n" + "="*80)
    print("AGGREGATE ERROR STATISTICS FOR CLUSTER VARIABLES (HOVERING ONLY)")
    print(f"Calculated from {len(flight_data)} flights")
    print("="*80)
    
    # Create formatted table similar to the image
    formatted_df = error_stats.set_index('Variable')
    formatted_df = formatted_df.round(4)  # Round to 4 decimal places
    
    print(formatted_df.to_string())
    print("="*80)
    
    # Create visualizations
    print("\nGenerating visualizations...")
    
    # 1. Hovering period identification
    fig1 = plot_all_flights_overview(flight_data)
    
    # 2. Error time series for all variables
    fig2 = plot_cluster_variables_and_errors(flight_data)
    
    # 3. Error distributions
    fig3 = plot_error_distributions(flight_data)
    
    plt.show()
    
    # Save results to CSV
    # prtprt
    
    # Create per-flight statistics for comparison
    per_flight_stats = []
    for filepath, data in flight_data.items():
        flight_errors, _, _, _, _ = collect_hover_errors_from_file(filepath)
        flight_name = os.path.basename(filepath)
        
        for var_name, errors in flight_errors.items():
            if len(errors) > 0:
                per_flight_stats.append({
                    'Flight': flight_name,
                    'Variable': var_name,
                    'Average Error': np.mean(errors),
                    'Std Dev': np.std(errors),
                    'Max Error': np.max(np.abs(errors))
                })
    
    per_flight_df = pd.DataFrame(per_flight_stats)
    per_flight_file = 'per_flight_error_statistics.csv'
    # per_flight_df.to_csv(per_flight_file, index=False)
    # print(f"Per-flight statistics saved to {per_flight_file}")
    
    # LaTeX table format
    print("\nLaTeX Table Format (Aggregate Statistics):")
    print("\\begin{tabular}{|c|c|c|c|}")
    print("\\hline")
    print("& Average & Error Standard & \\\\")
    print("& Error & Deviation & Max Error \\\\")
    print("\\hline")
    
    for _, row in error_stats.iterrows():
        var = row['Variable'].replace('_', '\\_')
        print(f"{var} & {row['Average Error']:.4f} & {row['Error Standard Deviation']:.4f} & {row['Max Error']:.4f} \\\\")
    
    print("\\hline")
    print("\\end{tabular}")
    
    return error_stats, flight_data

# Run the analysis
if __name__ == "__main__":
    # Example usage:
    # Process all files matching 'cluster_data*.csv' in current directory
    # error_stats, flight_data = main('cluster_data*.csv', '.', debug=False)
    directory = os.path.expanduser('~/Desktop/crazyflie_codebase/cluster_data/cluster_hover')
    # Or with debug mode to see more details:
    # error_stats, flight_data = main('cluster_data*.csv', '.', debug=True)
    
    # Or process files in a specific directory:
    error_stats, flight_data = main('cluster_data*.csv', directory)
    
    # Or process files with a different pattern:
    # error_stats, flight_data = main('flight_*.csv', '.')