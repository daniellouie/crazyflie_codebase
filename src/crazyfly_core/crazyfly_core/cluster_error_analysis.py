import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import glob
import os
from pathlib import Path

# COORDINATE SYSTEM NOTE:
# Y-axis is vertical (altitude)
# X and Z are horizontal planes

class ClusterFlightAnalyzer:
    """Unified analyzer for hover, dynamic, and P-changing flights"""
    
    def __init__(self, flight_type='dynamic'):
        """
        Initialize analyzer with flight type.
        
        Parameters:
        -----------
        flight_type : str
            'hover' - Static hovering flights
            'dynamic' - Dynamic trajectory following
            'p_change' - Flights with P parameter changes
        """
        self.flight_type = flight_type
        self.analysis_functions = {
            'hover': self.identify_hovering_period,
            'dynamic': self.identify_dynamic_period,
            'p_change': self.identify_p_change_period
        }
    
    def identify_analysis_period(self, df):
        """Route to appropriate analysis based on flight type"""
        if self.flight_type not in self.analysis_functions:
            print(f"Unknown flight type: {self.flight_type}, using dynamic")
            return self.identify_dynamic_period(df)
        
        return self.analysis_functions[self.flight_type](df)
    
    def identify_hovering_period(self, df):
        """
        Identify true hovering period for static hover flights.
        Uses Y-band detection with P stability check.
        """
        y_position = df['Cur_Cluster_Y'].values.copy()
        
        # Find hover altitude
        high_values = y_position[y_position > np.percentile(y_position, 70)]
        if len(high_values) > 0:
            hover_altitude = np.median(high_values)
        else:
            hover_altitude = np.max(y_position) * 0.95
        
        # Find stable period at hover altitude (±5% band)
        altitude_band = 0.05 * hover_altitude
        within_band = np.abs(y_position - hover_altitude) <= altitude_band
        
        # Find longest continuous segment
        segments = []
        in_segment = False
        start_idx = 0
        
        for i in range(len(within_band)):
            if within_band[i] and not in_segment:
                start_idx = i
                in_segment = True
            elif not within_band[i] and in_segment:
                if i - start_idx >= 30:
                    segments.append((start_idx, i-1))
                in_segment = False
        
        if in_segment and len(within_band) - start_idx >= 30:
            segments.append((start_idx, len(within_band)-1))
        
        if segments:
            # Merge close segments
            merged = []
            if len(segments) > 1:
                current_start = segments[0][0]
                current_end = segments[0][1]
                
                for i in range(1, len(segments)):
                    if segments[i][0] - current_end < 20:
                        current_end = segments[i][1]
                    else:
                        merged.append((current_start, current_end))
                        current_start = segments[i][0]
                        current_end = segments[i][1]
                
                merged.append((current_start, current_end))
                segments = merged
            
            # Get longest segment
            longest = max(segments, key=lambda x: x[1] - x[0])
            start, end = longest
            
            # Check P stability
            if 'Cur_P' in df.columns and 'Des_P' in df.columns:
                p_error = np.abs(df['Cur_P'].iloc[start:end+1] - df['Des_P'].iloc[start:end+1])
                # Find where P stabilizes
                for i in range(len(p_error)):
                    if p_error.iloc[i] < 0.2:
                        start = start + i
                        break
            
            # Trim landing
            for i in range(end, max(start, end - 10), -1):
                if i + 5 < len(y_position):
                    if y_position[i] - y_position[min(i+5, len(y_position)-1)] > 0.1:
                        end = i
                        break
            
            return start, end
        
        return 20, len(df) - 20  # Fallback
    
    def identify_dynamic_period(self, df):
        """
        Identify stable tracking period for dynamic trajectory flights.
        Waits for P convergence and excludes landing.
        """
        # Check for required columns
        required_cols = ['Cur_Cluster_Y', 'Cur_P', 'Des_P']
        for col in required_cols:
            if col not in df.columns:
                return 20, len(df) - 20
        
        p_error = np.abs(df['Cur_P'].values - df['Des_P'].values)
        y_position = df['Cur_Cluster_Y'].values
        
        # Find when P stabilizes AND altitude is reached
        start_idx = 20  # Minimum startup trim
        for i in range(start_idx, min(100, len(df))):
            if p_error[i] < 0.2 and y_position[i] > 0.5:
                if i + 10 < len(df):
                    if np.mean(p_error[i:i+10]) < 0.2:
                        start_idx = i
                        break
        
        # Find landing start
        end_idx = len(df) - 1
        for i in range(len(df) - 20, start_idx, -1):
            if y_position[i] > 0.8:
                end_idx = i
                break
        
        # Additional X tracking check
        if 'Cur_Cluster_X' in df.columns and 'Des_Cluster_X' in df.columns:
            x_error = np.abs(df['Cur_Cluster_X'].values - df['Des_Cluster_X'].values)
            for i in range(start_idx, min(start_idx + 30, end_idx)):
                if x_error[i] < 0.2:
                    start_idx = i
                    break
        
        return start_idx, end_idx
    
    def identify_p_change_period(self, df):
        """
        Identify analysis period for P-changing flights.
        Options:
        1. Include entire maneuver after initial convergence
        2. Analyze only steady-state periods between P changes
        """
        # Check if P actually changes
        if 'Des_P' not in df.columns:
            print("No Des_P column, falling back to dynamic analysis")
            return self.identify_dynamic_period(df)
        
        des_p = df['Des_P'].values
        p_range = np.max(des_p) - np.min(des_p)
        
        # If P doesn't change much, treat as dynamic
        if p_range < 0.1:
            print(f"Small P change ({p_range:.3f}m), using dynamic analysis")
            return self.identify_dynamic_period(df)
        
        # Option 1: Include entire maneuver (after initial convergence)
        # This is simpler and gives overall maneuver performance
        p_error = np.abs(df['Cur_P'].values - des_p)
        y_position = df['Cur_Cluster_Y'].values
        
        # Wait for initial convergence (before P starts changing)
        start_idx = 20
        for i in range(start_idx, min(100, len(df))):
            if p_error[i] < 0.3 and y_position[i] > 0.5:
                start_idx = i
                break
        
        # Find end (before landing)
        end_idx = len(df) - 1
        for i in range(len(df) - 20, start_idx, -1):
            if y_position[i] > 0.8:
                end_idx = i
                break
        
        # Option 2 (commented): Analyze only steady-state periods
        # Uncomment below to exclude P-change transients
        """
        # Find P change points
        p_changes = np.abs(np.diff(des_p)) > 0.05
        change_indices = np.where(p_changes)[0]
        
        if len(change_indices) > 0:
            # Analyze period after last P change (steady state)
            last_change = change_indices[-1]
            settling_time = 40  # samples for P to stabilize
            
            if last_change + settling_time < end_idx - 20:
                # Wait for P to settle after change
                start_idx = last_change + settling_time
                
                # Verify P has stabilized
                for i in range(start_idx, min(start_idx + 30, end_idx)):
                    if p_error[i] < 0.2:
                        start_idx = i
                        break
        """
        
        return start_idx, end_idx
    
    def collect_errors(self, filepath):
        """Process a single CSV file and collect error data"""
        df = pd.read_csv(filepath)
        
        # Get analysis period
        start, end = self.identify_analysis_period(df)
        
        # Validate period
        if end <= start:
            print(f"Warning: Invalid period for {os.path.basename(filepath)}")
            start = 20
            end = len(df) - 20
        
        # Extract analysis period
        analysis_df = df.loc[start:end].copy()
        
        # Define variables
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
    """Calculate statistics across all flights"""
    results = []
    
    for var_name, error_list in all_errors.items():
        if error_list and any(len(e) > 0 for e in error_list):
            all_errors_concat = np.concatenate([e for e in error_list if len(e) > 0])
            
            results.append({
                'Variable': var_name,
                'Average Error': np.mean(all_errors_concat),
                'Std Dev': np.std(all_errors_concat),
                'Max Error': np.max(np.abs(all_errors_concat)),
                'RMSE': np.sqrt(np.mean(all_errors_concat**2))
            })
    
    return pd.DataFrame(results)

def plot_analysis_overview(flight_data, title=""):
    """Plot showing analysis periods for each flight"""
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
        
        # Plot Y trajectories
        ax.plot(df.index, df['Cur_Cluster_Y'], 'b-', alpha=0.7, label='Current Y', linewidth=2)
        ax.plot(df.index, df['Des_Cluster_Y'], 'r--', alpha=0.7, label='Desired Y')
        
        # For P-change flights, also show P
        if 'flight_type' in data and data['flight_type'] == 'p_change':
            ax2 = ax.twinx()
            ax2.plot(df.index, df['Des_P'], 'g-', alpha=0.5, label='Des P')
            ax2.set_ylabel('P (m)', color='g')
            ax2.tick_params(axis='y', labelcolor='g')
        
        # Highlight analysis period
        ax.axvspan(start, end, alpha=0.3, color='green', label='Analysis Period')
        ax.axvline(x=start, color='green', linestyle=':', alpha=0.8, linewidth=2)
        ax.axvline(x=end, color='green', linestyle=':', alpha=0.8, linewidth=2)
        
        ax.set_title(f'{os.path.basename(filepath)}\nAnalysis: {start}-{end}', fontsize=10)
        ax.set_xlabel('Sample Index')
        ax.set_ylabel('Y Position (m) - Vertical')
        ax.legend(loc='upper left', fontsize=8)
        ax.grid(True, alpha=0.3)
    
    for idx in range(n_flights, len(axes)):
        axes[idx].axis('off')
    
    plt.suptitle(f'Analysis Periods - {title}', fontsize=14)
    plt.tight_layout()
    return fig

def plot_error_timeseries(flight_data):
    """Plot error time series for all variables"""
    variables = [
        ('Cluster_X', 'X (m)'), ('Cluster_Y', 'Y (m)'), ('Cluster_Z', 'Z (m)'),
        ('Alpha', 'α (rad)'), ('Beta', 'β (rad)'), ('P', 'P (m)'),
        ('Phi1', 'φ₁ (rad)'), ('Phi2', 'φ₂ (rad)')
    ]
    
    fig, axes = plt.subplots(4, 2, figsize=(15, 12))
    axes = axes.flatten()
    
    for idx, (var_base, label) in enumerate(variables):
        ax = axes[idx]
        
        for flight_idx, (filepath, data) in enumerate(flight_data.items()):
            df = data['df']
            start = data['analysis_start']
            end = data['analysis_end']
            analysis_df = df.loc[start:end]
            
            cur_col = f'Cur_{var_base}'
            des_col = f'Des_{var_base}'
            
            if cur_col in analysis_df.columns and des_col in analysis_df.columns:
                time = np.arange(len(analysis_df))
                error = analysis_df[cur_col].values - analysis_df[des_col].values
                ax.plot(time, error, alpha=0.7, label=f'Flight {flight_idx+1}', linewidth=0.8)
        
        ax.set_title(f'{label} Error')
        ax.set_xlabel('Time (samples)')
        ax.set_ylabel('Error')
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        
        if idx == 0:
            ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
    
    plt.suptitle('Tracking Errors', fontsize=14)
    plt.tight_layout()
    return fig

def process_flights(directory, file_pattern='cluster_data*.csv', 
                   flight_type='dynamic', debug=False):
    """Process multiple flight files"""
    
    analyzer = ClusterFlightAnalyzer(flight_type)
    
    search_path = os.path.join(directory, file_pattern)
    csv_files = glob.glob(search_path)
    
    if not csv_files:
        print(f"No files found: {search_path}")
        return None, None
    
    print(f"\n{'='*60}")
    print(f"Processing {len(csv_files)} files as {flight_type.upper()} flights")
    print(f"{'='*60}")
    
    all_errors = {
        'X_c (m)': [], 'Y_c (m)': [], 'Z_c (m)': [],
        'α (rad)': [], 'β (rad)': [], 'P (m)': [],
        'φ₁ (rad)': [], 'φ₂ (rad)': []
    }
    
    flight_data = {}
    total_analysis_samples = 0
    total_samples = 0
    
    for filepath in csv_files:
        filename = os.path.basename(filepath)
        print(f"Processing {filename}...")
        
        try:
            df = pd.read_csv(filepath)
            errors, start, end, analysis_samples, total = analyzer.collect_errors(filepath)
            
            if debug:
                print(f"  Analysis: samples {start}-{end} ({analysis_samples}/{total})")
                if flight_type == 'p_change' and 'Des_P' in df.columns:
                    p_range = df['Des_P'].max() - df['Des_P'].min()
                    print(f"  P range: {p_range:.3f}m")
                if 'Cur_P' in df.columns and 'Des_P' in df.columns:
                    p_err = abs(df.loc[start, 'Cur_P'] - df.loc[start, 'Des_P'])
                    print(f"  P error at start: {p_err:.3f}m")
            
            for var_name in all_errors.keys():
                if var_name in errors:
                    all_errors[var_name].append(errors[var_name])
            
            flight_data[filepath] = {
                'df': df,
                'analysis_start': start,
                'analysis_end': end,
                'analysis_samples': analysis_samples,
                'flight_type': flight_type
            }
            
            total_analysis_samples += analysis_samples
            total_samples += total
            
        except Exception as e:
            print(f"  Error: {e}")
    
    print(f"\nTotal analyzed: {total_analysis_samples}/{total_samples} samples "
          f"({total_analysis_samples/total_samples*100:.1f}%)")
    
    error_stats = calculate_statistics(all_errors)
    return error_stats, flight_data

def main(directory='.', flight_type='dynamic', file_pattern='cluster_data*.csv',
         save_results=True, show_plots=True, debug=False):
    """
    Main analysis function for all flight types.
    
    Parameters:
    -----------
    directory : str
        Directory containing CSV files
    flight_type : str
        'hover' - Static hovering flights
        'dynamic' - Dynamic trajectory flights  
        'p_change' - Flights with P parameter changes
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
        directory=directory,
        file_pattern=file_pattern,
        flight_type=flight_type,
        debug=debug
    )
    
    if error_stats is None:
        return None, None
    
    # Display results
    print("\n" + "="*70)
    print(f"ERROR STATISTICS - {flight_type.upper()} FLIGHTS")
    print(f"Analyzed {len(flight_data)} flights")
    print("="*70)
    
    formatted_df = error_stats.set_index('Variable').round(4)
    print(formatted_df.to_string())
    print("="*70)
    
    # LaTeX output
    print("\nLaTeX Table Format:")
    print("\\begin{tabular}{|c|c|c|c|c|}")
    print("\\hline")
    print("Variable & Avg Error & Std Dev & Max Error & RMSE \\\\")
    print("\\hline")
    for _, row in error_stats.iterrows():
        var = row['Variable'].replace('_', '\\_')
        print(f"{var} & {row['Average Error']:.4f} & {row['Std Dev']:.4f} & "
              f"{row['Max Error']:.4f} & {row['RMSE']:.4f} \\\\")
    print("\\hline")
    print("\\end{tabular}")
    
    # Save results
    if save_results:
        output_file = f'{flight_type}_error_statistics.csv'
        error_stats.to_csv(output_file, index=False)
        print(f"\nResults saved to {output_file}")
    
    # Generate plots
    if show_plots:
        fig1 = plot_analysis_overview(flight_data, flight_type.replace('_', ' ').title())
        fig2 = plot_error_timeseries(flight_data)
        plt.show()
    
    return error_stats, flight_data

# ============================================================================
# USAGE EXAMPLES
# ============================================================================

if __name__ == "__main__":
    
    # ========================================
    # OPTION 1: HOVERING FLIGHTS
    # ========================================
    directory = os.path.expanduser('~/Desktop/crazyflie_codebase/cluster_data/cluster_hover')
    error_stats, flight_data = main(
        directory=directory,
        flight_type='hover',
        debug=True
    )
    
    # ========================================
    # OPTION 2: DYNAMIC TRAJECTORY FLIGHTS
    # ========================================
    # directory = os.path.expanduser('~/Desktop/crazyflie_codebase/cluster_data/cluster_dynamic')
    # error_stats, flight_data = main(
    #     directory=directory,
    #     flight_type='dynamic',
    #     debug=True
    # )
    
    # ========================================
    # OPTION 3: P-CHANGING FLIGHTS
    # ========================================
    # directory = os.path.expanduser('~/Desktop/crazyflie_codebase/cluster_data/cluster_shrink')
    # error_stats, flight_data = main(
    #     directory=directory,
    #     flight_type='p_change',  # <-- For your P-changing flights
    #     debug=True
    # )
    
    # ========================================
    # COMPARE DIFFERENT ANALYSES
    # ========================================
    # You can analyze the same data with different methods:
    
    # for ftype in ['dynamic', 'p_change']:
    #     print(f"\n\n{'='*60}")
    #     print(f"Analyzing as {ftype.upper()}")
    #     print(f"{'='*60}")
    #     stats, data = main(
    #         directory=directory,
    #         flight_type=ftype,
    #         show_plots=(ftype == 'p_change')  # Only show plots for last one
    #     )