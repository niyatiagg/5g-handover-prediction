#!/usr/bin/env python3
"""
Data Analysis Script for 5G Handover Simulation
This script analyzes the generated CSV files and provides insights
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import sys
import os

def load_data(handover_file='handover_dataset.csv', flow_file='flow_statistics.csv'):
    """Load simulation data from CSV files"""
    try:
        handover_df = pd.read_csv(handover_file)
        print(f"✓ Loaded {len(handover_df)} records from {handover_file}")
    except FileNotFoundError:
        print(f"✗ Error: {handover_file} not found")
        handover_df = None
    
    try:
        flow_df = pd.read_csv(flow_file)
        print(f"✓ Loaded {len(flow_df)} flows from {flow_file}")
    except FileNotFoundError:
        print(f"✗ Warning: {flow_file} not found")
        flow_df = None
    
    return handover_df, flow_df

def analyze_handovers(df):
    """Analyze handover events"""
    if df is None:
        return
    
    print("\n" + "="*60)
    print("HANDOVER ANALYSIS")
    print("="*60)
    
    # Filter handover events
    handovers = df[df['Handover_Type'] == 'HANDOVER']
    measurements = df[df['Handover_Type'] == 'MEASUREMENT']
    
    print(f"\nTotal handover events: {len(handovers)}")
    print(f"Total measurement records: {len(measurements)}")
    print(f"Number of unique UEs: {df['UE_ID'].nunique()}")
    print(f"Number of unique gNBs: {df['New_gNB_ID'].nunique()}")
    
    if len(handovers) > 0:
        print(f"\nHandover Statistics:")
        print(f"  Average handovers per UE: {len(handovers) / df['UE_ID'].nunique():.2f}")
        print(f"  Handover rate: {len(handovers) / df['Time'].max():.2f} handovers/second")
        
        # Handovers per UE
        ho_per_ue = handovers.groupby('UE_ID').size()
        print(f"\nHandovers per UE:")
        print(f"  Min: {ho_per_ue.min()}")
        print(f"  Max: {ho_per_ue.max()}")
        print(f"  Mean: {ho_per_ue.mean():.2f}")
        print(f"  Median: {ho_per_ue.median():.2f}")
        
        # Handovers per gNB
        ho_per_gnb = handovers.groupby('New_gNB_ID').size()
        print(f"\nHandovers per gNB (target cell):")
        print(f"  Min: {ho_per_gnb.min()}")
        print(f"  Max: {ho_per_gnb.max()}")
        print(f"  Mean: {ho_per_gnb.mean():.2f}")
        print(f"  Median: {ho_per_gnb.median():.2f}")
        
        # RSRP analysis
        if 'RSRP_Old' in handovers.columns and handovers['RSRP_Old'].notna().any():
            print(f"\nRSRP Statistics (old cell):")
            print(f"  Mean: {handovers['RSRP_Old'].mean():.2f} dBm")
            print(f"  Std: {handovers['RSRP_Old'].std():.2f} dBm")
            print(f"  Min: {handovers['RSRP_Old'].min():.2f} dBm")
            print(f"  Max: {handovers['RSRP_Old'].max():.2f} dBm")
        
        if 'RSRP_New' in handovers.columns and handovers['RSRP_New'].notna().any():
            print(f"\nRSRP Statistics (new cell):")
            print(f"  Mean: {handovers['RSRP_New'].mean():.2f} dBm")
            print(f"  Std: {handovers['RSRP_New'].std():.2f} dBm")
            print(f"  Min: {handovers['RSRP_New'].min():.2f} dBm")
            print(f"  Max: {handovers['RSRP_New'].max():.2f} dBm")
    
    return handovers, measurements

def analyze_mobility(df):
    """Analyze UE mobility patterns"""
    if df is None:
        return
    
    print("\n" + "="*60)
    print("MOBILITY ANALYSIS")
    print("="*60)
    
    # Speed statistics
    if 'Speed' in df.columns and df['Speed'].notna().any():
        speeds = df[df['Speed'] > 0]['Speed']
        print(f"\nSpeed Statistics:")
        print(f"  Mean: {speeds.mean():.2f} m/s ({speeds.mean() * 3.6:.2f} km/h)")
        print(f"  Std: {speeds.std():.2f} m/s")
        print(f"  Min: {speeds.min():.2f} m/s ({speeds.min() * 3.6:.2f} km/h)")
        print(f"  Max: {speeds.max():.2f} m/s ({speeds.max() * 3.6:.2f} km/h)")
    
    # Position statistics
    if 'X_Position' in df.columns and 'Y_Position' in df.columns:
        print(f"\nPosition Statistics:")
        print(f"  X range: [{df['X_Position'].min():.2f}, {df['X_Position'].max():.2f}] m")
        print(f"  Y range: [{df['Y_Position'].min():.2f}, {df['Y_Position'].max():.2f}] m")

def analyze_throughput(flow_df):
    """Analyze traffic throughput"""
    if flow_df is None:
        return
    
    print("\n" + "="*60)
    print("THROUGHPUT ANALYSIS")
    print("="*60)
    
    if 'Throughput_DL_Mbps' in flow_df.columns:
        print(f"\nDownlink Throughput:")
        print(f"  Mean: {flow_df['Throughput_DL_Mbps'].mean():.4f} Mbps")
        print(f"  Std: {flow_df['Throughput_DL_Mbps'].std():.4f} Mbps")
        print(f"  Min: {flow_df['Throughput_DL_Mbps'].min():.4f} Mbps")
        print(f"  Max: {flow_df['Throughput_DL_Mbps'].max():.4f} Mbps")
    
    if 'Throughput_UL_Mbps' in flow_df.columns:
        print(f"\nUplink Throughput:")
        print(f"  Mean: {flow_df['Throughput_UL_Mbps'].mean():.4f} Mbps")
        print(f"  Std: {flow_df['Throughput_UL_Mbps'].std():.4f} Mbps")
        print(f"  Min: {flow_df['Throughput_UL_Mbps'].min():.4f} Mbps")
        print(f"  Max: {flow_df['Throughput_UL_Mbps'].max():.4f} Mbps")
    
    if 'Delay_Mean_ms' in flow_df.columns:
        print(f"\nDelay Statistics:")
        print(f"  Mean: {flow_df['Delay_Mean_ms'].mean():.4f} ms")
        print(f"  Std: {flow_df['Delay_Mean_ms'].std():.4f} ms")
        print(f"  Min: {flow_df['Delay_Mean_ms'].min():.4f} ms")
        print(f"  Max: {flow_df['Delay_Mean_ms'].max():.4f} ms")

def plot_handover_timeline(handovers, output_file='handover_timeline.png'):
    """Plot handover events over time"""
    if handovers is None or len(handovers) == 0:
        print("No handover events to plot")
        return
    
    plt.figure(figsize=(12, 6))
    plt.scatter(handovers['Time'], handovers['UE_ID'], alpha=0.6, s=50)
    plt.xlabel('Time (seconds)', fontsize=12)
    plt.ylabel('UE ID', fontsize=12)
    plt.title('Handover Events Timeline', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"\n✓ Saved plot to {output_file}")
    plt.close()

def plot_handover_distribution(handovers, output_file='handover_distribution.png'):
    """Plot handover distribution"""
    if handovers is None or len(handovers) == 0:
        return
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Handovers per UE
    ho_per_ue = handovers.groupby('UE_ID').size()
    axes[0].bar(ho_per_ue.index, ho_per_ue.values, alpha=0.7, color='steelblue')
    axes[0].set_xlabel('UE ID', fontsize=12)
    axes[0].set_ylabel('Number of Handovers', fontsize=12)
    axes[0].set_title('Handovers per UE', fontsize=13, fontweight='bold')
    axes[0].grid(True, alpha=0.3, axis='y')
    
    # Handovers per gNB
    ho_per_gnb = handovers.groupby('New_gNB_ID').size()
    axes[1].bar(ho_per_gnb.index, ho_per_gnb.values, alpha=0.7, color='coral')
    axes[1].set_xlabel('gNB ID', fontsize=12)
    axes[1].set_ylabel('Number of Handovers', fontsize=12)
    axes[1].set_title('Handovers per gNB (Target Cell)', fontsize=13, fontweight='bold')
    axes[1].grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"✓ Saved plot to {output_file}")
    plt.close()

def plot_mobility_pattern(df, output_file='mobility_pattern.png'):
    """Plot UE mobility pattern"""
    if df is None or len(df) == 0:
        return
    
    plt.figure(figsize=(10, 8))
    
    # Plot UE trajectories
    for ue_id in df['UE_ID'].unique():
        ue_data = df[df['UE_ID'] == ue_id]
        if len(ue_data) > 0 and 'X_Position' in ue_data.columns and 'Y_Position' in ue_data.columns:
            plt.plot(ue_data['X_Position'], ue_data['Y_Position'], 
                    alpha=0.3, linewidth=1, label=f'UE {ue_id}' if ue_id <= 3 else '')
    
    # Mark handover events
    handovers = df[df['Handover_Type'] == 'HANDOVER']
    if len(handovers) > 0 and 'X_Position' in handovers.columns:
        plt.scatter(handovers['X_Position'], handovers['Y_Position'], 
                   c='red', marker='x', s=100, label='Handover Event', zorder=5)
    
    plt.xlabel('X Position (meters)', fontsize=12)
    plt.ylabel('Y Position (meters)', fontsize=12)
    plt.title('UE Mobility Pattern and Handover Events', fontsize=14, fontweight='bold')
    plt.legend(loc='upper right', fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"✓ Saved plot to {output_file}")
    plt.close()

def main():
    """Main analysis function"""
    print("="*60)
    print("5G Handover Simulation Data Analysis")
    print("="*60)
    
    # Load data
    handover_df, flow_df = load_data()
    
    if handover_df is None:
        print("\n✗ Error: Cannot proceed without handover data")
        sys.exit(1)
    
    # Analyze data
    handovers, measurements = analyze_handovers(handover_df)
    analyze_mobility(handover_df)
    analyze_throughput(flow_df)
    
    # Generate plots
    print("\n" + "="*60)
    print("GENERATING PLOTS")
    print("="*60)
    
    plot_handover_timeline(handovers)
    plot_handover_distribution(handovers)
    plot_mobility_pattern(handover_df)
    
    print("\n" + "="*60)
    print("ANALYSIS COMPLETE")
    print("="*60)
    print("\nGenerated files:")
    print("  - handover_timeline.png")
    print("  - handover_distribution.png")
    print("  - mobility_pattern.png")

if __name__ == '__main__':
    main()

