import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
from pathlib import Path
import argparse
from typing import List, Dict, Tuple
import glob

class ROS2NetworkAnalyzer:
    def __init__(self, directory: str, topics_of_interest: List[str] = None):
        """
        Initialize the analyzer with a directory containing CSV files.
        
        Args:
            directory: Directory path containing CSV files
            topics_of_interest: List of specific topic names to always include
        """
        self.directory = Path(directory)
        self.topics_of_interest = topics_of_interest or [
            "/camera/color/image_raw",
            "/camera/aligned_depth_to_color/image_raw", 
            "/camera/infra1/image_rect_raw",
            "/camera/infra2/image_rect_raw",
            "/imu/data",
            "/pointcloud",
            "/joint_states"
        ]
        self.data = {}
        self.combined_data = None
        
    def load_data(self):
        """Load data from all CSV files in the directory."""
        csv_files = list(self.directory.glob("*.csv"))
        
        if not csv_files:
            print(f"No CSV files found in {self.directory}")
            return
            
        print(f"Found {len(csv_files)} CSV files in {self.directory}")
        
        for file_path in csv_files:
            dataset_name = file_path.stem
            try:
                df = pd.read_csv(file_path)
                # Clean column names (remove quotes if present)
                df.columns = df.columns.str.strip().str.replace('"', '')
                
                # Filter out zero-frequency topics EXCEPT topics of interest
                df_filtered = df[
                    (df['Frequency (Hz)'] > 0) | 
                    (df['Topic Name'].isin(self.topics_of_interest))
                ].copy()
                
                df_filtered['Dataset'] = dataset_name
                self.data[dataset_name] = df_filtered
                print(f"Loaded {len(df_filtered)} topics from {dataset_name} (filtered from {len(df)} total)")
            except Exception as e:
                print(f"Error loading {file_path}: {e}")
        
        # Combine all data
        if self.data:
            self.combined_data = pd.concat(self.data.values(), ignore_index=True)
            print(f"Total combined records: {len(self.combined_data)}")
    
    def get_top_bandwidth_topics_per_dataset(self, n: int = 3) -> Dict[str, pd.Series]:
        """Get top N topics by bandwidth for each dataset."""
        top_topics_dict = {}
        
        for dataset_name, df in self.data.items():
            # Get top N topics by bandwidth, excluding zero bandwidth
            top_topics = df[df['Bandwidth (KiB/s)'] > 0].nlargest(n, 'Bandwidth (KiB/s)')['Topic Name'].tolist()
            
            # Add topics of interest that exist in this dataset
            topics_of_interest_in_dataset = df[df['Topic Name'].isin(self.topics_of_interest)]['Topic Name'].tolist()
            
            # Combine and remove duplicates while preserving order
            combined_topics = top_topics.copy()
            for topic in topics_of_interest_in_dataset:
                if topic not in combined_topics:
                    combined_topics.append(topic)
            
            # Get the data for these topics
            selected_data = df[df['Topic Name'].isin(combined_topics)].set_index('Topic Name')['Bandwidth (KiB/s)']
            top_topics_dict[dataset_name] = selected_data
            
        return top_topics_dict
    
    def plot_bandwidth_analysis(self, figsize: Tuple[int, int] = None):
        """Create bandwidth analysis plots based on number of datasets."""
        num_datasets = len(self.data)
        
        if num_datasets == 0:
            print("No data to plot")
            return None
        
        top_topics_dict = self.get_top_bandwidth_topics_per_dataset()
        
        if num_datasets <= 3:
            # Grouped bar chart
            if figsize is None:
                figsize = (15, 8)
            fig, ax = plt.subplots(1, 1, figsize=figsize)
            
            # Get all unique topics across datasets
            all_topics = set()
            for topics in top_topics_dict.values():
                all_topics.update(topics.index)
            all_topics = sorted(list(all_topics))
            
            # Create grouped bar chart
            x = np.arange(len(all_topics))
            width = 0.8 / num_datasets
            
            colors = plt.cm.Set3(np.linspace(0, 1, num_datasets))
            
            for i, (dataset_name, topics_data) in enumerate(top_topics_dict.items()):
                values = [topics_data.get(topic, 0) for topic in all_topics]
                bars = ax.bar(x + i * width, values, width, label=dataset_name, color=colors[i], alpha=0.8)
                
                # Add value labels on bars
                for bar, value in zip(bars, values):
                    if value > 0:
                        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01*max([max(td.values) for td in top_topics_dict.values()]),
                               f'{value:.1f}', ha='center', va='bottom', fontsize=8)
            
            ax.set_xlabel('Topics')
            ax.set_ylabel('Bandwidth (KiB/s)')
            ax.set_title('Top Bandwidth Topics + Topics of Interest')
            ax.set_xticks(x + width * (num_datasets - 1) / 2)
            ax.set_xticklabels([topic.split('/')[-1] for topic in all_topics], rotation=45, ha='right')
            ax.legend()
            ax.grid(True, alpha=0.3, axis='y')
            
            # Highlight topics of interest with different border
            for i, topic in enumerate(all_topics):
                if topic in self.topics_of_interest:
                    for j in range(num_datasets):
                        rect = plt.Rectangle((x[i] + j * width, 0), width, 
                                           ax.get_ylim()[1], fill=False, 
                                           edgecolor='red', linewidth=2, linestyle='--', alpha=0.7)
                        ax.add_patch(rect)
        
        else:
            # Subplots for each dataset
            if figsize is None:
                cols = min(3, num_datasets)
                rows = (num_datasets + cols - 1) // cols
                figsize = (5 * cols, 4 * rows)
            
            fig, axes = plt.subplots(rows, cols, figsize=figsize)
            if num_datasets == 1:
                axes = [axes]
            elif rows == 1:
                axes = axes
            else:
                axes = axes.flatten()
            
            for idx, (dataset_name, topics_data) in enumerate(top_topics_dict.items()):
                ax = axes[idx]
                
                # Create colors: red for topics of interest, blue for others
                colors = ['red' if topic in self.topics_of_interest else 'skyblue' 
                         for topic in topics_data.index]
                
                bars = ax.bar(range(len(topics_data)), topics_data.values, color=colors, alpha=0.7)
                ax.set_title(f'{dataset_name}')
                ax.set_ylabel('Bandwidth (KiB/s)')
                ax.set_xticks(range(len(topics_data)))
                ax.set_xticklabels([topic.split('/')[-1] for topic in topics_data.index], 
                                  rotation=45, ha='right')
                ax.grid(True, alpha=0.3, axis='y')
                
                # Add value labels on bars
                for bar, value in zip(bars, topics_data.values):
                    if value > 0:
                        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01*max(topics_data.values),
                               f'{value:.1f}', ha='center', va='bottom', fontsize=8)
            
            # Hide unused subplots
            for idx in range(len(top_topics_dict), len(axes)):
                axes[idx].set_visible(False)
        
        plt.tight_layout()
        return fig
    
    def plot_frequency_analysis(self, figsize: Tuple[int, int] = (15, 8)):
        """Create frequency analysis using bar charts."""
        fig, ax = plt.subplots(1, 1, figsize=figsize)
        
        # Get topics of interest that exist in the data
        topics_in_data = []
        for topic in self.topics_of_interest:
            if topic in self.combined_data['Topic Name'].values:
                topics_in_data.append(topic)
        
        if not topics_in_data:
            ax.text(0.5, 0.5, 'None of the specified topics of interest found in data', 
                   ha='center', va='center', transform=ax.transAxes, fontsize=12)
            ax.set_title('Frequency Analysis: Topics of Interest')
            return fig
        
        # Create grouped bar chart
        datasets = sorted(self.data.keys())
        x = np.arange(len(datasets))
        width = 0.8 / len(topics_in_data)
        
        colors = plt.cm.tab10(np.linspace(0, 1, len(topics_in_data)))
        
        for i, topic in enumerate(topics_in_data):
            frequencies = []
            for dataset in datasets:
                topic_data = self.data[dataset][self.data[dataset]['Topic Name'] == topic]
                freq = topic_data['Frequency (Hz)'].iloc[0] if not topic_data.empty else 0
                frequencies.append(freq)
            
            bars = ax.bar(x + i * width, frequencies, width, 
                         label=topic.split('/')[-1], color=colors[i], alpha=0.8)
            
            # Add value labels on bars
            for bar, freq in zip(bars, frequencies):
                if freq > 0:
                    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01*max(frequencies),
                           f'{freq:.1f}', ha='center', va='bottom', fontsize=8)
        
        ax.set_xlabel('Dataset')
        ax.set_ylabel('Frequency (Hz)')
        ax.set_title('Frequency Analysis: Topics of Interest')
        ax.set_xticks(x + width * (len(topics_in_data) - 1) / 2)
        ax.set_xticklabels(datasets, rotation=45, ha='right')
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax.grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        return fig
    
    def generate_summary_report(self):
        """Generate a summary report of the analysis."""
        print("\n" + "="*60)
        print("ROS2 NETWORK ANALYSIS SUMMARY")
        print("="*60)
        print(f"Directory analyzed: {self.directory}")
        print(f"Topics of interest: {len(self.topics_of_interest)}")
        for topic in self.topics_of_interest:
            print(f"  - {topic}")
        
        for dataset_name, df in self.data.items():
            print(f"\nDataset: {dataset_name}")
            print(f"  Total topics (after filtering): {len(df)}")
            print(f"  Active topics (BW > 0): {len(df[df['Bandwidth (KiB/s)'] > 0])}")
            print(f"  Total bandwidth: {df['Bandwidth (KiB/s)'].sum():.2f} KiB/s")
            print(f"  Max frequency: {df['Frequency (Hz)'].max():.2f} Hz")
            
            # Check which topics of interest are present
            present_topics = df[df['Topic Name'].isin(self.topics_of_interest)]['Topic Name'].tolist()
            if present_topics:
                print(f"  Topics of interest present: {len(present_topics)}")
                for topic in present_topics:
                    topic_data = df[df['Topic Name'] == topic].iloc[0]
                    print(f"    {topic}: {topic_data['Bandwidth (KiB/s)']:.2f} KiB/s, {topic_data['Frequency (Hz)']:.2f} Hz")
            
            # Top 3 bandwidth topics for this dataset
            top_bw = df[df['Bandwidth (KiB/s)'] > 0].nlargest(3, 'Bandwidth (KiB/s)')
            if not top_bw.empty:
                print(f"  Top 3 bandwidth topics:")
                for _, row in top_bw.iterrows():
                    print(f"    {row['Topic Name']}: {row['Bandwidth (KiB/s)']:.2f} KiB/s")
        
        # Cross-dataset comparison
        if len(self.data) > 1:
            print(f"\nCROSS-DATASET COMPARISON:")
            dataset_totals = self.combined_data.groupby('Dataset')['Bandwidth (KiB/s)'].sum()
            print(f"  Highest total bandwidth: {dataset_totals.idxmax()} ({dataset_totals.max():.2f} KiB/s)")
            print(f"  Lowest total bandwidth: {dataset_totals.idxmin()} ({dataset_totals.min():.2f} KiB/s)")
    
    def analyze(self, save_plots: bool = False, plot_dir: str = "plots"):
        """Run complete analysis."""
        self.load_data()
        if not self.combined_data.empty:
            self.generate_summary_report()
            
            # Create plots
            print(f"\nGenerating plots...")
            bandwidth_fig = self.plot_bandwidth_analysis()
            frequency_fig = self.plot_frequency_analysis()
            
            if save_plots:
                Path(plot_dir).mkdir(exist_ok=True)
                if bandwidth_fig:
                    bandwidth_fig.savefig(f"{plot_dir}/bandwidth_analysis.png", dpi=300, bbox_inches='tight')
                if frequency_fig:
                    frequency_fig.savefig(f"{plot_dir}/frequency_analysis.png", dpi=300, bbox_inches='tight')
                print(f"Plots saved to {plot_dir}/")
            
            plt.show()
        else:
            print("No data loaded. Please check your CSV files.")

# Example usage and command-line interface
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze ROS2 network data from CSV files in a directory")
    parser.add_argument("directory", help="Directory containing CSV files to analyze")
    parser.add_argument("--save-plots", "-s", action="store_true", 
                       help="Save plots to files")
    parser.add_argument("--plot-dir", "-d", default="plots", 
                       help="Directory to save plots (default: plots)")
    
    args = parser.parse_args()
    
    # Default topics of interest (can be modified here if needed)
    topics_of_interest = [
        "/camera/color/image_raw",
        "/camera/aligned_depth_to_color/image_raw", 
        "/camera/infra1/image_rect_raw",
        "/camera/infra2/image_rect_raw",
        "/imu/data",
        "/pointcloud",
        "/joint_states"
    ]
    
    analyzer = ROS2NetworkAnalyzer(args.directory, topics_of_interest)
    analyzer.analyze(save_plots=args.save_plots, plot_dir=args.plot_dir)
