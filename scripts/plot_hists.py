import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import sys
import os

# Check for filename arguments
if len(sys.argv) < 2:
    print(f"Usage: python {sys.argv[0]} <csv_file1> [<csv_file2> ...]")
    sys.exit(1)

filenames = sys.argv[1:]
from datetime import datetime

# output_dir = "res/csv_outputs"
# os.makedirs(output_dir, exist_ok=True)

# Columns to plot
cols_to_plot = [
    "AvgOpenDuration",
    "OpenCount",
    "AvgReopenInterval",
    "TotalRefreshes",
    "AvgRefreshesBetweenReopens"
]

# Additional metric: per-bank access heatmap from row open counts
bank_access_counts = {}

# Dictionary to hold dataframes keyed by filename
dataframes = {}

# Read and store all dataframes
from collections import defaultdict

for filename in filenames:
    if not os.path.isfile(filename):
        print(f"Error: File '{filename}' not found.")
        continue

    prefix = os.path.splitext(os.path.basename(filename))[0]

    try:
        df = pd.read_csv(filename)
        df[cols_to_plot] = df[cols_to_plot].apply(pd.to_numeric, errors='coerce')
        dataframes[prefix] = df
        # Aggregate bank access counts
        grouped = df.groupby(["Channel", "Rank", "Bank"])["OpenCount"].sum()
        for index, count in grouped.items():
            bank_access_counts[index] = bank_access_counts.get(index, 0) + count
    except Exception as e:
        print(f"Error reading '{filename}': {e}")
        continue

    # Create individual PDF
    csv_dir = os.path.dirname(os.path.abspath(filename))
    pdf_path = os.path.join(csv_dir, f"{prefix}_plots.pdf")
    with PdfPages(pdf_path) as pdf:
        # Add per-bank access heatmap for this file
        bank_grouped = df.groupby(["Channel", "Rank", "Bank"])["OpenCount"].sum()
        fig, ax = plt.subplots(figsize=(10, 6))
        bank_labels = [f"C{ch}R{rk}B{bk}" for (ch, rk, bk) in bank_grouped.index]
        counts = bank_grouped.values
        ax.bar(bank_labels, counts, color='slateblue')
        ax.set_title(f"Bank Access Heatmap ({prefix})")
        ax.set_xlabel("Bank")
        ax.set_ylabel("Total OpenCount")
        ax.tick_params(axis='x', rotation=90)
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

        # Add row access distribution per bank
        bank_groups = list(df.groupby(["Channel", "Rank", "Bank"]))
        n_banks = len(bank_groups)
        n_cols = 2
        n_rows = (n_banks + n_cols - 1) // n_cols
        fig, axes = plt.subplots(n_rows, n_cols, figsize=(14, 5 * n_rows), squeeze=False)
        for ax, ((ch, rk, bk), group) in zip(axes.flat, bank_groups):
            row_min = group["Row"].min()
            row_max = group["Row"].max()
            row_range = row_max - row_min + 1
            bin_size = max(1, row_range // 20)
            bins = np.arange(row_min, row_max + bin_size, bin_size)
            hist_vals, _, _ = ax.hist(group["Row"], bins=bins, color='orange', edgecolor='black')
            total = hist_vals.sum()
            percentages = (hist_vals / total * 100) if total > 0 else hist_vals
            ax.clear()
            ax.bar(bins[:-1], percentages, width=np.diff(bins), align='edge', color='orange', edgecolor='black')
            ax.set_xticks(bins[::max(1, len(bins)//10)])
            ax.tick_params(axis='x', rotation=45)
            ax.text(0.98, 0.95, f"Hot Row Range: {row_min}–{row_max}", transform=ax.transAxes,
                    ha='right', va='top', fontsize=8, color='gray')
            ax.set_ylabel("% of Accesses")
            ax.set_title(f"Row Access Distribution - C{ch}R{rk}B{bk}")
            ax.set_xlabel("Row")
            ax.set_ylabel("% of Accesses")
        for ax in axes.flat[n_banks:]:
            fig.delaxes(ax)
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)
        for col in cols_to_plot:
            values = df[col].dropna()
            if values.empty:
                continue

            fig, axes = plt.subplots(1, 2, figsize=(14, 5))
            fig.suptitle(f"{col} Analysis ({prefix})", fontsize=14)

            # Histogram
            axes[0].hist(values, bins=100, color='skyblue', edgecolor='black')
            mean_val = values.mean()
            median_val = values.median()
            axes[0].axvline(mean_val, color='red', linestyle='--', label=f"Mean: {mean_val:.2e}")
            axes[0].axvline(median_val, color='green', linestyle='--', label=f"Median: {median_val:.2e}")
            axes[0].legend()
            axes[0].set_title("Histogram")
            axes[0].set_xlabel(col)
            axes[0].set_ylabel("Row Count")

            # CDF
            sorted_vals = values.sort_values()
            cdf = np.arange(1, len(sorted_vals) + 1) / len(sorted_vals)
            axes[1].plot(sorted_vals, cdf, color='blue')
            
            
            
            axes[1].set_title("CDF")
            axes[1].set_xlabel(col)
            axes[1].set_ylabel("Cumulative %")

            fig.tight_layout(rect=[0, 0.03, 1, 0.95])
            pdf.savefig(fig)
            plt.close(fig)

    print(f"Saved: {pdf_path}")

# Merge dataframes on shared keys: Channel, Rank, Bank, Row
if dataframes:
    keys = ["Channel", "Rank", "Bank", "Row"]
    merged_df = None
    for prefix, df in dataframes.items():
        df_renamed = df[keys + cols_to_plot].copy()
        df_renamed = df_renamed.rename(columns={col: f"{prefix}_{col}" for col in cols_to_plot})
        if merged_df is None:
            merged_df = df_renamed
        else:
            merged_df = pd.merge(merged_df, df_renamed, on=keys, how='outer')

    # Create combined PDF for all columns
    merged_output_dir = "res/merged"
    os.makedirs(merged_output_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    merged_pdf_path = os.path.join(merged_output_dir, f"merged_plots_{timestamp}.pdf")
    
    merged_output_dir = "res/merged"
    os.makedirs(merged_output_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    merged_pdf_path = os.path.join(merged_output_dir, f"merged_plots_{timestamp}.pdf")
    with PdfPages(merged_pdf_path) as pdf:
        # Add bank access heatmap
        fig, ax = plt.subplots(figsize=(10, 6))
        bank_labels = [f"C{ch}R{rk}B{bk}" for (ch, rk, bk) in bank_access_counts]
        counts = [bank_access_counts[key] for key in bank_access_counts]
        ax.bar(bank_labels, counts, color='slateblue')
        ax.set_title("Bank Access Heatmap (Merged)")
        ax.set_xlabel("Bank")
        ax.set_ylabel("Total OpenCount")
        ax.tick_params(axis='x', rotation=90)
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

        # Add row access distributions for each bank in merged
        bank_groups = list(merged_df.groupby(["Channel", "Rank", "Bank"]))
        n_banks = len(bank_groups)
        n_cols = 2
        n_rows = (n_banks + n_cols - 1) // n_cols
        fig, axes = plt.subplots(n_rows, n_cols, figsize=(14, 5 * n_rows), squeeze=False)
        for ax, ((ch, rk, bk), group) in zip(axes.flat, bank_groups):
            row_min = group["Row"].min()
            row_max = group["Row"].max()
            row_range = row_max - row_min + 1
            bin_size = max(1, row_range // 20)
            bins = np.arange(row_min, row_max + bin_size, bin_size)
            hist_vals, _, _ = ax.hist(group["Row"], bins=bins, color='orange', edgecolor='black')
            total = hist_vals.sum()
            percentages = (hist_vals / total * 100) if total > 0 else hist_vals
            ax.clear()
            ax.bar(bins[:-1], percentages, width=np.diff(bins), align='edge', color='orange', edgecolor='black')
            ax.set_xticks(bins[::max(1, len(bins)//10)])
            ax.tick_params(axis='x', rotation=45)
            ax.text(0.98, 0.95, f"Hot Row Range: {row_min}–{row_max}", transform=ax.transAxes,
                    ha='right', va='top', fontsize=8, color='gray')
            ax.set_title(f"Row Access Distribution - C{ch}R{rk}B{bk}")
            ax.set_xlabel("Row")
            ax.set_ylabel("% of Accesses")
        for ax in axes.flat[n_banks:]:
            fig.delaxes(ax)
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close(fig)

        for base_col in cols_to_plot:
            values = []
            for prefix in dataframes:
                colname = f"{prefix}_{base_col}"
                if colname in merged_df:
                    values.append(merged_df[colname])

            if not values:
                continue

            all_vals = pd.concat(values).dropna()
            if all_vals.empty:
                continue

            fig, axes = plt.subplots(1, 2, figsize=(14, 5))
            fig.suptitle(f"{base_col} Analysis (Merged)", fontsize=14)

            # Histogram
            axes[0].hist(all_vals, bins=100, color='skyblue', edgecolor='black')
            mean_val = all_vals.mean()
            median_val = all_vals.median()
            axes[0].axvline(mean_val, color='red', linestyle='--', label=f"Mean: {mean_val:.2e}")
            axes[0].axvline(median_val, color='green', linestyle='--', label=f"Median: {median_val:.2e}")
            axes[0].legend()
            axes[0].set_title("Histogram")
            axes[0].set_xlabel(base_col)
            axes[0].set_ylabel("Row Count")

            # CDF
            sorted_vals = all_vals.sort_values()
            cdf = np.arange(1, len(sorted_vals) + 1) / len(sorted_vals)
            axes[1].plot(sorted_vals, cdf, color='blue')
            axes[1].set_title("CDF")
            axes[1].set_xlabel(base_col)
            axes[1].set_ylabel("Cumulative %")

            fig.tight_layout(rect=[0, 0.03, 1, 0.95])
            pdf.savefig(fig)
            plt.close(fig)

        
    
    print(f"Merged plots saved to: {merged_pdf_path}")
