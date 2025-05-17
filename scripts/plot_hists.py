import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import sys
import os
from datetime import datetime

# --- Generic Plotting Functions ---

def plot_bank_access_heatmap(bank_access_counts, pdf, title="Bank Access Heatmap"):
    fig, ax = plt.subplots(figsize=(10, 6))
    bank_labels = [f"C{ch}R{rk}BG{bg}B{bk}" for (ch, rk, bg, bk) in bank_access_counts]
    counts = [bank_access_counts[key] for key in bank_access_counts]
    ax.bar(bank_labels, counts, color='slateblue')
    ax.set_title(title)
    ax.set_xlabel("Bank [Channel Rank BankGroup Bank]")
    ax.set_ylabel("Total OpenCount [Bank Opens]")
    ax.tick_params(axis='x', rotation=0)
    fig.tight_layout()
    pdf.savefig(fig)
    plt.close(fig)

def plot_row_access_distributions(bank_groups, pdf, title_prefix=""):
    n_banks = len(bank_groups)
    n_cols = 2
    n_rows = (n_banks + n_cols - 1) // n_cols
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(14, 5 * n_rows), squeeze=False)
    for ax, ((ch, rk, bg, bk), group) in zip(axes.flat, bank_groups):
        row_min = group["Row"].min()
        row_max = group["Row"].max()
        row_range = row_max - row_min + 1
        bin_size = max(1, row_range // 50)
        bins = np.arange(row_min, row_max + bin_size, bin_size)
        hist_vals, _, _ = ax.hist(group["Row"], bins=bins, color='orange', edgecolor='black')
        total = hist_vals.sum()
        percentages = (hist_vals / total * 100) if total > 0 else hist_vals
        ax.clear()
        ax.bar(bins[:-1], percentages, width=np.diff(bins), align='edge', color='orange', edgecolor='black')
        ax.set_xticks(bins[::max(1, len(bins)//10)])
        ax.tick_params(axis='x', rotation=45)
        sorted_rows = np.sort(group["Row"])
        if len(sorted_rows) > 0:
            lower_idx = int(0.25 * len(sorted_rows))
            upper_idx = int(0.75 * len(sorted_rows)) - 1
            hot_row_min = sorted_rows[lower_idx]
            hot_row_max = sorted_rows[upper_idx]
            ax.text(0.98, 0.95, f"Hot Row Range (IQR): {hot_row_min}–{hot_row_max}", transform=ax.transAxes,
                    ha='right', va='top', fontsize=8, color='gray')
        else:
            ax.text(0.98, 0.95, "No data", transform=ax.transAxes,
                    ha='right', va='top', fontsize=8, color='gray')
        ax2 = ax.twinx()
        cdf = np.arange(1, len(sorted_rows)+1) / len(sorted_rows) if len(sorted_rows) > 0 else []
        if len(sorted_rows) > 0:
            ax2.plot(sorted_rows, cdf*100, color='blue', linestyle='--')
        ax2.set_ylabel("Cumulative %", color='blue')
        ax2.tick_params(axis='y', labelcolor='blue')
        ax.set_title(f"{title_prefix}Row Access Distribution - C{ch}R{rk}BG{bg}B{bk}")
        ax.set_xlabel("Row [Row Number]")
        ax.set_ylabel("% of Accesses")
    for ax in axes.flat[n_banks:]:
        fig.delaxes(ax)
    fig.tight_layout()
    pdf.savefig(fig)
    plt.close(fig)

def plot_metric_histograms(values, col, pdf, title_prefix=""):
    if len(values) == 0:
        return

    # Define units for each metric
    units = {
        "AvgOpenDuration": "CPU cycles",
        "AvgReopenInterval": "CPU cycles",
        "OpenCount": "Number of row opens",
        "FullRefreshCycles": "Refresh Cycles",
        "AvgRefreshesBetweenReopens": "Refresh Cycles"
    }
    unit = units.get(col, "")

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle(f"{col} Analysis ({title_prefix})", fontsize=14)
    # Histogram
    if col in ["AvgOpenDuration", "OpenCount", "AvgReopenInterval"]:
        min_val = values[values > 0].min() if (values > 0).any() else 1e-10
        max_val = values.max()
        bins = np.logspace(np.log10(min_val), np.log10(max_val), 50)
        axes[0].hist(values, bins=bins, color='skyblue', edgecolor='black')
        axes[0].set_xscale('log')
        axes[1].set_xscale('log')
    else:
        axes[0].hist(values, bins=100, color='skyblue', edgecolor='black')
    mean_val = values.mean()
    median_val = values.median()
    axes[0].axvline(mean_val, color='red', linestyle='--', label=f"Mean: {mean_val:.2e} {unit}")
    axes[0].axvline(median_val, color='green', linestyle='--', label=f"Median: {median_val:.2e} {unit}")
    axes[0].legend()
    axes[0].set_title("Histogram")
    axes[0].set_xlabel(f"{col} [{unit}]" if unit else col)
    axes[0].set_ylabel("Row Count")
    # CDF
    sorted_vals = np.sort(values)
    cdf = np.arange(1, len(sorted_vals) + 1) / len(sorted_vals)
    axes[1].plot(sorted_vals, cdf, color='blue')
    axes[1].set_title("CDF")
    axes[1].set_xlabel(f"{col} [{unit}]" if unit else col)
    axes[1].set_ylabel("Cumulative %")
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    pdf.savefig(fig)
    plt.close(fig)

# --- Main Script Logic ---

if len(sys.argv) < 2:
    print(f"Usage: python {sys.argv[0]} <csv_file1> [<csv_file2> ...]")
    sys.exit(1)

filenames = sys.argv[1:]
cols_to_plot = [
    "AvgOpenDuration",
    "OpenCount",
    "AvgReopenInterval",
    "FullRefreshCycles",
    "AvgRefreshesBetweenReopens"
]
bank_access_counts = {}
dataframes = {}

for filename in filenames:
    if not os.path.isfile(filename):
        print(f"Error: File '{filename}' not found.")
        continue
    prefix = os.path.splitext(os.path.basename(filename))[0]
    try:
        df = pd.read_csv(filename)
        df[cols_to_plot] = df[cols_to_plot].apply(pd.to_numeric, errors='coerce')
        dataframes[prefix] = df
        grouped = df.groupby(["Channel", "Rank", "BankGroup", "Bank"])["OpenCount"].sum()
        for index, count in grouped.items():
            bank_access_counts[index] = bank_access_counts.get(index, 0) + count
    except Exception as e:
        print(f"Error reading '{filename}': {e}")
        continue

    # Create individual PDF
    csv_dir = os.path.dirname(os.path.abspath(filename))
    pdf_path = os.path.join(csv_dir, f"{prefix}_plots.pdf")
    with PdfPages(pdf_path) as pdf:
        plot_bank_access_heatmap(grouped.to_dict(), pdf, title=f"Bank Access Heatmap ({prefix})")
        bank_groups = list(df.groupby(["Channel", "Rank", "BankGroup", "Bank"]))
        plot_row_access_distributions(bank_groups, pdf)
        for col in cols_to_plot:
            values = df[col].dropna()
            plot_metric_histograms(values, col, pdf, title_prefix=prefix)
    print(f"Saved: {pdf_path}")

# Merge dataframes on shared keys: Channel, Rank, BankGroup, Bank, Row
if dataframes:
    keys = ["Channel", "Rank", "BankGroup", "Bank", "Row"]
    merged_df = None
    for prefix, df in dataframes.items():
        df_renamed = df[keys + cols_to_plot].copy()
        df_renamed = df_renamed.rename(columns={col: f"{prefix}_{col}" for col in cols_to_plot})
        if merged_df is None:
            merged_df = df_renamed
        else:
            merged_df = pd.merge(merged_df, df_renamed, on=keys, how='outer')

    merged_output_dir = "res/merged"
    os.makedirs(merged_output_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    merged_pdf_path = os.path.join(merged_output_dir, f"merged_plots_{timestamp}.pdf")
    with PdfPages(merged_pdf_path) as pdf:
        plot_bank_access_heatmap(bank_access_counts, pdf, title="Bank Access Heatmap (Merged)")
        bank_groups = list(merged_df.groupby(["Channel", "Rank", "BankGroup", "Bank"]))
        plot_row_access_distributions(bank_groups, pdf, title_prefix="Merged ")
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
            plot_metric_histograms(all_vals, base_col, pdf, title_prefix="Merged")
    print(f"Merged plots saved to: {merged_pdf_path}")