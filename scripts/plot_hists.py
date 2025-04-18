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
output_dir = "res/csv_outputs"
os.makedirs(output_dir, exist_ok=True)

# Columns to plot
cols_to_plot = [
    "AvgOpenDuration",
    "OpenCount",
    "AvgReopenInterval",
    "TotalRefreshes",
    "AvgRefreshesBetweenReopens"
]

# Dictionary to hold dataframes keyed by filename
dataframes = {}

# Read and store all dataframes
for filename in filenames:
    if not os.path.isfile(filename):
        print(f"Error: File '{filename}' not found.")
        continue

    prefix = os.path.splitext(os.path.basename(filename))[0]

    try:
        df = pd.read_csv(filename)
        df[cols_to_plot] = df[cols_to_plot].apply(pd.to_numeric, errors='coerce')
        dataframes[prefix] = df
    except Exception as e:
        print(f"Error reading '{filename}': {e}")
        continue

    # Create individual PDF
    pdf_path = os.path.join(output_dir, f"{prefix}_plots.pdf")
    with PdfPages(pdf_path) as pdf:
        for col in cols_to_plot:
            bank_groups = list(df.groupby(["Channel", "Rank", "Bank"]))
            n_banks = len(bank_groups)
            n_cols = 2
            n_rows = (n_banks + n_cols - 1) // n_cols
            fig, axes = plt.subplots(n_rows, n_cols, figsize=(14, 5 * n_rows), squeeze=False)
            for ax, ((ch, rk, bk), group) in zip(axes.flat, bank_groups):
                ax.plot(group["Row"], group[col], linestyle='None', marker='o', markersize=2)
                ax.set_title(f"{col} - C{ch}R{rk}B{bk}")
                ax.set_xlabel("Row")
                ax.set_ylabel(col)
                ax.grid(True)
            for ax in axes.flat[n_banks:]:
                fig.delaxes(ax)
            fig.suptitle(f"{col} per Bank ({prefix})", fontsize=16)
            fig.tight_layout(rect=[0, 0, 1, 0.97])
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

    # Create combined PDF
    merged_pdf_path = os.path.join(output_dir, "merged_plots.pdf")
    with PdfPages(merged_pdf_path) as pdf:
        for base_col in cols_to_plot:
            bank_groups = list(merged_df.groupby(["Channel", "Rank", "Bank"]))
            n_banks = len(bank_groups)
            n_cols = 2
            n_rows = (n_banks + n_cols - 1) // n_cols
            fig, axes = plt.subplots(n_rows, n_cols, figsize=(14, 5 * n_rows), squeeze=False)
            for ax, ((ch, rk, bk), group) in zip(axes.flat, bank_groups):
                for prefix in dataframes.keys():
                    colname = f"{prefix}_{base_col}"
                    if colname in group:
                        label = f"{prefix}"
                        ax.plot(group["Row"], group[colname], linestyle='None', marker='o', markersize=2, label=label)
                ax.set_title(f"{base_col} - C{ch}R{rk}B{bk}")
                ax.set_xlabel("Row")
                ax.set_ylabel(base_col)
                ax.grid(True)
                ax.legend()
            for ax in axes.flat[n_banks:]:
                fig.delaxes(ax)
            fig.suptitle(f"{base_col} per Bank (Merged)", fontsize=16)
            fig.tight_layout(rect=[0, 0, 1, 0.97])
            pdf.savefig(fig)
            plt.close(fig)

    print(f"Merged plots saved to: {merged_pdf_path}")
