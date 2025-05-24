import re
import sys
from collections import defaultdict
import pandas as pd

def parse_dram_log(log_content, target_bank=None, target_bg=None, target_rank=None, target_channel=None):
    """
    Parses the DRAM log content to extract row open/close events,
    optionally filtering by specific bank, bankgroup, rank, and channel IDs.

    Args:
        log_content (str): The full text content of the DRAM log file.
        target_bank (int, optional): The bank ID to filter by. Defaults to None (no filter).
        target_bg (int, optional): The bankgroup ID to filter by. Defaults to None (no filter).
        target_rank (int, optional): The rank ID to filter by. Defaults to None (no filter).
        target_channel (int, optional): The channel ID to filter by. Defaults to None (no filter).

    Returns:
        tuple: A tuple containing:
            - dict: A dictionary where keys are unique row identifiers
                    (e.g., "Row 13412, bank 1, bg 6, rank 1, channel 0")
                    and values are lists of (event_type, cycle) tuples.
            - dict: A dictionary of rows that are currently open at the
                    end of the log, with their open cycle.
    """
    # Dictionary to store the history of each row
    row_history = defaultdict(list)
    # Dictionary to keep track of currently open rows
    open_rows = {}

    # Regex to capture relevant information from each log line
    # It extracts row, bank, bg, rank, channel, event type (opened/closed), and cycle
    # It also captures the 'open for X cycles' part for closed events
    pattern = re.compile(
        r"Row (\d+) in bank (\d+), bg (\d+), rank (\d+), channel (\d+) "
        r"(opened|closed) at cycle (\d+)(?: \(open for (\d+) cycles\))?"
    )

    for line in log_content.splitlines():
        match = pattern.match(line)
        if match:
            # Extract matched groups
            row_id_str, bank_str, bg_str, rank_str, channel_str, event_type, cycle_str, open_cycles_str = match.groups()
            
            # Convert extracted string values to integers for comparison
            bank = int(bank_str)
            bg = int(bg_str)
            rank = int(rank_str)
            channel = int(channel_str)
            cycle = int(cycle_str)

            # Apply filters
            # If a target filter is specified and the current log line's value
            # does not match, skip this line and move to the next.
            if target_bank is not None and bank != target_bank:
                continue
            if target_bg is not None and bg != target_bg:
                continue
            if target_rank is not None and rank != target_rank:
                continue
            if target_channel is not None and channel != target_channel:
                continue

            # If all filters pass (or no filters are set), process the event
            unique_row_identifier = (
                f"Row {row_id_str}, bank {bank_str}, bg {bg_str}, rank {rank_str}, channel {channel_str}"
            )

            if event_type == "opened":
                # Add the 'opened' event to the row's history
                row_history[unique_row_identifier].append(("opened", cycle))
                # Mark the row as open
                open_rows[unique_row_identifier] = cycle
            elif event_type == "closed":
                # Add the 'closed' event to the row's history
                row_history[unique_row_identifier].append(("closed", cycle))
                # If the row was marked as open, remove it from the open_rows dictionary
                if unique_row_identifier in open_rows:
                    del open_rows[unique_row_identifier]

    return row_history, open_rows

def display_results(row_history, open_rows, output_file=None):
    """
    Displays the parsing results in a user-friendly format.

    Args:
        row_history (dict): A dictionary of row histories.
        open_rows (dict): A dictionary of rows currently open.
        output_file (str, optional): Path to the output file. If None, prints to console.
    """
    # Determine the output destination
    if output_file:
        try:
            f = open(output_file, "w")
            write_func = lambda s: f.write(s + "\n")
        except IOError as e:
            print(f"Error: Could not open output file '{output_file}': {e}", file=sys.stderr)
            write_func = print # Fallback to printing if file cannot be opened
    else:
        write_func = print

    # --- Part 1: Rows that remained open at the end of the run ---
    write_func("## Rows Remaining Open at the End of the Run")
    if open_rows:
        open_data = []
        for row_identifier, open_cycle in open_rows.items():
            open_data.append([row_identifier, open_cycle])
        
        # Create a DataFrame for better table representation
        open_df = pd.DataFrame(open_data, columns=["Row Identifier", "Opened at Cycle"])
        write_func(open_df.to_markdown(index=False))
    else:
        write_func("No rows remained open at the end of the simulation run.")
    write_func("\n")

    # --- Part 2: Sequence of Open/Close Events for Each Row ---
    write_func("## Detailed Open/Close Sequence for Each Row")
    if row_history:
        # Prepare data for a more detailed table or graphical representation concept
        detailed_data = []
        for row_identifier, events in row_history.items():
            # Sort events by cycle for correct sequence
            events.sort(key=lambda x: x[1])
            
            sequence_str = []
            for event_type, cycle in events:
                sequence_str.append(f"{event_type.capitalize()} @ {cycle}")
            
            detailed_data.append([row_identifier, " -> ".join(sequence_str)])
        
        # Create a DataFrame for detailed history
        detailed_df = pd.DataFrame(detailed_data, columns=["Row Identifier", "Event Sequence"])
        write_func(detailed_df.to_markdown(index=False))
    else:
        write_func("No row open/close history found in the log.")

    if output_file:
        f.close()

# Main execution block
if __name__ == "__main__":
    # Define default values for filter parameters
    target_bank = None
    target_bg = None
    target_rank = None
    target_channel = None
    output_file_path = None

    # Check for minimum arguments (script name + log file path)
    if len(sys.argv) < 2:
        print("Usage: python dram_parser.py <log_file_path> [output_file_path] [--bank <id>] [--bg <id>] [--rank <id>] [--channel <id>]", file=sys.stderr)
        sys.exit(1)

    log_file_path = sys.argv[1]
    
    # Parse command-line arguments for output file and filters
    i = 2
    while i < len(sys.argv):
        arg = sys.argv[i]
        if arg == "--bank":
            if i + 1 < len(sys.argv):
                target_bank = int(sys.argv[i+1])
                i += 2
            else:
                print("Error: --bank requires an argument.", file=sys.stderr)
                sys.exit(1)
        elif arg == "--bg":
            if i + 1 < len(sys.argv):
                target_bg = int(sys.argv[i+1])
                i += 2
            else:
                print("Error: --bg requires an argument.", file=sys.stderr)
                sys.exit(1)
        elif arg == "--rank":
            if i + 1 < len(sys.argv):
                target_rank = int(sys.argv[i+1])
                i += 2
            else:
                print("Error: --rank requires an argument.", file=sys.stderr)
                sys.exit(1)
        elif arg == "--channel":
            if i + 1 < len(sys.argv):
                target_channel = int(sys.argv[i+1])
                i += 2
            else:
                print("Error: --channel requires an argument.", file=sys.stderr)
                sys.exit(1)
        # Assuming the first argument after log_file_path that is not a flag is the output_file_path
        elif output_file_path is None and not arg.startswith("--"): 
            output_file_path = arg
            i += 1
        else:
            print(f"Error: Unknown argument or incorrect placement: {arg}", file=sys.stderr)
            sys.exit(1)

    try:
        with open(log_file_path, "r") as f:
            log_content = f.read()
    except FileNotFoundError:
        print(f"Error: The file '{log_file_path}' was not found.", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"An error occurred while reading the file: {e}", file=sys.stderr)
        sys.exit(1)

    # Pass filter parameters to parse_dram_log
    row_history, open_rows = parse_dram_log(
        log_content,
        target_bank=target_bank,
        target_bg=target_bg,
        target_rank=target_rank,
        target_channel=target_channel
    )
    display_results(row_history, open_rows, output_file_path)
