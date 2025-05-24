import re
import sys
from collections import defaultdict
import pandas as pd

def parse_dram_log(log_content, target_bank=None, target_bg=None, target_rank=None, target_channel=None):
    """
    Parses the DRAM log content to extract row open/close events and active buffer states,
    optionally filtering by specific bank, bankgroup, rank, and channel IDs for row events.

    Args:
        log_content (str): The full text content of the DRAM log file.
        target_bank (int, optional): The bank ID to filter row events by. Defaults to None (no filter).
        target_bg (int, optional): The bankgroup ID to filter row events by. Defaults to None (no filter).
        target_rank (int, optional): The rank ID to filter row events by. Defaults to None (no filter).
        target_channel (int, optional): The channel ID to filter row events by. Defaults to None (no filter).

    Returns:
        tuple: A tuple containing:
            - dict: A dictionary where keys are unique row identifiers
                    (e.g., "Row 13412, bank 1, bg 6, rank 1, channel 0")
                    and values are lists of (event_type, cycle) tuples.
            - dict: A dictionary of rows that are currently open at the
                    end of the log, with their open cycle.
            - list: A list of dictionaries, each representing an active buffer state
                    at a specific cycle, with details of requests in it.
    """
    row_history = defaultdict(list)
    open_rows = {}
    active_buffer_snapshots = []

    # Regex for row open/close events
    row_pattern = re.compile(
        r"Row (\d+) in bank (\d+), bg (\d+), rank (\d+), channel (\d+) "
        r"(opened|closed) at cycle (\d+)(?: \(open for (\d+) cycles\))?"
    )

    # Regex for Active Buffer header
    active_buffer_header_pattern = re.compile(r"\[Active Buffer\] Size: (\d+)")
    # Regex for individual request lines within Active Buffer
    request_pattern = re.compile(
        r"Req: cmd=(\d+) addr=\[channel:(\d+) rank:(\d+) bankgroup:(\d+) bank:(\d+) row:(\d+) column:(\d+) \] final_cmd=(\d+) type=(\d+)"
    )

    current_cycle = -1
    expecting_requests = False
    current_active_buffer_requests = []
    
    for line in log_content.splitlines():
        # Try to parse row events first
        row_match = row_pattern.match(line)
        if row_match:
            # If we were expecting requests, and a new row event appears,
            # it means the previous active buffer snapshot has ended.
            if expecting_requests and current_active_buffer_requests is not None:
                if current_cycle != -1: # Ensure we have a valid cycle for the snapshot
                    active_buffer_snapshots.append({
                        "cycle": current_cycle,
                        "requests": list(current_active_buffer_requests) # Store a copy
                    })
                current_active_buffer_requests = []
                expecting_requests = False

            # Process row event
            row_id_str, bank_str, bg_str, rank_str, channel_str, event_type, cycle_str, _ = row_match.groups()
            
            bank = int(bank_str)
            bg = int(bg_str)
            rank = int(rank_str)
            channel = int(channel_str)
            cycle = int(cycle_str)

            # Update current_cycle based on the most recent event's cycle
            current_cycle = cycle

            # Apply filters for row events
            if target_bank is not None and bank != target_bank:
                continue
            if target_bg is not None and bg != target_bg:
                continue
            if target_rank is not None and rank != target_rank:
                continue
            if target_channel is not None and channel != target_channel:
                continue

            unique_row_identifier = (
                f"Row {row_id_str}, bank {bank_str}, bg {bg_str}, rank {rank_str}, channel {channel_str}"
            )

            if event_type == "opened":
                row_history[unique_row_identifier].append(("opened", cycle))
                open_rows[unique_row_identifier] = cycle
            elif event_type == "closed":
                row_history[unique_row_identifier].append(("closed", cycle))
                if unique_row_identifier in open_rows:
                    del open_rows[unique_row_identifier]
            continue # Move to next line after processing row event

        # Try to parse active buffer header
        active_buffer_header_match = active_buffer_header_pattern.match(line)
        if active_buffer_header_match:
            # If we find a new header, it means the previous snapshot is complete
            if expecting_requests and current_active_buffer_requests is not None:
                if current_cycle != -1:
                    active_buffer_snapshots.append({
                        "cycle": current_cycle,
                        "requests": list(current_active_buffer_requests)
                    })
            current_active_buffer_requests = [] # Reset for new snapshot
            expecting_requests = True
            # The cycle for this snapshot is usually the cycle of the *preceding* Row event.
            # We assume current_cycle is updated by the last row event.
            continue

        # If we are expecting requests, try to parse request lines
        if expecting_requests:
            request_match = request_pattern.match(line.strip())
            if request_match:
                cmd, channel, rank, bg, bank, row, column, final_cmd, req_type = request_match.groups()
                current_active_buffer_requests.append({
                    "cmd": int(cmd),
                    "channel": int(channel),
                    "rank": int(rank),
                    "bankgroup": int(bg),
                    "bank": int(bank),
                    "row": int(row),
                    "column": int(column),
                    "final_cmd": int(final_cmd),
                    "type": int(req_type)
                })
            else:
                # If we encounter a line that's not a request and not a new header/row event,
                # it means the active buffer snapshot has ended.
                if current_active_buffer_requests: # Only add if there were requests
                    if current_cycle != -1:
                        active_buffer_snapshots.append({
                            "cycle": current_cycle,
                            "requests": list(current_active_buffer_requests)
                        })
                current_active_buffer_requests = []
                expecting_requests = False

    # After the loop, add any remaining active buffer requests
    if expecting_requests and current_active_buffer_requests:
        if current_cycle != -1:
            active_buffer_snapshots.append({
                "cycle": current_cycle,
                "requests": list(current_active_buffer_requests)
            })

    return row_history, open_rows, active_buffer_snapshots

def display_results(row_history, open_rows, active_buffer_snapshots, output_file=None):
    """
    Displays the parsing results in a user-friendly format.

    Args:
        row_history (dict): A dictionary of row histories.
        open_rows (dict): A dictionary of rows currently open.
        active_buffer_snapshots (list): A list of active buffer snapshots.
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

    # --- Part 2: Detailed Open/Close Sequence for Each Row ---
    write_func("## Detailed Open/Close Sequence for Each Row")
    if row_history:
        detailed_data = []
        for row_identifier, events in row_history.items():
            events.sort(key=lambda x: x[1]) # Sort events by cycle
            
            sequence_str = []
            for event_type, cycle in events:
                sequence_str.append(f"{event_type.capitalize()} @ {cycle}")
            
            detailed_data.append([row_identifier, " -> ".join(sequence_str)])
        
        detailed_df = pd.DataFrame(detailed_data, columns=["Row Identifier", "Event Sequence"])
        write_func(detailed_df.to_markdown(index=False))
    else:
        write_func("No row open/close history found in the log.")
    write_func("\n")

    # --- Part 3: Active Buffer Snapshots ---
    write_func("## Active Buffer Snapshots Throughout the Run")
    if active_buffer_snapshots:
        # Sort snapshots by cycle to ensure chronological order
        active_buffer_snapshots.sort(key=lambda x: x['cycle'])

        for snapshot in active_buffer_snapshots:
            write_func(f"--- Active Buffer at Cycle {snapshot['cycle']} ---")
            if snapshot['requests']:
                req_data = []
                for req in snapshot['requests']:
                    req_data.append([
                        req['cmd'], req['channel'], req['rank'],
                        req['bankgroup'], req['bank'], req['row'],
                        req['column'], req['final_cmd'], req['type']
                    ])
                req_df = pd.DataFrame(req_data, columns=[
                    "Cmd", "Chan", "Rank", "BG", "Bank", "Row", "Col", "Final Cmd", "Type"
                ])
                write_func(req_df.to_markdown(index=False))
            else:
                write_func("  (No requests in buffer)")
            write_func("\n")
    else:
        write_func("No active buffer snapshots found in the log.")


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
                try:
                    target_bank = int(sys.argv[i+1])
                except ValueError:
                    print(f"Error: Invalid ID for --bank: {sys.argv[i+1]}. Must be an integer.", file=sys.stderr)
                    sys.exit(1)
                i += 2
            else:
                print("Error: --bank requires an argument.", file=sys.stderr)
                sys.exit(1)
        elif arg == "--bg":
            if i + 1 < len(sys.argv):
                try:
                    target_bg = int(sys.argv[i+1])
                except ValueError:
                    print(f"Error: Invalid ID for --bg: {sys.argv[i+1]}. Must be an integer.", file=sys.stderr)
                    sys.exit(1)
                i += 2
            else:
                print("Error: --bg requires an argument.", file=sys.stderr)
                sys.exit(1)
        elif arg == "--rank":
            if i + 1 < len(sys.argv):
                try:
                    target_rank = int(sys.argv[i+1])
                except ValueError:
                    print(f"Error: Invalid ID for --rank: {sys.argv[i+1]}. Must be an integer.", file=sys.stderr)
                    sys.exit(1)
                i += 2
            else:
                print("Error: --rank requires an argument.", file=sys.stderr)
                sys.exit(1)
        elif arg == "--channel":
            if i + 1 < len(sys.argv):
                try:
                    target_channel = int(sys.argv[i+1])
                except ValueError:
                    print(f"Error: Invalid ID for --channel: {sys.argv[i+1]}. Must be an integer.", file=sys.stderr)
                    sys.exit(1)
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
    row_history, open_rows, active_buffer_snapshots = parse_dram_log(
        log_content,
        target_bank=target_bank,
        target_bg=target_bg,
        target_rank=target_rank,
        target_channel=target_channel
    )
    display_results(row_history, open_rows, active_buffer_snapshots, output_file_path)
