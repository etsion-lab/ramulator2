import re
import sys
from collections import defaultdict
import pandas as pd

def parse_dram_log(log_content, target_channel=None, target_rank=None, target_bankgroup=None, target_bank=None):
    """
    Parses the DRAM log content to extract row open/close events, active buffer states,
    and debug events (HIT, MISS, CONFLICT), filtering all by specific channel, rank,
    bankgroup, and bank IDs.

    Args:
        log_content (str): The full text content of the DRAM log file.
        target_channel (int, optional): The channel ID to filter by. Defaults to None (no filter).
        target_rank (int, optional): The rank ID to filter by. Defaults to None (no filter).
        target_bankgroup (int, optional): The bankgroup ID to filter by. Defaults to None (no filter).
        target_bank (int, optional): The bank ID to filter by. Defaults to None (no filter).

    Returns:
        tuple: A tuple containing:
            - dict: A dictionary where keys are unique row identifiers
                    (e.g., "Row 13412, bank 1, bg 6, rank 1, channel 0")
                    and values are lists of (event_type, cycle) tuples.
            - dict: A dictionary of rows that are currently open at the
                    end of the log, with their open cycle.
            - list: A list of dictionaries, each representing a chronological event
                    (row open/close, buffer snapshot, debug event).
    """
    row_history = defaultdict(list)
    
    # Dictionary to keep track of currently open rows for conflict detection:
    # Key: (channel, rank, bankgroup, bank)
    # Value: {'row_id': int, 'open_cycle': int, 'full_id': str}
    open_rows_state = {} 
    
    all_events = [] # To store all parsed events chronologically

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

    # Regex for debug events (HIT, MISS, CONFLICT)
    debug_pattern = re.compile(
        r"\[DEBUG\] Row (HIT|MISS|CONFLICT) for ch(\d+) rank(\d+) bg(\d+) bank(\d+) row (\d+)"
    )

    current_cycle = -1
    expecting_requests = False
    current_active_buffer_requests = []
    
    for line in log_content.splitlines():
        # Update current_cycle from any line that contains a cycle number
        # This is a heuristic, assuming cycle numbers are generally increasing.
        # More robust would be to parse cycle from every line type.
        cycle_match = re.search(r"at cycle (\d+)", line)
        if cycle_match:
            current_cycle = int(cycle_match.group(1))

        # Try to parse row events first
        row_match = row_pattern.match(line)
        if row_match:
            # If we were expecting requests, and a new row event appears,
            # it means the previous active buffer snapshot has ended.
            if expecting_requests and current_active_buffer_requests is not None:
                if current_cycle != -1: # Ensure we have a valid cycle for the snapshot
                    # Filter requests for the snapshot based on target filters before appending
                    filtered_requests_for_snapshot = []
                    for req in current_active_buffer_requests:
                        if (target_channel is None or req['channel'] == target_channel) and \
                           (target_rank is None or req['rank'] == target_rank) and \
                           (target_bankgroup is None or req['bankgroup'] == target_bankgroup) and \
                           (target_bank is None or req['bank'] == target_bank):
                            filtered_requests_for_snapshot.append(req)
                    
                    if filtered_requests_for_snapshot: # Only add snapshot if it contains filtered requests
                        all_events.append({
                            'type': 'buffer_snapshot',
                            'cycle': current_cycle,
                            'data': {'requests': filtered_requests_for_snapshot}
                        })
                current_active_buffer_requests = []
                expecting_requests = False

            # Process row event
            row_id_str, bank_str, bg_str, rank_str, channel_str, event_type, cycle_str, _ = row_match.groups()
            
            # Convert extracted string values to integers for comparison
            bank = int(bank_str)
            bg = int(bg_str)
            rank = int(rank_str)
            channel = int(channel_str)
            cycle = int(cycle_str)

            unique_location_tuple = (channel, rank, bg, bank)
            unique_row_identifier = (
                f"Row {row_id_str}, bank {bank_str}, bg {bg_str}, rank {rank_str}, channel {channel_str}"
            )

            # Apply filters for row events
            if target_channel is not None and channel != target_channel:
                continue
            if target_rank is not None and rank != target_rank:
                continue
            if target_bankgroup is not None and bg != target_bankgroup:
                continue
            if target_bank is not None and bank != target_bank:
                continue

            if event_type == "opened":
                row_history[unique_row_identifier].append(("opened", cycle))
                open_rows_state[unique_location_tuple] = {'row_id': int(row_id_str), 'open_cycle': cycle, 'full_id': unique_row_identifier}
                all_events.append({'type': 'row_open', 'cycle': cycle, 'data': {'row_id': unique_row_identifier}})
            elif event_type == "closed":
                row_history[unique_row_identifier].append(("closed", cycle))
                if unique_location_tuple in open_rows_state:
                    del open_rows_state[unique_location_tuple]
                all_events.append({'type': 'row_close', 'cycle': cycle, 'data': {'row_id': unique_row_identifier}})
            continue # Move to next line after processing row event

        # Try to parse active buffer header
        active_buffer_header_match = active_buffer_header_pattern.match(line)
        if active_buffer_header_match:
            # If we find a new header, it means the previous snapshot is complete
            if expecting_requests and current_active_buffer_requests is not None:
                if current_cycle != -1:
                    # Filter requests for the snapshot based on target filters before appending
                    filtered_requests_for_snapshot = []
                    for req in current_active_buffer_requests:
                        if (target_channel is None or req['channel'] == target_channel) and \
                           (target_rank is None or req['rank'] == target_rank) and \
                           (target_bankgroup is None or req['bankgroup'] == target_bankgroup) and \
                           (target_bank is None or req['bank'] == target_bank):
                            filtered_requests_for_snapshot.append(req)
                    
                    if filtered_requests_for_snapshot: # Only add snapshot if it contains filtered requests
                        all_events.append({
                            'type': 'buffer_snapshot',
                            'cycle': current_cycle,
                            'data': {'requests': filtered_requests_for_snapshot}
                        })
            current_active_buffer_requests = [] # Reset for new snapshot
            expecting_requests = True
            continue

        # If we are expecting requests, try to parse request lines
        if expecting_requests:
            request_match = request_pattern.match(line.strip())
            if request_match:
                cmd, channel_str, rank_str, bg_str, bank_str, row, column, final_cmd, req_type = request_match.groups()
                
                # Convert extracted string values to integers for filtering
                req_channel = int(channel_str)
                req_rank = int(rank_str)
                req_bg = int(bg_str)
                req_bank = int(bank_str)

                # Apply filters for active buffer requests (store all matching requests for the current snapshot)
                # Filtering logic is moved to when the snapshot is actually appended to all_events
                current_active_buffer_requests.append({
                    "cmd": int(cmd),
                    "channel": req_channel,
                    "rank": req_rank,
                    "bankgroup": req_bg,
                    "bank": req_bank,
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
                        # Filter requests for the snapshot based on target filters before appending
                        filtered_requests_for_snapshot = []
                        for req in current_active_buffer_requests:
                            if (target_channel is None or req['channel'] == target_channel) and \
                               (target_rank is None or req['rank'] == target_rank) and \
                               (target_bankgroup is None or req['bankgroup'] == target_bankgroup) and \
                               (target_bank is None or req['bank'] == target_bank):
                                filtered_requests_for_snapshot.append(req)
                        
                        if filtered_requests_for_snapshot: # Only add snapshot if it contains filtered requests
                            all_events.append({
                                'type': 'buffer_snapshot',
                                'cycle': current_cycle,
                                'data': {'requests': filtered_requests_for_snapshot}
                            })
                current_active_buffer_requests = []
                expecting_requests = False
            continue

        # New: Debug events (HIT, MISS, CONFLICT)
        debug_match = debug_pattern.match(line)
        if debug_match:
            event_type, ch_str, rk_str, bg_str, bk_str, row_str = debug_match.groups()
            
            # Convert extracted string values to integers for filtering
            ch = int(ch_str)
            rk = int(rk_str)
            bg = int(bg_str)
            bk = int(bk_str)
            row = int(row_str)

            debug_data = {
                'event': event_type,
                'channel': ch,
                'rank': rk,
                'bankgroup': bg,
                'bank': bk,
                'row': row
            }
            
            # Apply filters for debug events
            if target_channel is not None and ch != target_channel:
                continue
            if target_rank is not None and rk != target_rank:
                continue
            if target_bankgroup is not None and bg != target_bankgroup:
                continue
            if target_bank is not None and bk != target_bank:
                continue

            if event_type == "CONFLICT":
                location_tuple = (ch, rk, bg, bk)
                if location_tuple in open_rows_state:
                    debug_data['conflicting_open_row'] = open_rows_state[location_tuple]['full_id']
                    debug_data['conflicting_open_row_cycle'] = open_rows_state[location_tuple]['open_cycle']
                else:
                    debug_data['conflicting_open_row'] = "N/A (no row found open at this location)"
                    debug_data['conflicting_open_row_cycle'] = "N/A"

            all_events.append({'type': 'debug_event', 'cycle': current_cycle, 'data': debug_data})
            continue

    # After the loop, add any remaining active buffer requests
    if expecting_requests and current_active_buffer_requests:
        if current_cycle != -1:
            # Filter requests for the snapshot based on target filters before appending
            filtered_requests_for_snapshot = []
            for req in current_active_buffer_requests:
                if (target_channel is None or req['channel'] == target_channel) and \
                   (target_rank is None or req['rank'] == target_rank) and \
                   (target_bankgroup is None or req['bankgroup'] == target_bankgroup) and \
                   (target_bank is None or req['bank'] == target_bank):
                    filtered_requests_for_snapshot.append(req)
            
            if filtered_requests_for_snapshot: # Only add snapshot if it contains filtered requests
                all_events.append({
                    'type': 'buffer_snapshot',
                    'cycle': current_cycle,
                    'data': {'requests': filtered_requests_for_snapshot}
                })

    # Final processing of open_rows for the summary table
    final_open_rows_summary = {v['full_id']: v['open_cycle'] for k, v in open_rows_state.items()}

    return row_history, final_open_rows_summary, all_events

def display_results(row_history, open_rows_summary, all_events, output_file=None):
    """
    Displays the parsing results in a user-friendly format.

    Args:
        row_history (dict): A dictionary of row histories.
        open_rows_summary (dict): A dictionary of rows currently open at the end of the run.
        all_events (list): A list of all chronological events.
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
    if open_rows_summary:
        open_data = []
        for row_identifier, open_cycle in open_rows_summary.items():
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

    # --- Part 3: Chronological Log of Events (Filtered) ---
    write_func("## Chronological Log of Events (Filtered)")
    if all_events:
        all_events.sort(key=lambda x: x['cycle']) # Ensure strict chronological order

        for event in all_events:
            if event['type'] == 'row_open':
                write_func(f"Cycle {event['cycle']}: Row OPENED - {event['data']['row_id']}")
            elif event['type'] == 'row_close':
                write_func(f"Cycle {event['cycle']}: Row CLOSED - {event['data']['row_id']}")
            elif event['type'] == 'buffer_snapshot':
                write_func(f"Cycle {event['cycle']}: Active Buffer Snapshot (Size: {len(event['data']['requests'])})")
                if event['data']['requests']:
                    req_data = []
                    for req in event['data']['requests']:
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
                    write_func("  (No requests in buffer matching filter criteria)")
            elif event['type'] == 'debug_event':
                debug_data = event['data']
                if debug_data['event'] == 'CONFLICT':
                    write_func(f"Cycle {event['cycle']}: Row CONFLICT for ch{debug_data['channel']} rank{debug_data['rank']} bg{debug_data['bankgroup']} bank{debug_data['bank']} row {debug_data['row']}")
                    write_func(f"  Currently open row in this location: {debug_data['conflicting_open_row']} (opened at cycle {debug_data['conflicting_open_row_cycle']})")
                else:
                    write_func(f"Cycle {event['cycle']}: Row {debug_data['event']} for ch{debug_data['channel']} rank{debug_data['rank']} bg{debug_data['bankgroup']} bank{debug_data['bank']} row {debug_data['row']}")
            write_func("\n")
    else:
        write_func("No chronological events found in the log matching filter criteria.")


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
        print("Usage: python dram_parser.py <log_file_path> [output_file_path] [--channel <id>] [--rank <id>] [--bg <id>] [--bank <id>]", file=sys.stderr)
        sys.exit(1)

    log_file_path = sys.argv[1]
    
    # Parse command-line arguments for output file and filters
    i = 2
    while i < len(sys.argv):
        arg = sys.argv[i]
        if arg == "--channel":
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
        elif arg == "--bank":
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
    row_history, open_rows_summary, all_events = parse_dram_log(
        log_content,
        target_channel=target_channel,
        target_rank=target_rank,
        target_bankgroup=target_bg,
        target_bank=target_bank
    )
    display_results(row_history, open_rows_summary, all_events, output_file_path)
