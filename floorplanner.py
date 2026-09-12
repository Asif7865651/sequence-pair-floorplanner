"""
Sequence-Pair VLSI Floorplanner
--------------------------------
Given a set of rectangular modules and a Sequence Pair (positive/negative
sequence), this tool:

  1. Derives horizontal (HCG) and vertical (VCG) constraint graphs from the
     sequence pair.
  2. Computes lower-left corner coordinates for every module via longest-path
     calculations on the HCG/VCG (source -> module).
  3. Plots the resulting floorplan.
  4. Runs Random-Restart Hill Climbing to minimize total floorplan area,
     using three move types:
         - Swap two modules in the positive sequence (S1)
         - Swap two modules in the negative sequence (S2)
         - Swap two modules in both sequences ("Both")
         - Rotate a single module (w <-> h)
     and logs every improving move, plus the final move that produced the
     best area found.

Constraint rules used (derived from the sequence pair positions):
  Horizontal: A is left of B  if pos+(A) < pos+(B) and pos-(A) < pos-(B)
  Vertical:   A is below   B  if pos+(A) > pos+(B) and pos-(A) < pos-(B)
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import itertools
import copy
import random


# --- Function to get all inputs from the user in the terminal ---
def get_inputs_from_terminal():
    """Asks the user for all module and sequence data in the terminal."""
    print("--- Enter Your Floorplan Inputs ---")
    modules = {}

    # 1. Get number of modules
    while True:
        try:
            num_modules = int(input("How many modules are there? "))
            if num_modules > 0:
                break
        except ValueError:
            print("Invalid input. Please enter a number.")

    # 2. Get dimensions for each module
    print("\nEnter module names and dimensions:")
    for i in range(num_modules):
        module_name = input(f"  Enter name for module {i + 1} (e.g., '1', 'A'): ").strip()
        while True:
            try:
                dims_str = input(f"    Enter Width and Height for module '{module_name}' (e.g., '3 5'): ")
                w_str, h_str = dims_str.split()
                w, h = int(w_str), int(h_str)
                modules[module_name] = {'w': w, 'h': h}
                break
            except ValueError:
                print("    Invalid format. Please enter as 'width height' (e.g., '3 5').")

    # 3. Get the sequences
    print(f"\nEnter the sequences as {num_modules} space-separated names (e.g., 'A B C D')")
    module_name_set = set(modules.keys())

    while True:
        pos_seq = input("  Enter the Positive Sequence (+): ").split()
        if len(pos_seq) == num_modules and set(pos_seq) == module_name_set:
            break
        print(f"    Error: Sequence must contain all {num_modules} module names exactly once.")

    while True:
        neg_seq = input("  Enter the Negative Sequence (-): ").split()
        if len(neg_seq) == num_modules and set(neg_seq) == module_name_set:
            break
        print(f"    Error: Sequence must contain all {num_modules} module names exactly once.")

    print("--- Inputs Received. Starting Calculation. ---")
    return modules, (pos_seq, neg_seq)


# --- CORE ALGORITHM: SEQUENCE PAIR EVALUATOR ---
def evaluate_sp(solution):
    """
    Takes a solution ({'sp': (S1, S2), 'modules': {...}}) and returns
    (area, {module: (x, y)}, total_width, total_height).
    """
    modules = solution['modules']
    seq_pair = solution['sp']
    module_names = list(modules.keys())

    # 1. Derive Constraints
    pos_plus = {name: i for i, name in enumerate(seq_pair[0])}
    pos_minus = {name: i for i, name in enumerate(seq_pair[1])}
    h_constraints, v_constraints = [], []

    for i in range(len(module_names)):
        for j in range(i + 1, len(module_names)):
            a, b = module_names[i], module_names[j]

            # Horizontal: A is left of B if it's before B in both sequences
            if pos_plus[a] < pos_plus[b] and pos_minus[a] < pos_minus[b]:
                h_constraints.append((a, b))
            elif pos_plus[b] < pos_plus[a] and pos_minus[b] < pos_minus[a]:
                h_constraints.append((b, a))

            # Vertical: A is below B if it's after B in S1 and before B in S2
            if pos_plus[a] > pos_plus[b] and pos_minus[a] < pos_minus[b]:
                v_constraints.append((a, b))
            elif pos_plus[b] > pos_plus[a] and pos_minus[b] < pos_minus[a]:
                v_constraints.append((b, a))

    # 2. Build Graphs and Find Longest Paths
    def calculate_longest_paths(constraints, dim_key):
        nodes = module_names + ['source', 'sink']
        graph = {n: [] for n in nodes}
        in_degree = {n: 0 for n in nodes}
        for u, v in constraints:
            graph[u].append((v, modules[u][dim_key]))
            in_degree[v] += 1
        for m in module_names:
            if in_degree[m] == 0:
                graph['source'].append((m, 0))
                in_degree[m] += 1
            if not any(m == u for u, v in constraints):
                graph[m].append(('sink', modules[m][dim_key]))
                in_degree['sink'] += 1

        queue = [n for n in nodes if in_degree[n] == 0]
        topo_order = []
        while queue:
            u = queue.pop(0)
            topo_order.append(u)
            for v, w in graph[u]:
                in_degree[v] -= 1
                if in_degree[v] == 0:
                    queue.append(v)

        distances = {n: 0 for n in nodes}
        for u in topo_order:
            for v, w in graph[u]:
                if distances[u] + w > distances[v]:
                    distances[v] = distances[u] + w
        return distances

    x_distances = calculate_longest_paths(h_constraints, 'w')
    y_distances = calculate_longest_paths(v_constraints, 'h')

    # 3. Final Results
    final_coords = {name: (x_distances[name], y_distances[name]) for name in module_names}
    total_width = x_distances['sink']
    total_height = y_distances['sink']
    area = total_width * total_height

    return area, final_coords, total_width, total_height


# --- VISUALIZATION FUNCTION ---
def plot_floorplan(modules_with_coords, total_width, total_height, area, title="Floorplan"):
    """Draws the final floorplan plot."""
    fig, ax = plt.subplots(1, figsize=(10, 12))
    ax.set_xlim(0, total_width)
    ax.set_ylim(0, total_height)
    ax.set_aspect('equal')

    colors = ['#FF9999', '#66B2FF', '#99FF99', '#FFCC99', '#c2c2f0', '#ffb3e6', '#c4e17f', '#76D7C4']

    for i, (name, data) in enumerate(modules_with_coords.items()):
        x, y, w, h = data['x'], data['y'], data['w'], data['h']
        color = colors[i % len(colors)]
        rect = patches.Rectangle((x, y), w, h, linewidth=2, edgecolor='black', facecolor=color, alpha=0.9)
        ax.add_patch(rect)
        plt.text(x + w / 2, y + h / 2, name, ha='center', va='center', fontsize=16, fontweight='bold')

    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xticks(range(int(total_width) + 2))
    plt.yticks(range(int(total_height) + 2))
    plt.xlabel("Width", fontsize=12)
    plt.ylabel("Height", fontsize=12)
    plt.title(f"{title}\nArea = {area}", fontsize=14)
    plt.tight_layout()
    plt.show()


# --- MAIN EXECUTION: RANDOM-RESTART HILL CLIMBING ---
if __name__ == "__main__":
    # --- 1. Get User Inputs ---
    INITIAL_MODULES, initial_sp = get_inputs_from_terminal()
    module_names = list(INITIAL_MODULES.keys())

    # --- 2. Evaluate and Plot Initial Solution ---
    initial_solution = {'sp': initial_sp, 'modules': copy.deepcopy(INITIAL_MODULES)}
    initial_area, initial_coords, initial_w, initial_h = evaluate_sp(initial_solution)
    print(f"\nInitial Area: {initial_area}")

    print("\nCalculated Initial Lower-Left Corner Coordinates:")
    for name in sorted(initial_coords.keys(), key=lambda x: str(x)):
        print(f"  Module {name}: {initial_coords[name]}")

    initial_modules_for_plot = copy.deepcopy(initial_solution['modules'])
    for name, (x, y) in initial_coords.items():
        initial_modules_for_plot[name]['x'] = x
        initial_modules_for_plot[name]['y'] = y
    plot_floorplan(initial_modules_for_plot, initial_w, initial_h, initial_area, title="Initial Floorplan")

    # --- 3. Optimization Setup ---
    NUM_RESTARTS = 100

    best_solution_overall = copy.deepcopy(initial_solution)
    best_area_overall = initial_area
    last_move_that_led_to_best = "No improvement found (started at optimal)."

    print(f"\nStarting Random-Restart Hill Climbing for {NUM_RESTARTS} restarts...")

    for i in range(NUM_RESTARTS):
        if i == 0:
            current_solution = copy.deepcopy(initial_solution)
            print(f"--- Restart 1/{NUM_RESTARTS}: Using user-provided starting point. ---")
        else:
            random.shuffle(module_names)
            s1 = list(module_names)
            random.shuffle(module_names)
            s2 = list(module_names)
            current_solution = {'sp': (s1, s2), 'modules': copy.deepcopy(INITIAL_MODULES)}
            print(f"\n--- Restart {i + 1}/{NUM_RESTARTS}: Starting from a new random solution. ---")

        current_best_area, _, _, _ = evaluate_sp(current_solution)
        last_move_for_this_restart = "Started at local optimum."

        # --- Run one full Hill Climb ---
        while True:
            improvement_found = False

            for mod1, mod2 in itertools.combinations(module_names, 2):
                for move_type in ['S1', 'S2', 'Both']:
                    candidate_solution = copy.deepcopy(current_solution)
                    s1, s2 = list(candidate_solution['sp'][0]), list(candidate_solution['sp'][1])
                    if move_type == 'S1':
                        idx1, idx2 = s1.index(mod1), s1.index(mod2)
                        s1[idx1], s1[idx2] = s1[idx2], s1[idx1]
                    elif move_type == 'S2':
                        idx1, idx2 = s2.index(mod1), s2.index(mod2)
                        s2[idx1], s2[idx2] = s2[idx2], s2[idx1]
                    elif move_type == 'Both':
                        idx1_s1, idx2_s1 = s1.index(mod1), s1.index(mod2)
                        idx1_s2, idx2_s2 = s2.index(mod1), s2.index(mod2)
                        s1[idx1_s1], s1[idx2_s1] = s1[idx2_s1], s1[idx1_s1]
                        s2[idx1_s2], s2[idx2_s2] = s2[idx2_s2], s2[idx1_s2]

                    candidate_solution['sp'] = (tuple(s1), tuple(s2))
                    new_area, _, _, _ = evaluate_sp(candidate_solution)

                    if new_area < current_best_area:
                        move_desc = f"Swap {mod1}<->{mod2} in '{move_type}' sequence(s), New Area: {new_area}"
                        print(f"  > Improvement Found: Area {current_best_area} -> {new_area} ({move_desc})")
                        last_move_for_this_restart = move_desc
                        current_best_area, current_solution, improvement_found = new_area, candidate_solution, True

            for mod_to_rotate in module_names:
                candidate_solution = copy.deepcopy(current_solution)
                w, h = candidate_solution['modules'][mod_to_rotate]['w'], candidate_solution['modules'][mod_to_rotate]['h']
                candidate_solution['modules'][mod_to_rotate]['w'] = h
                candidate_solution['modules'][mod_to_rotate]['h'] = w
                new_area, _, _, _ = evaluate_sp(candidate_solution)

                if new_area < current_best_area:
                    move_desc = f"Rotate module '{mod_to_rotate}', New Area: {new_area}"
                    print(f"  > Improvement Found: Area {current_best_area} -> {new_area} ({move_desc})")
                    last_move_for_this_restart = move_desc
                    current_best_area, current_solution, improvement_found = new_area, candidate_solution, True

            if not improvement_found:
                print(f"--- Restart {i + 1} finished. No further improvements. ---")
                break

        # --- Compare this run's result to the all-time best ---
        if current_best_area < best_area_overall:
            best_area_overall = current_best_area
            best_solution_overall = copy.deepcopy(current_solution)
            last_move_that_led_to_best = last_move_for_this_restart
            print(f"*** New Best Overall Area Found: {best_area_overall} ***")

    # --- 4. Final Result ---
    print("\nOptimization finished!")
    print(f"Minimized Area Found: {best_area_overall}")
    print(f"Best Sequence Pair Found: {best_solution_overall['sp']}")
    print(f"Final move of the winning run: {last_move_that_led_to_best}")

    final_area, final_coords, final_w, final_h = evaluate_sp(best_solution_overall)
    final_modules_for_plot = copy.deepcopy(best_solution_overall['modules'])
    for name, (x, y) in final_coords.items():
        final_modules_for_plot[name]['x'] = x
        final_modules_for_plot[name]['y'] = y
    plot_floorplan(final_modules_for_plot, final_w, final_h, final_area, title="Minimized Floorplan")
