# Sequence-Pair VLSI Floorplanner

A terminal-based VLSI floorplanning tool built around the **Sequence Pair**
representation, with a **Random-Restart Hill Climbing** optimizer to
minimize total floorplan area.

## How it works

1. **Input**: number of modules, each module's (width, height), and a
   positive/negative sequence pair.
2. **Constraint graphs**: the sequence pair is converted into a Horizontal
   Constraint Graph (HCG) and Vertical Constraint Graph (VCG):
   - `A` is **left of** `B` if `A` appears before `B` in *both* sequences.
   - `A` is **below** `B` if `A` appears after `B` in the positive sequence
     and before `B` in the negative sequence.
3. **Coordinates**: each module's lower-left corner is the **longest path**
   from a `source` node to that module in the HCG (x) and VCG (y) — edge
   weight from a module equals that module's width (HCG) or height (VCG).
   Total chip width/height is the longest source→sink path in each graph.
4. **Optimization**: Random-Restart Hill Climbing searches for the minimum
   area using four move types, tested exhaustively each sweep, taking the
   first improving move found:
   - Swap two modules in the positive sequence (`S1`)
   - Swap two modules in the negative sequence (`S2`)
   - Swap two modules in both sequences (`Both`)
   - Rotate a single module (swap its width/height)

   Each hill climb runs until no move improves the area (a local minimum).
   Restarting from many random sequence pairs (default: 100) increases the
   chance of finding the true global minimum instead of getting stuck in
   the first local minimum found.
5. **Output**: the initial floorplan and the minimized floorplan are each
   plotted with `matplotlib`, and every improving move is logged to the
   terminal, along with the specific move that produced the best area found
   overall.

## Usage

```bash
pip install matplotlib
python floorplanner.py
```

You'll be prompted for:
- Number of modules
- Each module's name and `width height`
- The positive sequence (space-separated module names)
- The negative sequence (space-separated module names)

### Example input (6 modules)

```
Modules (name: width height):
  1: 2 1
  2: 2 4
  3: 1 3
  4: 3 2
  5: 1 4
  6: 4 2

Positive sequence: 1 3 4 5 2 6
Negative sequence: 5 6 3 2 4 1
```

See `sample_input.txt` for a copy-pasteable version of this example.

## Notes

- The number of candidate moves checked per sweep is `3 * C(n, 2) + n` for
  `n` modules (3 swap types over all module pairs, plus one rotation per
  module) — e.g. 92 moves for 8 modules, 51 for 6 modules.
- `NUM_RESTARTS` (default 100) is easily adjustable at the top of the
  `__main__` block in `floorplanner.py`.
