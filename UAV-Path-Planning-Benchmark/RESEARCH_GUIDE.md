# Research Guide: UAV Path Planning Benchmark

This repository is designed for rigorous multi-objective path planning research. Follow the steps below to conduct experiments and analyze results.

## 1. Project Structure
- `/core`: Shared logic (Path representation, B-splines, Constraints).
- `/core/metrics`: Implementation of HV (Hypervolume) and PD (Pure Diversity).
- `/algorithms`: Modular implementation of optimization algorithms (e.g., NSGA-II).
- `/problems`: Standardized terrain scenarios (`terrainStruct`).
- `/scripts`: Entry points for running experiments.
- `/results`: Raw output data (`.mat` files) organized by problem and run.
- `/analysis`: Scripts for statistical tests and visualization.

## 2. Running Experiments
1. Open MATLAB in the project root.
2. Run `startup.m` to initialize paths.
3. Use `scripts/run_benchmark.m` to execute the full suite.
   - Modify `params` in this script to change population size, generations, or number of runs.

## 3. Adding a New Algorithm
1. Create a new folder in `/algorithms/YourAlgorithm`.
2. Implement your algorithm as a function that accepts `model` and `params`.
3. Add its path to `startup.m`.
4. Update `scripts/run_benchmark.m` to include your new algorithm in the comparison loop.

## 4. Statistical Analysis
After running the benchmark, use `analysis/statistical_analysis.m` (coming soon) to:
- Generate mean/std tables for all metrics.
- Perform Wilcoxon Rank-Sum tests for significance.
- Export results to LaTeX format.

## 5. Visualizing Paths
Use `analysis/path_visualizer.m` to generate 3D plots of the resulting Pareto front paths.
