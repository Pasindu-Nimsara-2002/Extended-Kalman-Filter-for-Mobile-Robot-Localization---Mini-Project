LaTeX report build instructions

1. Ensure a LaTeX distribution is installed (TeX Live or MiKTeX) and `pdflatex` is available on PATH.
2. Place `ekf_slam_results.png` (or your result figures) alongside `MiniProject_Report.tex`.
3. From the project directory run:

```powershell
build_report.bat
```

4. If compilation fails, inspect `MiniProject_Report.log` for errors and re-run after fixing missing packages or image files.

Notes:
- For reproducible MATLAB figures, set `rng(0)` at top of `ekf_slam_v01.m` and save the plotted figure as `ekf_slam_results.png` before running LaTeX build.
- The LaTeX file is `MiniProject_Report.tex` — edit sections to insert numerical results and additional figures.
