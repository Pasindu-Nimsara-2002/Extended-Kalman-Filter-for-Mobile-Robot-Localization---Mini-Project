@echo off
REM Build the LaTeX report (requires pdflatex on PATH)
pdflatex -interaction=nonstopmode MiniProject_Report.tex
pdflatex -interaction=nonstopmode MiniProject_Report.tex
if exist MiniProject_Report.pdf (
  echo Build succeeded: MiniProject_Report.pdf
) else (
  echo Build failed. Check the .log file.
)
pause
