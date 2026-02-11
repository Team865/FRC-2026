@echo off
git fetch --prune
for /f "delims=" %%b in ('git branch --format="%%(refname:short)" ^| findstr /v "^main$"') do (
    git ls-remote --exit-code --heads origin %%b >nul 2>&1
    if errorlevel 1 (
        git branch -D %%b
    )
)
pause

