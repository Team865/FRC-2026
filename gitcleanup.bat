@echo off
git fetch --prune

echo Are you sure? Press any key after this to go through with the deletion
pause

for /f "delims=" %%b in ('git branch --format="%%(refname:short)" ^| findstr /v "^main$"') do (
    git ls-remote --exit-code --heads origin %%b >nul 2>&1

    if errorlevel 1 (
        git branch -D %%b
    )
)