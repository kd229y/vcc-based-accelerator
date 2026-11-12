param(
    [ValidateSet('smoke','modules','integration','system','regression','all')]
    [string]$Suite = 'smoke',
    [switch]$Baseline,
    [switch]$Rebuild
)
$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
$validationPython = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $validationPython)) { throw 'Create .venv and install requirements_runtime.txt first; see README_執行與結果.md.' }
if ($Rebuild -or -not (Test-Path -LiteralPath (Join-Path $PSScriptRoot 'build\build_report.json'))) {
    & $validationPython 'runner\build.py'
    if ($LASTEXITCODE -ne 0) { throw 'Build failed. Check build/baseline_build.log.' }
}
$validationArguments = @('runner\run_validation.py','--suite',$Suite)
if ($Baseline) { $validationArguments += '--baseline' }
& $validationPython @validationArguments
exit $LASTEXITCODE
