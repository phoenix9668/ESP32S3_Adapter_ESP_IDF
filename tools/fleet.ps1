$ErrorActionPreference = "Stop"
if (-not $env:IDF_PATH) {
    throw "IDF_PATH is not set. Open an ESP-IDF v5.5.4 PowerShell first."
}
$ProjectDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
& python (Join-Path $ProjectDir "tools\fleet.py") @args
if ($LASTEXITCODE -ne 0) {
    throw "fleet.py failed with exit code $LASTEXITCODE"
}
