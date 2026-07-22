[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
$ProjectDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path

if (-not $env:IDF_PATH) {
    throw "IDF_PATH is not set. Open an ESP-IDF v5.5.4 PowerShell first."
}
$IdfVersion = (& idf.py --version | Out-String).Trim()
if ($LASTEXITCODE -ne 0 -or $IdfVersion -notmatch "ESP-IDF v5\.5\.4") {
    throw "ESP-IDF v5.5.4 is required; active version: $IdfVersion"
}

$KeyPath = if ($env:OTA_SIGNING_KEY) {
    [IO.Path]::GetFullPath($env:OTA_SIGNING_KEY)
} else {
    Join-Path $ProjectDir "keys\ota_signing_key.pem"
}
if (Test-Path -LiteralPath $KeyPath) {
    throw "Refusing to replace existing signing key: $KeyPath"
}

New-Item -ItemType Directory -Force -Path (Split-Path -Parent $KeyPath) | Out-Null
$Espsecure = Join-Path $env:IDF_PATH `
    "components\esptool_py\esptool\espsecure.py"
& python $Espsecure generate_signing_key --version 2 $KeyPath
if ($LASTEXITCODE -ne 0) {
    throw "Signing key generation failed with exit code $LASTEXITCODE"
}

Write-Host "Generated RSA-3072 OTA signing key: $KeyPath"
Write-Host "Back up this file to at least two offline encrypted media before manufacturing."
if ($env:OTA_SIGNING_KEY) {
    Write-Host "build_ota.ps1 will temporarily stage this offline key at the project signing path."
}
