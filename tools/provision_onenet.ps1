[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [string]$Port = ""
)

$ErrorActionPreference = "Stop"
$ProjectDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$NvsSize = "0x6000"
$NvsOffset = "0x9000"

if (-not $env:IDF_PATH) {
    throw "IDF_PATH is not set. Open an ESP-IDF v5.5.4 PowerShell first."
}

$IdfVersion = (& idf.py --version | Out-String).Trim()
if ($LASTEXITCODE -ne 0 -or $IdfVersion -notmatch "ESP-IDF v5\.5\.4") {
    throw "ESP-IDF v5.5.4 is required; active version: $IdfVersion"
}

$Csv = if ($env:ONENET_NVS_CSV) {
    [IO.Path]::GetFullPath($env:ONENET_NVS_CSV)
} else {
    Join-Path $ProjectDir "tools\onenet_nvs.csv"
}
$Bin = if ($env:ONENET_NVS_BIN) {
    [IO.Path]::GetFullPath($env:ONENET_NVS_BIN)
} else {
    Join-Path $ProjectDir "build\onenet_nvs.bin"
}

if (-not (Test-Path -LiteralPath $Csv -PathType Leaf)) {
    throw "Missing $Csv; copy tools\onenet_nvs.csv.example and fill it locally."
}

$Generator = Join-Path $env:IDF_PATH `
    "components\nvs_flash\nvs_partition_generator\nvs_partition_gen.py"
$Esptool = Join-Path $env:IDF_PATH `
    "components\esptool_py\esptool\esptool.py"
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $Bin) | Out-Null

& python $Generator generate $Csv $Bin $NvsSize
if ($LASTEXITCODE -ne 0) {
    throw "NVS generation failed with exit code $LASTEXITCODE"
}
Write-Host "Generated $Bin"

if ($Port) {
    & python $Esptool --chip esp32s3 --port $Port write_flash $NvsOffset $Bin
    if ($LASTEXITCODE -ne 0) {
        throw "NVS flashing failed with exit code $LASTEXITCODE"
    }
    Write-Host "Flashed OneNET NVS partition through $Port"
} else {
    Write-Host "Pass a COM port as the first argument to flash offset $NvsOffset."
}
