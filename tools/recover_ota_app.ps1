[CmdletBinding()]
param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string]$Port,

    [Parameter(Mandatory = $true, Position = 1)]
    [string]$Application
)

$ErrorActionPreference = "Stop"
$OtaDataOffset = "0xF000"
$OtaDataSize = "0x2000"
$Ota0Offset = "0x20000"
$OtaSlotSize = 0x400000

if (-not $env:IDF_PATH) {
    throw "IDF_PATH is not set. Open an ESP-IDF v5.5.4 PowerShell first."
}
$IdfVersion = (& idf.py --version | Out-String).Trim()
if ($LASTEXITCODE -ne 0 -or $IdfVersion -notmatch "ESP-IDF v5\.5\.4") {
    throw "ESP-IDF v5.5.4 is required; active version: $IdfVersion"
}
if (-not (Test-Path -LiteralPath $Application -PathType Leaf)) {
    throw "Application image not found: $Application"
}

$AppBin = (Resolve-Path -LiteralPath $Application).Path
$AppSize = (Get-Item -LiteralPath $AppBin).Length
if ($AppSize -le 0 -or $AppSize -gt $OtaSlotSize) {
    throw "Application size $AppSize does not fit the 4 MiB OTA slot."
}

$Esptool = Join-Path $env:IDF_PATH `
    "components\esptool_py\esptool\esptool.py"
Write-Host "Recovery image: $AppBin ($AppSize bytes)"
Write-Host "Erasing only otadata at $OtaDataOffset, size $OtaDataSize."
Write-Host "OneNET NVS and rfid_store will not be erased."

& python $Esptool --chip esp32s3 --port $Port --after no_reset `
    erase_region $OtaDataOffset $OtaDataSize
if ($LASTEXITCODE -ne 0) {
    throw "otadata erase failed with exit code $LASTEXITCODE"
}
& python $Esptool --chip esp32s3 --port $Port --after hard_reset `
    write_flash --verify --flash_mode keep --flash_freq keep --flash_size keep `
    $Ota0Offset $AppBin
if ($LASTEXITCODE -ne 0) {
    throw "Recovery application flash failed with exit code $LASTEXITCODE"
}

Write-Host "Recovery application written to ota_0 and boot selection reset."
Write-Host "Monitor with: idf.py -p $Port monitor"
