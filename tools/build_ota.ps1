[CmdletBinding()]
param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string]$Version
)

$ErrorActionPreference = "Stop"
$ProjectDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path

if ($Version -notmatch "^[0-9]+\.[0-9]+\.[0-9]+([.-][0-9A-Za-z]+)*$" -or
    $Version.Length -gt 20) {
    throw "Invalid OneNET application version: $Version"
}
$OutBaseName = "s3-$Version.bin"
if ($OutBaseName.Length -gt 20) {
    throw "OneNET OTA filename exceeds 20 characters; use a version no longer than 13 characters."
}
if (-not $env:IDF_PATH) {
    throw "IDF_PATH is not set. Open an ESP-IDF v5.5.4 PowerShell first."
}
$IdfVersion = (& idf.py --version | Out-String).Trim()
if ($LASTEXITCODE -ne 0 -or $IdfVersion -notmatch "ESP-IDF v5\.5\.4") {
    throw "ESP-IDF v5.5.4 is required; active version: $IdfVersion"
}

$VersionFile = (Get-Content -LiteralPath (Join-Path $ProjectDir "version.txt") `
    -Raw).Trim()
if ($VersionFile -ne $Version) {
    throw "Version mismatch: version.txt=$VersionFile, requested=$Version"
}

$ConfiguredKey = Join-Path $ProjectDir "keys\ota_signing_key.pem"
$KeyPath = if ($env:OTA_SIGNING_KEY) {
    [IO.Path]::GetFullPath($env:OTA_SIGNING_KEY)
} else {
    $ConfiguredKey
}
if (-not (Test-Path -LiteralPath $KeyPath -PathType Leaf)) {
    throw "Missing OTA signing key: $KeyPath"
}

$KeyPath = (Resolve-Path -LiteralPath $KeyPath).Path
$ConfiguredKeyFull = [IO.Path]::GetFullPath($ConfiguredKey)
$TemporaryKey = $false
$OldReleaseVersion = $env:APP_RELEASE_VERSION

try {
    if ($KeyPath -ne $ConfiguredKeyFull) {
        if (Test-Path -LiteralPath $ConfiguredKeyFull) {
            throw "Cannot use OTA_SIGNING_KEY while $ConfiguredKeyFull already exists."
        }
        New-Item -ItemType Directory -Force -Path `
            (Split-Path -Parent $ConfiguredKeyFull) | Out-Null
        Copy-Item -LiteralPath $KeyPath -Destination $ConfiguredKeyFull
        $TemporaryKey = $true
    }

    $env:APP_RELEASE_VERSION = $Version
    Push-Location $ProjectDir
    try {
        & idf.py fullclean build
        if ($LASTEXITCODE -ne 0) {
            throw "ESP-IDF build failed with exit code $LASTEXITCODE"
        }
    } finally {
        Pop-Location
    }

    $AppBin = Join-Path $ProjectDir "build\ESP32S3_Adapter_ESP_IDF.bin"
    $Espsecure = Join-Path $env:IDF_PATH `
        "components\esptool_py\esptool\espsecure.py"
    & python $Espsecure verify_signature --version 2 --keyfile $KeyPath $AppBin
    if ($LASTEXITCODE -ne 0) {
        throw "RSA application signature verification failed."
    }

    $OutDir = Join-Path $ProjectDir "dist"
    $OutBin = Join-Path $OutDir $OutBaseName
    $Manifest = Join-Path $OutDir "s3-$Version.manifest.json"
    New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
    Copy-Item -LiteralPath $AppBin -Destination $OutBin -Force

    $Payload = [IO.File]::ReadAllBytes($OutBin)
    $Md5Object = [Security.Cryptography.MD5]::Create()
    $ShaObject = [Security.Cryptography.SHA256]::Create()
    try {
        $Md5 = ($Md5Object.ComputeHash($Payload) | ForEach-Object {
            $_.ToString("x2")
        }) -join ""
        $Sha256 = ($ShaObject.ComputeHash($Payload) | ForEach-Object {
            $_.ToString("x2")
        }) -join ""
    } finally {
        $Md5Object.Dispose()
        $ShaObject.Dispose()
    }

    $GitCommit = (& git -C $ProjectDir rev-parse HEAD | Out-String).Trim()
    if ($LASTEXITCODE -ne 0) {
        throw "Cannot determine the Git commit for the release manifest."
    }
    & git -C $ProjectDir diff --quiet --ignore-submodules HEAD
    $GitDirty = $LASTEXITCODE -ne 0

    $ManifestData = [ordered]@{
        artifact = $OutBaseName
        version = $Version
        size = $Payload.Length
        md5 = $Md5
        sha256 = $Sha256
        git_commit = $GitCommit
        git_dirty = $GitDirty
        built_at_utc = [DateTimeOffset]::UtcNow.ToString("yyyy-MM-ddTHH:mm:sszzz")
        idf_version = "v5.5.4"
        upload_to_onenet = $true
        image_kind = "signed ESP-IDF application only"
    }
    $Json = $ManifestData | ConvertTo-Json -Depth 3
    $Utf8NoBom = New-Object Text.UTF8Encoding($false)
    [IO.File]::WriteAllText($Manifest, $Json + [Environment]::NewLine, $Utf8NoBom)

    Write-Host "OneNET upload file: $OutBin"
    Write-Host "Local release manifest: $Manifest"
    Write-Host "Do not upload merged factory images, bootloader, partition table, or NVS."
} finally {
    if ($null -eq $OldReleaseVersion) {
        Remove-Item Env:APP_RELEASE_VERSION -ErrorAction SilentlyContinue
    } else {
        $env:APP_RELEASE_VERSION = $OldReleaseVersion
    }
    if ($TemporaryKey) {
        Remove-Item -LiteralPath $ConfiguredKeyFull -Force -ErrorAction SilentlyContinue
    }
}
