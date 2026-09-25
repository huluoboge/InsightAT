# Stage InsightAT Windows CLI binaries into a zip.
#
# Env (set by CI or caller):
#   GITHUB_WORKSPACE / repo root (default: parent of packaging/)
#   VCPKG_ROOT, VCPKG_DEFAULT_TRIPLET
#   INSIGHTAT_ZIP_NAME  — output zip file name
#   CUDA_PATH           — optional, written into build-info.json

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = if ($env:GITHUB_WORKSPACE) { $env:GITHUB_WORKSPACE } else {
  (Resolve-Path (Join-Path $ScriptDir "..\..")).Path
}

$release_dir = Join-Path $RepoRoot "build\Release"
$stage_dir = Join-Path $RepoRoot "windows-package"
$zip_name = if ($env:INSIGHTAT_ZIP_NAME) { $env:INSIGHTAT_ZIP_NAME } else {
  "InsightAT-Windows.zip"
}
$zip_path = Join-Path $RepoRoot $zip_name

if (-not (Test-Path $release_dir)) {
  Write-Error "Release output directory not found: $release_dir"
  exit 1
}

$required = @(
  "isat_project.exe",
  "isat_sfm.exe",
  "isat_extract.exe",
  "isat_match.exe"
)
foreach ($name in $required) {
  $path = Join-Path $release_dir $name
  if (-not (Test-Path $path)) {
    Write-Error "Required Windows executable was not built: $path"
    exit 1
  }
}

Remove-Item $stage_dir -Recurse -Force -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Path $stage_dir | Out-Null
Get-ChildItem -Path $release_dir -Filter "isat_*.exe" -File |
  Copy-Item -Destination $stage_dir -Force
Get-ChildItem -Path (Join-Path $RepoRoot "build") -Recurse -Filter "*.dll" -File -ErrorAction SilentlyContinue |
  Copy-Item -Destination $stage_dir -Force -ErrorAction SilentlyContinue
if (Test-Path (Join-Path $RepoRoot "data")) {
  Copy-Item (Join-Path $RepoRoot "data") (Join-Path $stage_dir "data") -Recurse -Force
}

$triplet = if ($env:VCPKG_DEFAULT_TRIPLET) { $env:VCPKG_DEFAULT_TRIPLET } else { "x64-windows-release" }
if ($env:VCPKG_ROOT) {
  $triplet_bin = Join-Path $env:VCPKG_ROOT "installed\$triplet\bin"
  if (Test-Path $triplet_bin) {
    Copy-Item (Join-Path $triplet_bin "*.dll") $stage_dir -Force -ErrorAction SilentlyContinue
  }
}

@{
  project = "InsightAT"
  cuda = $env:CUDA_PATH
  triplet = $triplet
  commit = $env:GITHUB_SHA
} | ConvertTo-Json | Set-Content (Join-Path $stage_dir "build-info.json")

& (Join-Path $stage_dir "isat_project.exe") create -h
if ($LASTEXITCODE -ne 0) {
  Write-Error "isat_project.exe create -h smoke test failed"
  exit 1
}

if (Test-Path $zip_path) { Remove-Item $zip_path -Force }
Compress-Archive -Path (Join-Path $stage_dir "*") -DestinationPath $zip_path -CompressionLevel Optimal -Force
Write-Host "Windows package: $zip_path"
