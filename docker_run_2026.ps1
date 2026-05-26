$ErrorActionPreference = "Stop"

$repo = Split-Path -Parent $MyInvocation.MyCommand.Path
$image = "cyberdog_sim:v2026"
$name = "cyberdog2026"
$distro = $env:WSL_DISTRO
if ([string]::IsNullOrWhiteSpace($distro)) {
  $distro = "Ubuntu-22.04"
}

$repoWsl = (wsl.exe -d $distro -- wslpath -u "$repo").Trim()
if ([string]::IsNullOrWhiteSpace($repoWsl)) {
  throw "Unable to convert repo path to WSL path: $repo"
}

wsl.exe -d $distro -- docker rm -f $name 2>$null | Out-Null

$args = @(
  "-d", $distro, "--",
  "docker", "run", "-dit",
  "--name", $name,
  "--privileged",
  "-e", "DISPLAY=:0",
  "-e", "WAYLAND_DISPLAY=wayland-0",
  "-e", "XDG_RUNTIME_DIR=/run/user/1000",
  "-v", "${repoWsl}:/workspace/xiaomi_cup",
  "-v", "/tmp/.X11-unix:/tmp/.X11-unix",
  "-v", "/mnt/wslg:/mnt/wslg",
  $image,
  "bash"
)

wsl.exe @args

Write-Host "Started $name from $image"
Write-Host "WSL distro: $distro"
Write-Host "Code is mounted at /workspace/xiaomi_cup"
