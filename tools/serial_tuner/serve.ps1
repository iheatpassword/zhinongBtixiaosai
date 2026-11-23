<#
.SYNOPSIS
  C30D 串口调参台 —— 启动本地静态服务器（Python http.server）。

.DESCRIPTION
  Web Serial 只在“安全上下文”可用：http://localhost / http://127.0.0.1 或 https。
  直接双击 index.html（file://）会因为 ES module 的 CORS 限制而白屏，所以必须用本脚本
  （或 python -m http.server 8765）起一个本地服务器再访问。

  默认在脚本所在目录启动，端口 8765，并尝试用默认浏览器打开。

.PARAMETER Port
  监听端口，默认 8765。

.PARAMETER NoBrowser
  只启动服务器，不自动打开浏览器。

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\serve.ps1

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\serve.ps1 -Port 9000 -NoBrowser
#>
[CmdletBinding()]
param(
  [int]$Port = 8765,
  [switch]$NoBrowser
)

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

# 找一个可用的 Python（python 或 py 启动器）
$pythonExe = $null
foreach ($name in @('python', 'py')) {
  $cmd = Get-Command $name -ErrorAction SilentlyContinue
  if ($null -ne $cmd) {
    $pythonExe = $cmd.Source
    break
  }
}

$url = "http://localhost:$Port"

if (-not $pythonExe) {
  Write-Host ''
  Write-Host '[错误] 未找到 python 或 py。' -ForegroundColor Red
  Write-Host '请安装 Python 3，或用任意静态服务器在本目录启动，然后访问:'
  Write-Host "  $url"
  Write-Host ''
  exit 1
}

Write-Host ''
Write-Host '  ============================================' -ForegroundColor DarkCyan
Write-Host '   C30D 串口调参台' -ForegroundColor Cyan
Write-Host '  ============================================' -ForegroundColor DarkCyan
Write-Host "   目录 : $PSScriptRoot"
Write-Host "   地址 : $url"
Write-Host '   串口 : COM18 / USART1 / 115200 8N1'
Write-Host '   停止 : 按 Ctrl+C'
Write-Host ''
Write-Host '   提示：浏览器必须是桌面版 Chrome / Edge；先关掉占用 COM18 的串口助手。' -ForegroundColor DarkGray
Write-Host ''

if (-not $NoBrowser) {
  try {
    Start-Process $url | Out-Null
  } catch {
    Write-Host '（无法自动打开浏览器，请手动访问上面的地址）' -ForegroundColor Yellow
  }
}

try {
  & $pythonExe -m http.server $Port
} finally {
  Write-Host ''
  Write-Host '本地服务器已停止。' -ForegroundColor DarkGray
}
exit $LASTEXITCODE
