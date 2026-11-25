$ErrorActionPreference = 'Continue'
$sp = New-Object System.IO.Ports.SerialPort('COM3', 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$sp.ReadTimeout = 300
$buf = New-Object byte[] 16384

function Grab([int]$ms) {
    $o = New-Object System.IO.MemoryStream
    $end = (Get-Date).AddMilliseconds($ms)
    while ((Get-Date) -lt $end) {
        try {
            $n = $sp.Read($buf, 0, $buf.Length)
            if ($n -gt 0) { $o.Write($buf, 0, $n) }
        } catch { }
        Start-Sleep -Milliseconds 10
    }
    return [System.Text.Encoding]::GetEncoding(28591).GetString($o.ToArray())
}

function Stat([string]$text) {
    $ps = @()
    foreach ($f in ($text -split "`r`n")) {
        if ($f -match '^A,') {
            $c = $f -split ','
            if ($c.Count -eq 4) { $ps += [int]$c[2] / 100 }
        }
    }
    if ($ps.Count -lt 5) { return $null }
    $sorted = $ps | Sort-Object
    $avg = ($ps | Measure-Object -Average).Average
    return @{
        avg = [math]::Round($avg, 2)
        pk  = [math]::Round(($ps | Measure-Object -Maximum).Maximum - ($ps | Measure-Object -Minimum).Minimum, 2)
        p5  = [math]::Round($sorted[[int]($sorted.Count * 0.05)], 2)
        p95 = [math]::Round($sorted[[int]($sorted.Count * 0.95)], 2)
    }
}

try {
    $sp.Open()
    $sp.DiscardInBuffer()
    "=== 每秒统计（观察是否收敛）==="
    for ($i = 1; $i -le 8; $i++) {
        $sp.DiscardInBuffer()
        $text = Grab 900
        $s = Stat $text
        $sv = "?"
        $m = [regex]::Match($text, 'servo=(\d+)/(\d+)')
        if ($m.Success) { $sv = $m.Groups[2].Value }
        if ($s) {
            "  {0}s: 均值 {1,7}°  峰峰 {2,6}°  5~95% {3} ~ {4}  舵机 {5}" -f $i, $s.avg, $s.pk, $s.p5, $s.p95, $sv
        } else {
            "  {0}s: 数据不足  舵机 {1}" -f $i, $sv
        }
    }
} catch {
    "COM3 错误: " + $_.Exception.Message
} finally {
    if ($sp.IsOpen) { $sp.Close() }
    $sp.Dispose()
}
