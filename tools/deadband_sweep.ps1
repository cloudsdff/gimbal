$ErrorActionPreference = 'Continue'
$sp = New-Object System.IO.Ports.SerialPort('COM3', 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$sp.ReadTimeout = 300
$buf = New-Object byte[] 16384

function Grab([int]$ms) {
    $o = New-Object System.IO.MemoryStream
    $end = (Get-Date).AddMilliseconds($ms)
    while ((Get-Date) -lt $end) {
        try { $n = $sp.Read($buf, 0, $buf.Length); if ($n -gt 0) { $o.Write($buf, 0, $n) } } catch { }
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
    $avg = ($ps | Measure-Object -Average).Average
    return @{
        avg = [math]::Round($avg, 2)
        pk  = [math]::Round(($ps | Measure-Object -Maximum).Maximum - ($ps | Measure-Object -Minimum).Minimum, 2)
        n   = $ps.Count
    }
}

try {
    $sp.Open()
    $sp.DiscardInBuffer()
    if ($args.Count -gt 0 -and $args[0] -eq "prep") {
        # 完整标定 + 启动
        $sp.Write("D`r`n"); Start-Sleep -Milliseconds 400
        $sp.Write("P,1500,1900`r`n"); Start-Sleep -Milliseconds 3000
        $sp.Write("Z`r`n"); Start-Sleep -Milliseconds 1500
        $sp.Write("G,500,80,80`r`n"); Start-Sleep -Milliseconds 300
        $sp.Write("T,0,0`r`n"); Start-Sleep -Milliseconds 300
        $sp.Write("X`r`n")
        "已启动控制"
        Start-Sleep -Seconds 8
    }

    "=== 死区扫描（每组：设定死区 → 等 3 秒 → 测 2 秒）==="
    foreach ($db in @(5, 10, 15, 20, 25)) {
        $sp.Write("H,$db`r`n")
        Start-Sleep -Milliseconds 300
        Start-Sleep -Seconds 3
        $sp.DiscardInBuffer()
        $s = Stat (Grab 2000)
        if ($s) {
            $mark = "?"
            if ($s.pk -lt 1.0) { $mark = "很稳" }
            elseif ($s.pk -lt 2.0) { $mark = "稳" }
            elseif ($s.pk -lt 5.0) { $mark = "微振" }
            else { $mark = "振荡" }
            "  死区 {0:N1}°: 均值 {1,7}°  峰峰 {2,6}°  ({3} 帧) {4}" -f ($db / 10.0), $s.avg, $s.pk, $s.n, $mark
        } else {
            "  死区 {0:N1}°: 数据不足" -f ($db / 10.0)
        }
    }
} catch {
    "COM3 错误: " + $_.Exception.Message
} finally {
    if ($sp.IsOpen) { $sp.Close() }
    $sp.Dispose()
}
