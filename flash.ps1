Write-Host "Compiling C++ code..." -ForegroundColor Cyan
Set-Location build
cmake ..

# Note: 'cmake --build .' is the cross-platform way to run 'make' on Windows
cmake --build .
Set-Location ..

# 1. Find the Pico's COM port (Looks for USB serial devices)
$picoPort = Get-CimInstance Win32_PnPEntity | Where-Object { $_.Caption -match "\(COM\d+\)" -and $_.Caption -match "USB" }

if ($picoPort) {
    # Extract just the "COMx" part
    $picoPort.Caption -match "(COM\d+)" > $null
    $comPort = $Matches[1]

    Write-Host "Pico found on $comPort. Triggering 1200-baud hardware reset..." -ForegroundColor Cyan

    # Windows equivalent of 'stty 1200' is the 'mode' command
    cmd.exe /c "mode $comPort baud=1200 data=8 parity=n stop=1" | Out-Null

    Start-Sleep -Seconds 2
} else {
    Write-Host "Warning: Pico USB serial port not found. Is it already in bootloader mode?" -ForegroundColor Yellow
}

Write-Host "Waiting for RPI-RP2 drive to mount..." -ForegroundColor Cyan
$picoDrive = $null

# 2. Wait and look for the drive by its Volume Label
for ($i = 1; $i -le 10; $i++) {
    $volume = Get-Volume -ErrorAction SilentlyContinue | Where-Object { $_.FileSystemLabel -eq "RPI-RP2" }
    if ($volume) {
        $picoDrive = $volume.DriveLetter + ":\"
        break
    }
    Start-Sleep -Seconds 1
}

# 3. Upload the firmware
if ($picoDrive) {
    Write-Host "Drive found at $picoDrive! Uploading firmware..." -ForegroundColor Green

    # Copy the compiled file to the Pico
    Copy-Item -Path "build\LSM6DSV80X.uf2" -Destination $picoDrive

    Write-Host "Upload Complete! Pico is rebooting to run your new code." -ForegroundColor Green
} else {
    Write-Host "Error: RPI-RP2 drive did not mount. You may need to press the button manually this one last time." -ForegroundColor Red
    exit 1
}