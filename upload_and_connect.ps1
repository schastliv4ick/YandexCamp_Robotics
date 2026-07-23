<#
.SYNOPSIS
    Uploads the build, config and remote script to the VM, then connects to it.

.EXAMPLE
    .\upload_and_connect.ps1 -VmIp 158.160.207.86 -LocalBuildZip .\Build_Linux.zip
#>

param(
    [Parameter(Mandatory=$true)][string]$VmIp,
    [string]$KeyPath        = "$env:USERPROFILE\.ssh\yc_key",  # -i <key>: your private key, never share it
    [string]$Login          = "student",
    [string]$LocalBuildZip  = ".\Build_Linux.zip",
    [string]$LocalConfig    = ".\config.yaml",
    [string]$LocalRemoteScript = ".\train_remote.sh"
)

$Target = "$Login@$VmIp"

Write-Host "Uploading build..."
scp -i $KeyPath $LocalBuildZip      "${Target}:~/Build_Linux.zip"

Write-Host "Uploading config..."
scp -i $KeyPath $LocalConfig        "${Target}:~/config.yaml"

Write-Host "Uploading remote script..."
scp -i $KeyPath $LocalRemoteScript  "${Target}:~/train_remote.sh"

Write-Host "Connecting to $Target ..."
ssh -i $KeyPath $Target   # -i: private key for ssh, drops you into the VM shell
