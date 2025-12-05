# PowerShell script to process multiple datasets sequentially
# Usage: 
#   Dry-run:  .\run_all_datasets.ps1 -DryRun
#   Real run: .\run_all_datasets.ps1

param(
    [switch]$DryRun
)

$datasets = @(
    "D:\data\ncdb-cls\ncdb-cls-640x384\synchronized_data_pangyo_optimized",
    "D:\data\ncdb-cls\ncdb-cls-640x384\2025-07-11_15-00-27_410410_A\synced_data",
    "D:\data\ncdb-cls\ncdb-cls-640x384\2025-07-11_15-39-30_243127_B\synced_data"
)

$resolution = "640x384"
$startTime = Get-Date

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  Batch Processing: $($datasets.Count) datasets" -ForegroundColor Cyan
Write-Host "  Resolution: $resolution" -ForegroundColor Cyan
Write-Host "  Mode: $(if ($DryRun) { 'DRY-RUN (test only)' } else { 'REAL RUN' })" -ForegroundColor $(if ($DryRun) { 'Yellow' } else { 'Green' })
Write-Host "  Start time: $startTime" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

$index = 0
foreach ($dataset in $datasets) {
    $index++
    Write-Host ""
    Write-Host "[$index/$($datasets.Count)] Processing: $dataset" -ForegroundColor Yellow
    Write-Host "----------------------------------------"
    
    # Check if folder exists
    if (-not (Test-Path $dataset)) {
        Write-Host "[ERROR] Folder not found: $dataset" -ForegroundColor Red
        continue
    }
    
    # Check if pcd folder exists
    $pcdFolder = Join-Path $dataset "pcd"
    if (-not (Test-Path $pcdFolder)) {
        Write-Host "[ERROR] PCD folder not found: $pcdFolder" -ForegroundColor Red
        continue
    }
    
    $pcdCount = (Get-ChildItem -Path $pcdFolder -Filter "*.pcd" | Measure-Object).Count
    Write-Host "[INFO] Found $pcdCount PCD files" -ForegroundColor Cyan
    
    if ($DryRun) {
        Write-Host "[DRY-RUN] Would execute:" -ForegroundColor Magenta
        Write-Host "  python integrated_pcd_depth_pipeline_v3.py --parent_folder `"$dataset`" --resolution $resolution" -ForegroundColor Gray
        
        # Quick validation: try to run with --help or just import check
        Write-Host "[DRY-RUN] Validating script syntax..." -ForegroundColor Magenta
        $result = python -c "import integrated_pcd_depth_pipeline_v3; print('OK')" 2>&1
        if ($result -match "OK") {
            Write-Host "[DRY-RUN] Script import: OK" -ForegroundColor Green
        } else {
            Write-Host "[DRY-RUN] Script import: FAILED" -ForegroundColor Red
            Write-Host $result -ForegroundColor Red
        }
    } else {
        $datasetStart = Get-Date
        Write-Host "[START] $(Get-Date -Format 'HH:mm:ss')" -ForegroundColor Green
        
        python integrated_pcd_depth_pipeline_v3.py --parent_folder "$dataset" --resolution $resolution
        
        $datasetEnd = Get-Date
        $elapsed = $datasetEnd - $datasetStart
        Write-Host "[DONE] Elapsed: $($elapsed.ToString('hh\:mm\:ss'))" -ForegroundColor Green
    }
}

$endTime = Get-Date
$totalElapsed = $endTime - $startTime

Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  Batch Processing Complete!" -ForegroundColor Cyan
Write-Host "  Total time: $($totalElapsed.ToString('hh\:mm\:ss'))" -ForegroundColor Cyan
Write-Host "  End time: $endTime" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
