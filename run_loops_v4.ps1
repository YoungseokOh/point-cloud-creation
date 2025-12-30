# run_loops_v4.ps1
$loops = @(
  "D:\data\ncdb-cls\ncdb-cls-indoor\loop_3\synchronized_data_indoor_50ms_second",
  "D:\data\ncdb-cls\ncdb-cls-indoor\loop_4\synchronized_data_indoor_100ms_second",
  "D:\data\ncdb-cls\ncdb-cls-indoor\loop_5\synchronized_data_indoor_100ms_second"
)

$dryRun = $false   # dry-run: $true → 동작만 예고, 실제 처리 안 함. 실제 실행 시 $false 로 변경.

foreach ($p in $loops) {
  Write-Host "`n==== Processing $p ====" -ForegroundColor Cyan
  if ($dryRun) {
    python integrated_pcd_depth_pipeline_v4.py --parent_folder "$p" --dry-run
  } else {
    python integrated_pcd_depth_pipeline_v4.py --parent_folder "$p"
  }
}