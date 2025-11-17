C:\Users\seok436\Documents\VSCode\Projects\point-cloud-creation\point-cloud-creation\output\u2d_test.raw.npy
지금 이 경로에 Depth 결과가 있어 이거는 Depth Anything으로 돌린 결과야.
이거를 지금 @integrated_pipeline.py 여기에서 Depth_map 부분에 경로로 넣어서 바로 Distorted Depth를 만드는걸 하고싶어.
즉, Depth Anything을 추론할 필요 없이 경로로 바로 받아서 해볼 수 있을 것 같아. 그래서 기존의 함수를 제거하고, Depth를 받아서 수행하는 코드로 바꿔줘.