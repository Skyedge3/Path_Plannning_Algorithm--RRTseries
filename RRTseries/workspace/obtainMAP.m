% 步骤 1: 读取 PNG 文件
imageFile = 'map2.png'; % 替换为你的PNG文件路径
img = imread(imageFile);

% 如果图像是 RGB 图像，将其转换为灰度图像
if size(img, 3) == 3
    img = rgb2gray(img);
end

% 步骤 2: 创建 occupancyMap 对象
% 将灰度图像转换为二值图像，黑色为障碍物，白色为可通行区域
binaryImg = img < 128; % 根据需要调整阈值

% 创建 occupancyMap 对象，假设图像分辨率是1单位/像素
map = occupancyMap(binaryImg, 1);

% 步骤 3: 保存到 .mat 文件
save('map2.mat', 'map');
