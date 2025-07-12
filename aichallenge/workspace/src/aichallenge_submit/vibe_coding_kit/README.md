# Vibe Coding Kit

カメラ画像を使用した自動運転制御システム

## 概要

Vibe Coding Kitは、AWSIMのカメラ画像を入力として受け取り、画像処理によるレーン検出と障害物検出を行い、自動車の制御指令を生成するシステムです。

## 機能

- **カメラ画像処理**: AWSIMのカメラ画像（1920×1080 Full HD）をリアルタイムで処理
- **レーン検出**: 画像から道路のレーンを検出し、ステアリング角を計算
- **障害物検出**: 画像から障害物を検出し、加速度制御を行う
- **制御指令出力**: 検出結果に基づいて制御指令を生成
- **リアルタイム可視化**: 処理結果を複数の方法で可視化

## 入力

### カメラ画像

- **トピック**: `/sensing/camera/image_raw`
- **型**: `sensor_msgs/msg/Image`
- **解像度**: 1920×1080 (Full HD)
- **エンコーディング**: BGR8
- **フレームID**: camera_link

### カメラ情報

- **トピック**: `/sensing/camera/camera_info`
- **型**: `sensor_msgs/msg/CameraInfo`
- **歪み補正モデル**: plumb_bob

## 出力

### 制御指令

- **トピック**: `/control/command/control_cmd`
- **型**: `autoware_auto_control_msgs/msg/AckermannControlCommand`
- **内容**: ステアリング角と加速度制御指令

### デバッグ画像

- **トピック**: `/vibe_coding_kit/debug_image`
- **型**: `sensor_msgs/msg/Image`
- **内容**: 元画像と処理画像を並べて表示

## 可視化機能

### 1. OpenCVウィンドウ表示

- リアルタイムで処理結果を表示
- リサイズ可能なウィンドウ（初期サイズ: 960×540）
- 元画像とエッジ検出画像を並べて表示

### 2. ROSトピック配信

- `/vibe_coding_kit/debug_image` トピックで可視化画像を配信
- `rqt_image_view` で確認可能

### 3. CSVログ保存

- `/tmp/vibe_coding_kit_log.csv` に制御指令の履歴を保存
- フレーム番号、ステアリング角、加速度を記録

### 4. リアルタイムグラフ表示

- matplotlibによる制御指令値のリアルタイムグラフ
- ステアリング角と加速度の履歴を表示

## 使用方法

### 1. AWSIMでカメラ画像を有効化

AWSIMで「use image」ボタンを押してカメラ画像を有効にします。

### 2. システムの起動

#### Dockerコンテナ内での起動

```bash
# Dockerコンテナに入る
cd ~/aichallenge-2025
./docker_exec.sh

# 環境変数を設定
source /aichallenge/workspace/install/setup.bash

# ビルド
bash build_autoware.sh

# 直接実行
python3 /aichallenge/workspace/src/aichallenge_submit/vibe_coding_kit/scripts/vibe_coding_node.py
```

#### Launchファイルでの起動

```bash
ros2 launch vibe_coding_kit vibe_coding.launch.xml
```

### 3. 可視化の確認

#### OpenCVウィンドウ

- ノード実行時に「Vibe Coding Kit - Camera Processing」ウィンドウが表示されます
- ウィンドウサイズは自由にリサイズ可能です

#### rqt_image_viewでの確認

```bash
# 別ターミナルで実行
rqt_image_view
```

- トピック一覧から `/vibe_coding_kit/debug_image` を選択

#### CSVログの確認

```bash
# ログファイルの確認
cat /tmp/vibe_coding_kit_log.csv

# リアルタイムで監視
tail -f /tmp/vibe_coding_kit_log.csv
```

### 4. 動作確認

- カメラ画像が正常に受信されていることをログで確認
- 制御指令が生成されていることをログで確認
- 可視化ウィンドウで処理結果を確認

## システム構成

### ファイル構成

```
vibe_coding_kit/
├── scripts/
│   ├── vibe_coding_node.py      # メインノード（可視化機能付き）
│   └── camera_controller.py     # カメラ画像処理コントローラー
├── launch/
│   └── vibe_coding.launch.xml   # 起動ファイル
├── package.xml                  # パッケージ設定
├── CMakeLists.txt              # ビルド設定
├── requirements.txt             # Python依存関係
└── README.md                   # このファイル
```

### 主要クラス

#### VibeCodingNode

- カメラ画像の受信
- 制御指令の生成とパブリッシュ
- リアルタイム可視化（OpenCV、matplotlib）
- CSVログ保存
- ROS2ノードの管理

#### CameraController

- 画像の前処理（グレースケール変換、ノイズ除去、エッジ検出）
- レーン検出（Hough変換による直線検出）
- 障害物検出（輪郭検出）
- 制御指令の計算
- 可視化画像の生成

## 制御パラメータ

### ステアリング制御

- **最大ステアリング角**: 0.5 rad
- **レーン検出閾値**: 50
- **最小線長**: 50 pixels

### 加速度制御

- **最大加速度**: 2.0 m/s²
- **障害物検出面積閾値**: 1000 pixels²

### 可視化設定

- **OpenCVウィンドウ初期サイズ**: 960×540 pixels
- **CSVログ保存先**: `/tmp/vibe_coding_kit_log.csv`
- **グラフ更新頻度**: フレーム毎

## 開発環境

- **ROS2**: Humble
- **Python**: 3.8+
- **OpenCV**: 4.5+
- **NumPy**: 1.20+
- **matplotlib**: 3.3+
- **cv_bridge**: ROS2標準

## トラブルシューティング

### カメラ画像が受信されない

1. AWSIMでカメラ画像が有効になっているか確認
2. トピック名が正しいか確認: `/sensing/camera/image_raw`
3. QoS設定を確認（BEST_EFFORT）

### 制御指令が生成されない

1. カメラ画像のエンコーディングが正しいか確認（BGR8）
2. 画像処理でエラーが発生していないかログを確認
3. 依存パッケージが正しくインストールされているか確認

### 可視化ウィンドウが表示されない

1. X11転送が有効になっているか確認（dockerの場合）
2. DISPLAY環境変数が設定されているか確認
3. GUI環境が利用可能か確認

### パフォーマンスの問題

1. 画像処理のパラメータを調整
2. 処理頻度を下げる（フレームスキップ）
3. 可視化機能を無効化（コメントアウト）

## カスタマイズ

### レーン検出の調整

`camera_controller.py`の `detect_lanes`メソッドで以下のパラメータを調整できます：

- `threshold`: 直線検出の閾値
- `minLineLength`: 最小線長
- `maxLineGap`: 最大線間隔

### 障害物検出の調整

`detect_obstacles`メソッドで以下のパラメータを調整できます：

- 輪郭面積の閾値
- 減速・加速の強度

### 可視化の調整

- OpenCVウィンドウサイズ: `cv2.resizeWindow`の引数を変更
- グラフ表示: matplotlibの設定を調整
- CSVログ: 保存先やフォーマットを変更

## ライセンス

The MIT License (MIT)

---

## おすすめの軽量セマンティックセグメンテーションモデル

### 1. **Fast-SCNN**
- モバイル向けに設計され、推論が非常に高速
- PyTorch実装が豊富
- Cityscapesなどの道路シーンに強い

### 2. **ENet**
- 低リソース環境向けの超軽量モデル
- PyTorchやTensorFlow実装あり

### 3. **YOLOv8-seg**
- YOLOv8のセグメンテーション版
- 物体検出＋マスク出力（レーンや壁を物体として検出できる場合に有効）
- pipで簡単に導入可能

---

## 例：YOLOv8-seg（ultralytics）を使う場合

### 1. インストール
```bash
pip install ultralytics
```

### 2. サンプルコード（camera_controller.pyに組み込むイメージ）

```python
from ultralytics import YOLO
import numpy as np
import cv2

class CameraController:
    def __init__(self):
        # 軽量モデルをロード（例: yolov8n-seg.pt）
        self.model = YOLO('yolov8n-seg.pt')  # 事前にダウンロードしておく
        self.lane_class_id = 0  # モデルに応じて設定
        self.wall_class_id = 1  # モデルに応じて設定

    def process_image(self, image: np.ndarray):
        # YOLOv8-segで推論
        results = self.model(image)
        masks = results[0].masks.data.cpu().numpy()  # (N, H, W)
        classes = results[0].boxes.cls.cpu().numpy() # (N,)

        lane_mask = np.zeros(image.shape[:2], dtype=np.uint8)
        wall_mask = np.zeros(image.shape[:2], dtype=np.uint8)
        for i, cls in enumerate(classes):
            if cls == self.lane_class_id:
                lane_mask = np.logical_or(lane_mask, masks[i])
            if cls == self.wall_class_id:
                wall_mask = np.logical_or(wall_mask, masks[i])

        # レーンの重心でステアリング
        M = cv2.moments(lane_mask.astype(np.uint8))
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            error = (cx - (image.shape[1] // 2)) / (image.shape[1] // 2)
            steering_angle = error * 0.3  # ゲインは調整
        else:
            steering_angle = 0.0

        # 壁の面積で加減速
        if np.sum(wall_mask) > 1000:
            acceleration = -1.0
        else:
            acceleration = 1.0

        return steering_angle, acceleration
```

---

## 注意点
- **モデルファイル（例: yolov8n-seg.pt）は事前にダウンロード**しておく必要があります。
- **クラスID（lane_class_id, wall_class_id）はモデルによって異なる**ので、学習済みモデルの仕様を確認してください。
- **推論速度**はCPUでも十分速いですが、GPUがあればさらに快適です。

---

## Fast-SCNNやENetを使いたい場合
- PyTorch HubやGitHubから学習済みモデルをダウンロードし、同様に`process_image`で推論・マスク抽出・制御値計算を行います。
- 使いたいモデルが決まっていれば、そのモデル用のサンプルコードもご用意できます。

---

## まとめ

- **YOLOv8-seg**は導入が簡単で、物体検出＋セグメンテーションが一度にできるのでおすすめです。
- **Fast-SCNN/ENet**は道路シーンに特化した学習済みモデルがあれば、さらに高精度が期待できます。

---

### どのモデルを使いたいか（例: YOLOv8-seg, Fast-SCNN, ENetなど）を教えていただければ、  

---

## 選択肢

### 1. torchvisionの**DeepLabV3**や**FCN**（学習済み重みが自動DL）
- PyTorchの`torchvision`には、`pretrained=True`で自動的に学習済み重みがダウンロードされるモデルがあります。
- 例: `torchvision.models.segmentation.deeplabv3_resnet50(pretrained=True)`
- CityscapesやCOCOなどで学習済み（道路・壁・車・人などのクラスあり）

### 2. HuggingFace Transformersの一部モデル
- `transformers`ライブラリの一部セグメンテーションモデルも、初回実行時に自動で重みをダウンロードします。

---

## 一番手軽な例：**torchvisionのDeepLabV3**

### インストール
```bash
pip install torch torchvision
```

### サンプルコード（camera_controller.pyに組み込み）

```python
import torch
import torchvision
import cv2
import numpy as np
from torchvision import transforms

class CameraController:
    def __init__(self):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        # 学習済み重みは初回実行時に自動DL
        self.model = torchvision.models.segmentation.deeplabv3_resnet50(pretrained=True).to(self.device)
        self.model.eval()
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((520, 520)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        # COCOのクラスID例
        self.road_class_id = 0  # background
        self.wall_class_id = 12 # wall（COCOのID、モデルによって異なる）

    def process_image(self, image: np.ndarray):
        input_tensor = self.transform(image).unsqueeze(0).to(self.device)
        with torch.no_grad():
            output = self.model(input_tensor)['out'][0]
            pred = output.argmax(0).cpu().numpy()  # (H, W)

        # レーン（road）・壁マスク
        lane_mask = (pred == self.road_class_id)
        wall_mask = (pred == self.wall_class_id)

        # レーンの重心でステアリング
        M = cv2.moments(lane_mask.astype(np.uint8))
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            error = (cx - (image.shape[1] // 2)) / (image.shape[1] // 2)
            steering_angle = error * 0.3
        else:
            steering_angle = 0.0

        # 壁の面積で加減速
        if np.sum(wall_mask) > 1000:
            acceleration = -1.0
        else:
            acceleration = 1.0

        return steering_angle, acceleration

    def visualize_processing(self, image: np.ndarray, pred: np.ndarray) -> np.ndarray:
        color_mask = np.zeros_like(image)
        color_mask[pred == self.road_class_id] = [0, 255, 0]  # 緑:レーン
        color_mask[pred == self.wall_class_id] = [0, 0, 255]  # 赤:壁
        vis = cv2.addWeighted(image, 0.7, color_mask, 0.3, 0)
        return vis
```

---

## ポイント
- **重みファイルの手動ダウンロード不要**（初回実行時に自動で取得）
- COCOやVOCのクラスIDは[公式リスト](https://github.com/pytorch/vision/blob/main/torchvision/datasets/_coco.py)を参照
- クラスIDはモデルによって異なるので、必要に応じて`print(np.unique(pred))`で確認

---

## まとめ

- **torchvisionのDeepLabV3/FCN**なら、重みファイルの手動ダウンロード不要で、すぐに使えます！
- 速度はENetやYOLOv8-segよりやや遅いですが、CPUでも十分実用的です。

---

**この方法でcamera_controller.pyを修正しますか？  
（もしくは他のモデルが良ければご希望を教えてください！）**

---

## 原因
PyTorch（torch）がインストールされていないためです。

---

## 対応方法

**dockerコンテナ内で以下のコマンドを実行してください：**

```bash
pip install torch torchvision
```

- これでPyTorchとtorchvisionがインストールされ、DeepLabV3が動作するようになります。

---

### 補足
- CPUのみで動かす場合は、上記コマンドでOKです。
- GPU（CUDA）対応が必要な場合は、[公式インストールガイド](https://pytorch.org/get-started/locally/)を参照してください。

---

インストール後、再度ノードを起動すれば動作するはずです。  
もし他にもエラーが出た場合は、その内容を教えてください！

---

## UltraFast Lane Detection 導入・組み込み手順

### 1. リポジトリのクローンと依存パッケージのインストール

```bash
git clone https://github.com/cfzd/Ultra-Fast-Lane-Detection.git
cd Ultra-Fast-Lane-Detection
pip install -r requirements.txt
```

### 2. 学習済みモデルのダウンロード

- [Releasesページ](https://github.com/cfzd/Ultra-Fast-Lane-Detection/releases)から
  - 例: `culane_18.pth` または `tusimple_18.pth`
- `Ultra-Fast-Lane-Detection` ディレクトリ内に保存

### 3. サンプル画像で推論テスト

```bash
python test.py --test_model culane_18.pth --dataset CULane --image_folder ./data/test_images --save_dir ./output
```

### 4. **camera_controller.pyへの組み込みイメージ**

- `Ultra-Fast-Lane-Detection`の`model.py`や`utils.py`をimportし、  
  `CameraController`クラスでモデルをロード＆推論
- 入力画像（BGR→RGB変換）をUltraFastの`predict`関数に渡し、  
  出力マスクからレーン位置を抽出
- レーンの重心や曲率からステアリング角を計算

---

## camera_controller.py 組み込み例（イメージ）

```python
import sys
sys.path.append('/path/to/Ultra-Fast-Lane-Detection')  # UltraFastのパスを追加

import torch
import cv2
import numpy as np
from model.model import parsingNet
from utils.common import merge_config
from utils.dist_utils import dist_print

class CameraController:
    def __init__(self):
        # UltraFastの設定
        import argparse
        cfg, _, _ = merge_config()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = parsingNet(pretrained=False, backbone='18', cls_dim=(101, 56, 4), use_aux=False).to(self.device)
        self.model.load_state_dict(torch.load('/path/to/culane_18.pth', map_location=self.device), strict=False)
        self.model.eval()
        self.img_w, self.img_h = 1640, 590  # UltraFastのデフォルトサイズ

    def process_image(self, image: np.ndarray):
        # UltraFast用にリサイズ・正規化
        img = cv2.resize(image, (self.img_w, self.img_h))
        img = img / 255.0
        img = img.transpose(2, 0, 1)
        img = torch.from_numpy(img).float().unsqueeze(0).to(self.device)

        with torch.no_grad():
            out = self.model(img)
            # out[1]がレーンの位置情報
            lane_points = out[1].cpu().numpy()  # (batch, 4, 56)

        # lane_pointsからレーンの重心や中心線を計算し、ステアリング角を決定
        # ここは実際の出力に応じて調整
        # 例: lane_points[0, 1, :]（中央レーン）を使う
        center_lane = lane_points[0, 1, :]
        valid_points = center_lane[center_lane > 0]
        if len(valid_points) > 0:
            cx = np.mean(valid_points)
            error = (cx - (self.img_w // 2)) / (self.img_w // 2)
            steering_angle = error * 0.3
        else:
            steering_angle = 0.0

        acceleration = 1.0  # 障害物検出はUltraFast単体ではできないので、必要なら別途追加

        return steering_angle, acceleration

    def visualize_processing(self, image: np.ndarray, processed_image: np.ndarray = None) -> np.ndarray:
        # UltraFastの出力を元画像に重ねて可視化（必要に応じて実装）
        return image
```

---

## 注意点
- UltraFastは**レーン検出専用**なので、障害物検出（壁や車）は別途実装が必要です。
- モデルのパスや画像サイズは環境に合わせて調整してください。
- ROSノードや既存のcamera_controller.pyに組み込む際は、UltraFastのリポジトリをパスに追加する必要があります。

---

## 次のステップ

1. UltraFast Lane Detectionのリポジトリをクローン＆セットアップ
2. 学習済みモデル（`culane_18.pth`など）をダウンロード
3. サンプル画像で動作確認
4. camera_controller.pyに組み込み

---

**「UltraFastのセットアップができた」「camera_controller.pyに組み込むサンプルが欲しい」など、  
ご希望や進捗を教えていただければ、さらに具体的なサポートをします！**
