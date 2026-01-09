# D3QN Sosyal Robot Navigasyonu (ROS2 + Gazebo)

Bu depo, TurtleBot3 üzerinde sosyal farkındalıklı navigasyon için D3QN tabanlı bir eğitim/test altyapısı içerir.

## Özellikler
- Dueling + Double DQN (D3QN)
- Deneyim belleği (Replay Buffer)
- Davranış ağacı (mod seçimi + aksiyon kısıtlama)
- ROS2 entegrasyonu: `/scan`, `/odom`, `/cmd_vel`, TF
- Senaryo tabanlı konfigürasyon (hastane / okul / avm)
- Metrik hesaplama ve raporlama: başarı oranı, çarpışma, yakın mesafe ihlali, hedefe ulaşma süresi, yol verimliliği, sistem kararlılığı
- Çıktılar: TensorBoard log, JSON/CSV metrik raporu, model kontrol noktaları

## Kurulum (özet)
```bash
sudo apt-get update
sudo apt-get install -y python3-pip ros-humble-gazebo-ros-pkgs ros-humble-turtlebot3-gazebo
pip3 install --user -r requirements.txt
```

## Derleme
```bash
mkdir -p ~/ros2_ws/src
cp -r d3qn_sosyal_navigasyon ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Çalıştırma
### 1) Simülasyon + eğitim
```bash
ros2 launch d3qn_sosyal_navigasyon hastane_egitim.launch.py
```

### 2) Eğitim (node ile) - opsiyonel CLI
```bash
ros2 run d3qn_sosyal_navigasyon egit --senaryo <senaryo.yaml> --parametreler <d3qn_parametreler.yaml>
```

### 3) Test
```bash
ros2 launch d3qn_sosyal_navigasyon test_navigasyon.launch.py
```

Detaylı anlatım: `dokumanlar/kurulum.md`, `dokumanlar/mimari.md`, `dokumanlar/egitim_kilavuzu.md`
