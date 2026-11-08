# 旋回機構・駆動系

## 無限回転Yaw 
機体上側と下側が回転する際に使用される機構です。<br>
2026年度はモータを変えたためベルト駆動となっており、2025年度との比較を画像1、画像2にそれぞれ示します。<br>
回転しても配線がねじれないようにするスリップリングが搭載されています。

![alt text](image-7.png)
<div style="text-align: center">画像1 ver.2025</div>

![alt text](image-8.png)
<div style="text-align: center">画像2 ver.2026</div>

## 砲台Yaw 
RoboStride 05を用いて発射機構を左右に動かす機構です。<br>
画像3に機構を示します。<br>
砲台Pitchと合わせることで発射機構を自由に動かします。<br>

![alt text](image-10.png)
<div style="text-align: center">画像3</div>

## 砲台Pitch
サーボモータを用いて発射機構を縦に動かす機構です。<br>
画像4に機構を示します。<br>
砲台Yaw と合わせることで発射機構を自由に動かします。<br>

![alt text](image-9.png)
<div style="text-align: center">画像4</div>

## 関連ページ

- [砲塔・装填・発射機構](turret.md)
- [回路構成：CANバスとモータID](../circuit/buses.md) — バス構成とID割り当て
- [core_body_controller パッケージ](../packages/core_body_controller/index.md)
