# Stackchan_Bluetooth_A2DP_simple


# 概要
機能をBluetoothスピーカーとリップシンクのみに絞ったｽﾀｯｸﾁｬﾝのアプリです。

# 対象機種
M5Stack Basic/Gray/Fire, M5Go, M5Stack Core2/AWSで動作確認しています。

# ビルド
Platformio用です。

ArduinoIDEで利用する場合は、srcフォルダとmain.cppの名前を揃えて変更してください。

## 例
src -> Stackchan_Bluetooth_A2DP_simple 

main.cpp -> Stackchan_Bluetooth_A2DP_simple.ino

# 必要なライブラリ
詳しくはplatformio.iniを参照してください。
- [m5stack-avatar](https://github.com/meganetaaan/m5stack-avatar)
- [M5Unified](https://github.com/m5stack/M5Unified)
- [ServoEasing](https://github.com/ArminJo/ServoEasing)
- [ESP32Servo](https://github.com/madhephaestus/ESP32Servo)
- [ESP32-A2DP](https://github.com/pschatzmann/ESP32-A2DP)

## 2022/4時点の注意事項
 m5stack-avatarライブラリはM5Unified対応が必要です。公開されているライブラリではなく、[Github](https://github.com/meganetaaan/m5stack-avatar)から最新版を取得してください。
 ※ おそらく0.7.5以降だと思います。
# 引用元
 [M5Unifiedライブラリ](https://github.com/m5stack/M5Unified)のexamples/Advanced/Bluetooth_with_ESP32A2DPをベースに改変しました。

# Author
[Takao Akaki](https://github.com/mongonta0716)

# License
[MIT](https://github.com/mongonta0716/stackchan_test/blob/main/LICENSE)