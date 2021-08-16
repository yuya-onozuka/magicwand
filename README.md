# magicwand packages
## 依存パッケージ
- [ros-keyboard](https://github.com/lrse/ros-keyboard)
- om_modbus_master（オリエンタルモータのROSパッケージ）

## 実行
### 1. モータを接続
* 以下のコマンドを実行し、ttyUSB0などが表示されるか確認
```
$ cd /dev
$ ls | grep ttyU
```
### 2. モータのアクセス権限を付与
```
$ sudo chmod 666 /dev/ttyUSB*
```
### 3. 実行パラメータ設定
* magicwand/launch/master.launchで設定
```yaml
<arg name="nodelet" default="true"/>       # nodeletを起動するか                 
<arg name="debug" default="false"/>        # debugモードで起動するか
<arg name="online" default="true"/>        # モータとの接続ノードを起動するか
<arg name="use_keyboard" default="true"/>  # keyboardでコマンドを送信するノードを起動するか
```
### 4. 実行
```
roslaunch magicwand master.launch
```
