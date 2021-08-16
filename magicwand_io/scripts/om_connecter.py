#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import time
import math

from om_modbus_master.msg import om_query
from om_modbus_master.msg import om_response
from om_modbus_master.msg import om_state
from magicwand_msgs.msg import Motor

class motorConnecter:

    query_msg = om_query()  # ノードで定義されたメッセージを使用
    motor_state = Motor()

    state_driver = 0   # 通信可能フラグ変数(0:通信可能,1:通信中)
    state_mes = 0      # メッセージ(0:メッセージなし,1:メッセージ到達,2:メッセージエラー)
    state_error = 0    # エラー(0:エラーなし,1:無応答,2:例外応答)

    wheel_radius = 0.075 # [m]
    reduction_ratio = 100.0

    is_enable_commandCallback = True
    
    def __init__(self):
        rospy.init_node('motor_connecter', anonymous=True)
        self.query_pub = rospy.Publisher("om_query1", om_query, queue_size=1)  # OMにノードに送信するまでの定義
        self.state_pub = rospy.Publisher('motor_state', Motor, queue_size=100)
        rospy.Subscriber('motor_command', Motor, self.commandCallback)
        rospy.Subscriber("om_state1", om_state, self.stateCallback)  # レスポンスのコールバック定義
        rospy.Subscriber("om_response1", om_response, self.resCallback)  # レスポンスのコールバック定義

        self.init_query()

        time.sleep(1.0)
    
    def init_query(self):
        """初期化関数

        処理内容1:運転入力方式を3ワイヤ方式に変更
        処理内容2:運転データNo.2の回転速度の初期化(0[r/min])
        処理内容3:Configrationの実行

        """
        # 運転入力方式の変更(3ワイヤ)
        self.query_msg.slave_id = 0x00     # 号機選択(Hex): 0(ブロードキャスト)
        self.query_msg.func_code = 1       # ファンクションコード選択: 1(Write)
        self.query_msg.write_addr = 4160   # 先頭アドレス選択(Dec): 運転入力方式パラメータ
        self.query_msg.write_num = 1       # 書き込みデータサイズ: 1(32bit)
        self.query_msg.data[0] = 1         # 書き込みデータ: 0(2ワイヤ),1(3ワイヤ)
        self.query_pub.publish(self.query_msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()                  # 処理待ち

        # 運転データ 回展速度No.2を0[r/min]に初期化
        self.query_msg.slave_id = 0x00     # 号機選択(Hex): 0(ブロードキャスト)
        self.query_msg.func_code = 1       # ファンクションコード選択: 1(Write)
        self.query_msg.write_addr = 1156   # 先頭アドレス選択(Dec): データNo.2 回転速度
        self.query_msg.write_num = 1       # 書き込みデータサイズ: 1 (32bit)
        self.query_msg.data[0] = 0         # 書き込みデータ: 0[r/min]
        self.query_pub.publish(self.query_msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()                  # 処理待ち

        # Configrationの実行
        self.query_msg.slave_id = 0x00   # 号機選択(Hex): 0(ブロードキャスト)
        self.query_msg.func_code = 1     # ファンクションコード選択: 1(Write)
        self.query_msg.write_addr = 396  # 先頭アドレス選択(Dec): Configration実行コマンド
        self.query_msg.write_num = 1     # 書き込みデータサイズ: 1 (32bit)
        self.query_msg.data[0] = 1       # 書き込みデータ: 1(実行)
        self.query_pub.publish(self.query_msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()                # 処理待ち

        rospy.loginfo("Initialize motor")

    def resCallback(self, res):
        """レスポンスコールバック関数

        購読したレスポンスデータをグローバル変数に反映する

        """
        if (res.slave_id == 1 and res.func_code == 3):
            # 号機番号が1かつ読み込みのときに値を更新
            self.motor_state.left_vel = self.rpm2ms(res.data[0])
            self.motor_state.header.stamp = rospy.get_rostime()
        elif (res.slave_id == 2 and res.func_code == 3):
            # 号機番号が2かつ読み込みのときに値を更新
            self.motor_state.right_vel = self.rpm2ms(res.data[0])
            self.motor_state.header.stamp = rospy.get_rostime()

    def stateCallback(self, res):
        """ステータスコールバック関数

        購読したステータスデータをグローバル変数に反映する

        """
        self.state_driver = res.state_driver
        self.state_mes = res.state_mes
        self.state_error = res.state_error

    def wait(self):
        """処理待ちサービス関数

        規定時間後(30ms)、通信可能になるまでウェイトがかかるサービス

        """
        time.sleep(0.03)  # ウェイト時間の設定(1 = 1.00s)
        # 通信が終了するまでループ
        while (self.state_driver == 1):
            pass

    def rpm2ms(self, rotational_speed):
        translational_speed = rotational_speed * 2 * math.pi * self.wheel_radius / 60 / self.reduction_ratio
        return translational_speed

    def ms2rpm(self, translational_speed):
        rotational_speed = translational_speed / (2 * math.pi * self.wheel_radius) * 60 * self.reduction_ratio
        return int(math.fabs(rotational_speed))

    def commandCallback(self, msg):
        # 運転指令(FWD方向)(M1,START/STOP,RUN/BRAKEをON)
        self.query_msg.slave_id = 0x01     # 号機選択(Hex): 1号機
        self.query_msg.func_code = 1       # ファンクションコード選択: 1(Write)
        self.query_msg.write_addr = 124    # 先頭アドレス選択(Dec): 動作コマンド
        self.query_msg.write_num = 1       # 書き込みデータサイズ: 1 (32bit)
        if msg.slow_stop.data == True:
            # 減速停止
            self.query_msg.data[0] = 18      # 書き込みデータ: ONビット(0000 0000 0001 0010) = 18
            self.query_pub.publish(self.query_msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
            # self.wait()                # 処理待ち
        elif msg.fast_stop.data == True:
            # 即時停止
            self.query_msg.data[0] = 10      # 書き込みデータ: ONビット(0000 0000 0010 1010) = 10
            self.query_pub.publish(self.query_msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
            # self.wait()                # 処理待ち
        else:
            # 運転指令(FWD方向)(M1,START/STOP,RUN/BRAKEをON)
            if msg.left_vel < 0:
                self.query_msg.data[0] = 26        # 書き込みデータ: ONビット(0000 0000 0001 1010) = 26
            else:
                self.query_msg.data[0] = 58        # 書き込みデータ: ONビット(0000 0000 0011 1010) = 58
            self.query_pub.publish(self.query_msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
            # self.wait()                  # 処理待ち

            # 回転速度の設定
            self.query_msg.slave_id = 0x01         # 号機選択(Hex): 1号機
            self.query_msg.func_code = 1           # ファンクションコード選択: 1(Write)
            self.query_msg.write_addr = 1156       # 先頭アドレス選択(Dec): データNo.2 回転速度
            self.query_msg.write_num = 1           # 書き込みデータサイズ: 1 (32bit)
            self.query_msg.data[0] = self.ms2rpm(msg.left_vel) # 書き込みデータ: 500/700/900/1100/1300[r/min]
            self.query_pub.publish(self.query_msg)            # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
            # self.wait()                      # 処理待ち

        # 運転指令(REV方向)(M1,START/STOP,RUN/BRAKE,FWD/REVをON)
        self.query_msg.slave_id = 0x02     # 号機選択(Hex): 2号機
        self.query_msg.func_code = 1       # ファンクションコード選択: 1(Write)
        self.query_msg.write_addr = 124    # 先頭アドレス選択(Dec): 動作コマンド
        self.query_msg.write_num = 1       # 書き込みデータサイズ: 1 (32bit)
        # self.wait()                  # 処理待ち
        if msg.slow_stop.data == True:
            # 減速停止
            self.query_msg.data[0] = 18      # 書き込みデータ: ONビット(0000 0000 0001 0010) = 18
            self.query_pub.publish(self.query_msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
            # self.wait()                # 処理待ち
        elif msg.fast_stop.data == True:
            # 即時停止
            self.query_msg.data[0] = 10      # 書き込みデータ: ONビット(0000 0000 0010 1010) = 10
            self.query_pub.publish(self.query_msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
            # self.wait()                # 処理待ち
        else:
            # 運転指令(FWD方向)(M1,START/STOP,RUN/BRAKEをON)
            if msg.right_vel < 0:
                self.query_msg.data[0] = 26        # 書き込みデータ: ONビット(0000 0000 0001 1010) = 26
            else:
                self.query_msg.data[0] = 58        # 書き込みデータ: ONビット(0000 0000 0011 1010) = 58
            self.query_pub.publish(self.query_msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
            # self.wait()                  # 処理待ち

            # 回転速度の設定
            self.query_msg.slave_id = 0x02         # 号機選択(Hex): 2号機
            self.query_msg.func_code = 1           # ファンクションコード選択: 1(Write)
            self.query_msg.write_addr = 1156       # 先頭アドレス選択(Dec): データNo.2 回転速度
            self.query_msg.write_num = 1           # 書き込みデータサイズ: 1 (32bit)
            self.query_msg.data[0] = self.ms2rpm(msg.right_vel) # 書き込みデータ: 500/700/900/1100/1300[r/min]
            self.query_pub.publish(self.query_msg)            # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
            # self.wait()                      # 処理待ち

        # time.sleep(2)

        # 回転速度読み込み
        self.query_msg.slave_id = 0x01   # 号機選択(Hex): 1号機
        self.query_msg.func_code = 0     # ファンクションコード選択: 0(Read)
        self.query_msg.read_addr = 206   # 先頭アドレス選択(Dec): フィードバック速度[r/min](符号付)
        self.query_msg.read_num = 1      # 読み込みデータサイズ: 1 (32bit)
        self.query_pub.publish(self.query_msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        # self.wait()
        self.query_msg.slave_id = 0x02   # 号機選択(Hex): 2号機
        self.query_msg.func_code = 0     # ファンクションコード選択: 0(Read)
        self.query_msg.read_addr = 206   # 先頭アドレス選択(Dec): フィードバック速度[r/min](符号付)
        self.query_msg.read_num = 1      # 読み込みデータサイズ: 1 (32bit)
        self.query_pub.publish(self.query_msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        # self.wait()

        self.state_pub.publish(self.motor_state)

        print("FeedbackSpeed1 = {0:}[m/s]".format(self.motor_state.left_vel))     # 取得した速度の表示[r/min]
        print("FeedbackSpeed2 = {0:}[m/s]".format(self.motor_state.right_vel))     # 取得した速度の表示[r/min]
    
    def start_spin(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():

            rate.sleep()

if __name__ == '__main__':

	connecter = motorConnecter()
	connecter.start_spin()
