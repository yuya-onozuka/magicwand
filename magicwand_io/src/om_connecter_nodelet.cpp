#include <ros/ros.h>
#include <om_modbus_master/om_query.h>
#include <om_modbus_master/om_response.h>
#include <om_modbus_master/om_state.h>

//Nodelet includes
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

//keyboard message include
#include <keyboard/Key.h>
#include <magicwand_msgs/Motor.h>

#include <boost/thread.hpp>
#include <cmath>


namespace mw
{

class omConnecter : public nodelet::Nodelet
{
public:
  virtual void onInit();
  void resCallback(const om_modbus_master::om_response msg);
  void stateCallback(const om_modbus_master::om_state msg);
  void leftCallback(const magicwand_msgs::Motor msg);
  void loop();
  void initQuery();
  void wait();
  int ms2rpm(double translational_speed);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber res_sub_, state_sub_, left_sub_;
  ros::Publisher query_pub_, left_state_pub_, right_state_pub_;

  om_modbus_master::om_query left_query_msg_;	
  magicwand_msgs::Motor left_state_, right_state_;

  /* グローバル変数 */
  int state_driver_;  /* 通信可能フラグ変数(0:通信可能,1:通信中) */
  int state_mes_;     /* メッセージ(0:メッセージなし,1:メッセージ到達,2:メッセージエラー) */
  int state_error_;   /* エラー(0:エラーなし,1:無応答,2:例外応答) */

  double wheel_radius_, reduction_ratio_;

  boost::shared_ptr<boost::thread> loop_thread_;
};

void omConnecter::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  //Get rosparam
  pnh_.getParam("wheel_radius", wheel_radius_);
  pnh_.getParam("reduction_ratio", reduction_ratio_);

  query_pub_ = nh_.advertise<om_modbus_master::om_query>("om_query1",1);	/* OMにノードに送信するまでの定義 */
  res_sub_ = nh_.subscribe("om_response1",1, &omConnecter::resCallback, this);				/* レスポンスのコールバック定義 */
  state_sub_ = nh_.subscribe("om_state1",1, &omConnecter::stateCallback, this);				/* レスポンスのコールバック定義 */
  left_sub_ = nh_.subscribe("left_command",1,&omConnecter::leftCallback, this);

  left_query_msg_.slave_id = 0x01;     // 号機選択(Hex): 1号機

  state_driver_ = 0; 
  state_mes_ = 0;
  state_error_ = 0;

  ros::Duration(1.0).sleep();	/* 1秒待機 */
	
  initQuery();	/* 初期化関数のコール */

  //loop process
  loop_thread_ = boost::shared_ptr<boost::thread>
          (new boost::thread(boost::bind(&omConnecter::loop, this)));
}

void omConnecter::initQuery()
{
    om_modbus_master::om_query msg;
    /* 運転入力方式の変更(3ワイヤ) */
	msg.slave_id = 0x00;	/* 号機選択(Hex): 0(ブロードキャスト) */
	msg.func_code = 1;		/* ファンクションコード選択: 1(Write) */
	msg.write_addr = 4160;	/* 先頭アドレス選択(Dec): 運転入力方式パラメータ */
	msg.write_num = 1;		/* 書き込みデータサイズ: 1(32bit) */
	msg.data[0] = 1;		/* 書き込みデータ: 0(2ワイヤ),1(3ワイヤ) */
	query_pub_.publish(msg);		/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();					/* 処理待ち */

	/* 運転データ 回展速度No.2を0[r/min]に初期化 */
	msg.slave_id = 0x00;	/* 号機選択(Hex): 0(ブロードキャスト) */
	msg.func_code = 1;		/* ファンクションコード選択: 1(Write) */
	msg.write_addr = 1156;	/* 先頭アドレス選択(Dec): データNo.2 回転速度 */
	msg.write_num = 1;		/* 書き込みデータサイズ: 1 (32bit) */
	msg.data[0] = 0;		/* 書き込みデータ: 0[r/min] */
	query_pub_.publish(msg);		/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();					/* 処理待ち */

	/* Configrationの実行 */
	msg.slave_id = 0x00;	/* 号機選択(Hex): 0(ブロードキャスト) */
	msg.func_code = 1;		/* ファンクションコード選択: 1(Write) */
	msg.write_addr = 396;	/* 先頭アドレス選択(Dec): Configration実行コマンド */
	msg.write_num = 1;		/* 書き込みデータサイズ: 1 (32bit) */
	msg.data[0] = 1;		/* 書き込みデータ: 1(実行) */
	query_pub_.publish(msg);		/* クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信 */
	wait();					/* 処理待ち */

    ROS_INFO("Initialize query");
}

void omConnecter::wait()
{
    ros::Duration(0.03).sleep();
	ros::spinOnce();
	
	/* ドライバの通信が終了するまでループ */
	while(state_driver_ == 1)
	{
		ros::spinOnce();
	}
}

int omConnecter::ms2rpm(double translational_speed)
{
    double rotational_speed = translational_speed / (2 * M_PI * wheel_radius_) * 60 * reduction_ratio_;
    return int(rotational_speed);
}

/*---------------------------------------------------------------------------*/
/** レスポンスコールバック関数

@details	購読したレスポンスデータをグローバル変数に反映する
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void omConnecter::resCallback(const om_modbus_master::om_response msg)
{
	if(msg.slave_id == 1 && msg.func_code == 3)
	{
		/* 号機番号が1かつ読み込みのときに値を更新 */
		left_state_.left_vel = msg.data[0];
        left_state_.header.stamp = ros::Time::now();
	}
	else if(msg.slave_id == 2 && msg.func_code == 3)
	{
		/* 号機番号が2かつ読み込みのときに値を更新 */
		right_state_.right_vel = msg.data[0];
        right_state_.header.stamp = ros::Time::now();
	}
}

/*---------------------------------------------------------------------------*/
/** ステータスコールバック関数

@details	購読したステータスデータをグローバル変数に反映する
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void omConnecter::stateCallback(const om_modbus_master::om_state msg)
{
	state_driver_ = msg.state_driver;
	state_mes_ = msg.state_mes;
	state_error_ = msg.state_error;
}

void omConnecter::leftCallback(const magicwand_msgs::Motor msg)
{
    left_query_msg_.func_code = 1;     // ファンクションコード選択: 1(Write)
    left_query_msg_.write_addr = 124;  // 先頭アドレス選択(Dec): 動作コマンド
    left_query_msg_.write_num = 1;     // 書き込みデータサイズ: 1 (32bit)
    if (msg.slow_stop.data == true)
    {
        // 減速停止
        left_query_msg_.data[0] = 18;      // 書き込みデータ: ONビット(0000 0000 0001 0010) = 18
        query_pub_.publish(left_query_msg_);      // クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        wait();                // 処理待ち
    }
    else if (msg.fast_stop.data == true)
    {
        // 即時停止
        left_query_msg_.data[0] = 10;      // 書き込みデータ: ONビット(0000 0000 0010 1010) = 10
        query_pub_.publish(left_query_msg_);      // クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        wait();                // 処理待ち
    }
    else
    {
        // 運転指令(FWD方向)(M1,START/STOP,RUN/BRAKEをON)
        if (msg.left_vel < 0)
        {
            left_query_msg_.data[0] = 26;        // 書き込みデータ: ONビット(0000 0000 0001 1010) = 26
        }
        else
        {
            left_query_msg_.data[0] = 58;        // 書き込みデータ: ONビット(0000 0000 0011 1010) = 58
        }
        query_pub_.publish(left_query_msg_);        // クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        wait();                  // 処理待ち

        // 回転速度の設定
        left_query_msg_.func_code = 1;           // ファンクションコード選択: 1(Write)
        left_query_msg_.write_addr = 1156;       // 先頭アドレス選択(Dec): データNo.2 回転速度
        left_query_msg_.write_num = 1;           // 書き込みデータサイズ: 1 (32bit)
        left_query_msg_.data[0] = ms2rpm(msg.left_vel); // 書き込みデータ: 500/700/900/1100/1300[r/min]
        query_pub_.publish(left_query_msg_);            // クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        wait();                      // 処理待ち
    }
}

void omConnecter::loop()
{
  ros::Rate loop_rate(30);

  while(getMTPrivateNodeHandle().ok())
  {
    loop_rate.sleep();
  }
}

}//namespace mw

PLUGINLIB_EXPORT_CLASS(mw::omConnecter, nodelet::Nodelet)
