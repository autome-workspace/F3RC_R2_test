/*
基準について
「機体を上から見て」は機体の前方向を上にしたときを基準とする。
進行方向θは、上から見て機体の右方向を0度とし、反時計回りを正とする。
機体の回転についても反時計回りを正とする。
モーターについて、機体が前に進む向きを正とする。

#define Apin 41
#define Bpin 38
#define steps 8192

RotaryEncoderPCNT encoder(Apin, Bpin);

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.print("***Ready***");
}

void loop(){
  Serial.println(encoder.position());
  delay(200);
}

*/

#include <Arduino.h>
#include <math.h>
#include <RotaryEncoderPCNT.h>


const int Apin[4] = {0, 0, 0, 0};
const int Bpin[4] = {0, 0, 0, 0};

const int PULSE_RATE = 1000; // [pulse/s]
const int PWM_MAX = 255; //最大PWM値

const float aMAX = 60; //[mm/s^2] モーターの最大加速度（接地面基準）
float aLIMIT = aMAX; //[mm/s^2] 機体の加速度制限（接地面基準）
const float vMAX = 100; //[mm/s] モーターの最大速度(接地面基準）
float vLIMIT = vMAX; //[mm/s] 機体の速度制限（接地面基準）

const float WHEEL_X = 100, WHEEL_Y = 100; //ホイールのX, Y [mm]
const float K[4] = {1, 1, 1, 1}; //モーターのK値（モーターごとの調整）


void setup() {
  Serial.begin(115200);
  for(int i = 0; i < 4; i++) {
    pinMode(Apin[i], OUTPUT);
    pinMode(Bpin[i], OUTPUT);
  }

}

int pwm[4] = {0}; //各モーターのPWM値

void motor_write() {
  for(int i = 0; i < 4; i++) {
    if(pwm[i] >= 0) {
      analogWrite(Apin[i], pwm[i]);
      analogWrite(Bpin[i], 0);
    } else {
      analogWrite(Apin[i], 0);
      analogWrite(Bpin[i], -pwm[i]);
    }
  }
  return;
}

int translation_motion(float lengh, float angle) { //移動中は０、移動終了時に１を返す
  static int step = 0; //現在のステップ
  static int step_remember = 0; //段階変化時のステップ
  static int step_count = 0; //ステップのカウント
  static int phase = 0; //現在の段階
  static float cv = 0; //現在の速度
  static float at_MAX = 0; //移動偏角ごとの加速度
  static float vt_MAX = 0; //移動偏角ごとの速度
  static float pre_lengh = 0; //前回の目標移動距離
  static float pre_angle = 0; //前回の目標移動偏角

  if( lengh == 0 || angle == 0) { //目標移動距離が0または目標移動偏角が0ならば
    for(int i = 0; i < 4; i++) pwm[i] = 0; //モーターを停止
    motor_write(); //モーターにPWM値を書き込む
    return 1; //終了
  } else if(pre_lengh != lengh || pre_angle != angle) {
    if(cv != 0) {
      cv -= at_MAX / PULSE_RATE; //現在の速度を減速
      if(cv < 0) cv = 0; //速度を0に制限
      for(int i = 0; i < 4; i++)
        pwm[i] = K[i] * PWM_MAX * cv / vMAX * 
        (sinf(pre_angle) + ((i%2)*2-1)*cosf(pre_angle)); //各モーターのPWM値を計算
      motor_write(); //モーターにPWM値を書き込む
      return 0; //継続
    } else { //移動開始
      step = 0; //ステップをリセット
      phase = 0; //段階をリセット
      at_MAX = aMAX / (abs(sinf(angle)) + abs(cosf(angle))); //移動偏角ごとの最大加速度
      if(at_MAX > aLIMIT) at_MAX = aLIMIT; //加速度制限
      vt_MAX = vMAX / (abs(sinf(angle)) + abs(cosf(angle))); //移動偏角ごとの最大速度)
      if(vt_MAX > vLIMIT) vt_MAX = vLIMIT; //速度制限
      step_count = PULSE_RATE * (lengh / vt_MAX - vt_MAX / at_MAX); //定速段階のステップ数を計算
      if(step_count < 0) {
        step_count = PULSE_RATE * sqrt(lengh / at_MAX); //定速段階がない場合は加速段階のステップ数を計算
        phase = 3; //最大まで加速しないとき特殊フェーズに移行
      }
    }
  }

  switch(phase) {
    case 0: //加速段階
      cv += at_MAX / PULSE_RATE; //加速度を適用
      if(cv > vt_MAX) { //最大速度を超えたら
        cv = vt_MAX; //最大速度に制限
        phase = 1; //定速段階へ移行
        step_remember = step; //段階変化時のステップを記憶
      } break;
    case 1: //定速段階
      if(step - step_remember >= step_count) { //目標距離に到達したら
        phase = 2; //減速段階へ移行
      } break;
    case 2: //減速段階
      cv -= at_MAX / PULSE_RATE; //減速度を適用
      if(cv < 0) { //速度が0未満になったら
        cv = 0; //速度を0に制限
        for(int i = 0; i < 4; i++) pwm[i] = 0;
        motor_write(); //モーターにPWM値を書き込む
        return 1; //終了
      } break;
    case 3: //最大まで加速しない特殊段階
      if(step >= step_count) {
        phase = 2; //減速段階へ移行
      } else {
        cv += at_MAX / PULSE_RATE; //加速度を適用
      }
      if(cv > vt_MAX) { //最大速度を超えたら
        cv = vt_MAX; //最大速度に制限
        return 1; //終了
      } break;
    default: break; //不正な段階は無視
  }
  step++; //ステップを進める

  for(int i = 0; i < 4; i++)
    pwm[i] = K[i] * (PWM_MAX/vMAX) * cv * 
    (sinf(angle) + ((i%2)*2-1)*cosf(angle)); //各モーターのPWM値を計算
  
  motor_write(); //モーターにPWM値を書き込む

  pre_lengh = lengh;
  pre_angle = angle;

  return 0; //継続
}

void loop() {
  static int main_phase = 0; //メインフェーズ
  static float target_length = 0; //目標移動距離
  static float target_angle = 0; //目標移動偏角
  aLIMIT = aMAX; //加速度制限をリセット
  vLIMIT = vMAX; //速度制限をリセット

  while(micros() % (1000000 / PULSE_RATE) > 1) {} //一定間隔でループを実行
  
  switch(main_phase) {

    //単純な移動
    case 0:
      target_length = 100; //目標移動距離を設定
      target_angle = M_PI / 2; //目標移動偏角を設定
      if(translation_motion(target_length, target_angle)) { //移動が終了したら
        main_phase = 1; //次のフェーズへ移行
      }
      break;

    //慎重な移動
    case 1:
      target_length = 100; //目標移動距離を設定
      target_angle = M_PI * 3 / 2; //目標移動偏角を設定
      aLIMIT = 20; //加速度制限を設定
      vLIMIT = 30; //速度制限を設定
      if(translation_motion(target_length, target_angle)) { //移動が終了したら
        main_phase = 2; //次のフェーズへ移行
      }
      break;

    //探りながら移動
    case 2:
      target_length = 100; //目標移動距離を設定
      target_angle = 0; //目標移動偏角を設定
      aLIMIT = 20; //加速度制限を設定
      vLIMIT = 30; //速度制限を設定
      if(translation_motion(target_length, target_angle)) { //移動が終了したら
        main_phase = 2; //次のフェーズへ移行
      } else {
        //ここに監視コード
        /*
        example:
        if(sensor_data > threshold) {
          translation_motion(0, 0); //移動を急停止
          Serial.println("Sensor triggered, stopping motors."); 
        }
        */
      }
      break;
    
    //不正なフェーズ
    default:
      translation_motion(0, 0); //不正なフェーズは移動を急停止
      break;
  }
}