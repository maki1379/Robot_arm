#include <Servo.h>

/* ────── 定数設定 ────── */
const uint8_t  NUM_SERVOS = 6;
const uint8_t  servoPins[NUM_SERVOS] = {3, 5, 6, 9, 10, 11};

const int16_t POS_INIT [NUM_SERVOS] = { 90,135,  0,180, 90,180};   // 初期
const int16_t POS_PICK [NUM_SERVOS] = { 90,  0, 40, 40, 90,  0};   // ピック
const int16_t POS_PLACE_R[NUM_SERVOS] = {  0, 0, 40, 40, 90,180};  // 右 (1軸=0°)
const int16_t POS_PLACE_L[NUM_SERVOS] = {180, 0, 40, 40, 90,180};  // 左 (1軸=180°)

const int16_t GRIP_CLOSE = 180;          // 6軸 つかむ
const int16_t GRIP_OPEN  =   0;          // 6軸 開放

const uint8_t  BTN_PIN  = 2;             // タクトスイッチ
const uint16_t STEP_MS  = 20;            // 1 ステップ間隔

/* ────── 変数 ────── */
Servo servos[NUM_SERVOS];
bool  waiting  = true;      // 初期姿勢で待機中？
bool  btnPrev  = HIGH;      // 前回のボタン状態
bool  toLeft   = false;     // false=次は右 / true=次は左

/* ────── 補助関数 ────── */
void moveAllSlow(const int16_t from[], const int16_t to[], uint16_t dly)
{
  int16_t cur[NUM_SERVOS]; memcpy(cur, from, sizeof(cur));
  bool done = false;
  while (!done) {
    done = true;
    for (uint8_t i = 0; i < NUM_SERVOS; ++i) {
      if      (cur[i] < to[i]) { ++cur[i]; done = false; }
      else if (cur[i] > to[i]) { --cur[i]; done = false; }
      servos[i].write(cur[i]);
    }
    delay(dly);
  }
}

void moveBodySlow(const int16_t from[], const int16_t to[],
                  uint16_t dly, int16_t gripAngle)
{
  int16_t cur[NUM_SERVOS]; memcpy(cur, from, sizeof(cur)); cur[5] = gripAngle;
  bool done = false;
  while (!done) {
    done = true;
    for (uint8_t i = 0; i < NUM_SERVOS - 1; ++i) {
      if      (cur[i] < to[i]) { ++cur[i]; done = false; }
      else if (cur[i] > to[i]) { --cur[i]; done = false; }
      servos[i].write(cur[i]);
    }
    servos[5].write(gripAngle);
    delay(dly);
  }
}

/* ---- ピック＆プレース ---- */
void runSequence(bool toLeftNow)
{
  /* ① 初期 → ピック */
  moveAllSlow(POS_INIT, POS_PICK, STEP_MS);

  /* ② つかむ */
  servos[5].write(GRIP_CLOSE);
  delay(500);

  /* ③ ピック → 初期（6軸固定）*/
  int16_t pickGrip[NUM_SERVOS], initGrip[NUM_SERVOS];
  memcpy(pickGrip, POS_PICK, sizeof(pickGrip));
  memcpy(initGrip, POS_INIT, sizeof(initGrip));
  pickGrip[5] = initGrip[5] = GRIP_CLOSE;
  moveBodySlow(pickGrip, initGrip, STEP_MS, GRIP_CLOSE);

  /* ④ 1軸回転 */
  int16_t rotate[NUM_SERVOS]; memcpy(rotate, initGrip, sizeof(rotate));
  rotate[0] = toLeftNow ? 180 : 0;
  moveBodySlow(initGrip, rotate, STEP_MS, GRIP_CLOSE);

  /* ⑤ 置き姿勢へ */
  const int16_t* PLACE = toLeftNow ? POS_PLACE_L : POS_PLACE_R;
  int16_t placeGrip[NUM_SERVOS]; memcpy(placeGrip, PLACE, sizeof(placeGrip));
  placeGrip[5] = GRIP_CLOSE;
  moveBodySlow(rotate, placeGrip, STEP_MS, GRIP_CLOSE);

  /* ⑥ 離す */
  servos[5].write(GRIP_OPEN);
  delay(500);

  /* ⑦ 初期へ戻る */
  moveAllSlow(placeGrip, POS_INIT, STEP_MS);
}

/* ────── セットアップ ────── */
void setup()
{
  Serial.begin(9600);
  pinMode(BTN_PIN, INPUT_PULLUP);

  for (uint8_t i = 0; i < NUM_SERVOS; ++i) {
    servos[i].attach(servoPins[i]);
    servos[i].write(POS_INIT[i]);
  }
  Serial.println(F("Ready: ボタンで右→左→右→…交互動作"));
}

/* ────── メインループ ────── */
void loop()
{
  /* ボタンの立下り検出 */
  bool btnNow = digitalRead(BTN_PIN);
  if (btnPrev == HIGH && btnNow == LOW && waiting) {
    waiting = false;                 // 動作開始
    runSequence(toLeft);             // 現在フラグに従って実行
    toLeft  = !toLeft;               // 次回方向を反転
    waiting = true;                  // 終了したら待機
  }
  btnPrev = btnNow;
}
