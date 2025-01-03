「シミュレーション演算仕様書」をもとに「ソフトウェア基本設計書」を行ってください。

# ソフトウェア基本設計書の記載要求
## 条件
1. 基本設計以下が明確になること。
  - プログラムのファイル構成
  - 各処理のインタフェース
  - 入力ファイルのデータ構造
2. ECS（entity-component-syste）と関数型の双方を組み合わせた構造とすること。
3. モジュール構成は、汎用演算(math)とシミュレーション（models）の２つのモジュールを作成し、別プログラムで流用できるようにすること。
4. ファイル構成およびモジュール構成を明確にすること。
5. 3次元ベクトル計算・積分計算・ローパスフィルタなど汎用的な演算は、運動演算と切り離し汎用的に使えるようにしてください。
  - 可読性を重視し、汎用演算のうち積分計算・フォーパスフィルタは、ステートフルなオブジェクトに前回値を保持する構造にすること。

## 対象
Rust初心者のプログラマが、迷わないようモジュールやデータ構造を記載してください。

## 制約
- 数値計算部分に関してはcrateを多用せず、実装すること。
- プログラムの実装にあたっては、関数型プログラミングの原則に厳密に即すること。
   - ただし、可読性を重視し、汎用演算のうち積分計算・フォーパスフィルタは、ステートフルなオブジェクトに前回値を保持する構造とすること。

### 入力データについて
1. 各演算モデルの固定的な性能諸元は、モデルごとのパラメータファイルから読み込むこと。
2. 各オブジェクトの初期位置・初期姿勢などは、パラメータファイル群とは別の「シナリオファイル」から読み込むことと
3. 各設定ファイルは、yaml形式で記述すること。

### 出力データについて
1. 出力結果として、各サイクルの中距離弾道ミサイルと迎撃ミサイルの位置と姿勢角、レーダの探知状況および探知位置をCSVファイルとして出力すること。
2. csvのヘッダには、変数名とともに単位を記載すること。

# ミサイルシミュレーション 演算仕様書

## 1. 座標系と全般仕様

1. **シミュレーション空間座標系**  
   - 右手系 $(X, Y, Z)$ で、$Z$ 軸を上向き、$X$ 軸を東向き、$Y$ 軸を北向きとする。  
   - 以下では、弾道ミサイルの位置を $\boldsymbol{r}_t(t)$、迎撃ミサイルの位置を $\boldsymbol{r}_m(t)$ と表記し、いずれもこのシミュレーション空間座標系で管理する。  

2. **機体座標系**  
   - 各ミサイルに固有の座標系。オイラー角 ($\phi$ = ロール, $\theta$ = ピッチ, $\psi$ = ヨー) を用いて、機体座標系ベクトルをシミュレーション空間座標系に変換する。  
   - **ただしロール $\phi$ は常に 0** とし、$\theta,\psi$ のみ時変する。

3. **演算サイクル**  
   - 時間刻み $\Delta t$ [s] で、$t=0, \Delta t, 2\Delta t, \dots$ の各ステップで演算する。デフォルト値は 0.1 [s]。  

4. **重力モデル**  
   - 重力加速度 $g = 9.80665\,[\mathrm{m/s^2}]$ を一定とし、地球回転やジオイドは無視。

5. **大気密度モデル**  
   - 標準大気モデルを用い、高度 $z$ に応じて $\rho(z)$ を与える。  
   - 詳細は実装側で近似式やテーブルを参照してよい。

6. **空力抵抗**  
   - 抗力係数 $C_D$ と断面積 $A$ を用い、シミュレーション空間座標系で

     $
       \boldsymbol{F}_D =  -\,\tfrac12\,\rho(z)\,C_D\,A\,\|\boldsymbol{v}\|\;\boldsymbol{v}
     $
     とする。  

7. **推力特性**  
   - 燃焼時間中は一定推力 $T_\text{const}$、燃焼終了後は $0$ とする。

8. **ローパスフィルタ**  
   - 姿勢角演算や誘導演算による指令に一次遅れフィルタをかける。例えば  
     $
       y_{k+1} = y_k + \Delta t \,\frac{1}{\tau}\Bigl(u_k - y_k\Bigr).
     $
   - $\tau$ は性能値として指定。

9. **積分法 (Adams-Bashforth 2段法)**  
   - 速度・位置、姿勢角などの更新に用いる。

---

## 2. モデル分類・各種記号

### 2.1. 弾道ミサイル ($_t$) の性能値

| 変数名                      | 説明                           | 単位     |
|-----------------------------|--------------------------------|----------|
| $T_{\text{const}_t}$     | 弾道ミサイル推力（定値）        | N        |
| $t_{\text{burn}_t}$      | 弾道ミサイル燃焼時間            | s        |
| $m_{0_t}$                | 弾道ミサイル初期質量            | kg       |
| $\dot{m}_{\text{fuel}_t}$| 弾道ミサイル燃料消費率          | kg/s     |
| $C_{D_t}$                | 弾道ミサイル抗力係数            | –        |
| $A_t$                    | 弾道ミサイル断面積              | m²       |
| $\tau_{\theta_t},\tau_{\psi_t}$ | 姿勢角フィルタ時間定数   | s        |

### 2.2. 迎撃ミサイル ($_m$) の性能値

| 変数名                      | 説明                           | 単位     |
|-----------------------------|--------------------------------|----------|
| $T_{\text{const}_m}$     | 迎撃ミサイル推力（定値）        | N        |
| $t_{\text{burn}_m}$      | 迎撃ミサイル燃焼時間            | s        |
| $m_{0_m}$                | 迎撃ミサイル初期質量            | kg       |
| $\dot{m}_{\text{fuel}_m}$| 迎撃ミサイル燃料消費率          | kg/s     |
| $C_{D_m}$                | 迎撃ミサイル抗力係数            | –        |
| $A_m$                    | 迎撃ミサイル断面積              | m²       |
| $N_m$                    | 迎撃ミサイル比例航法定数        | –        |
| $\tau_{\theta_m},\tau_{\psi_m}$ | 姿勢角フィルタ時間定数   | s        |

### 2.3. レーダ ($_r$) の性能値

| 変数名               | 説明                     | 単位 |
|----------------------|--------------------------|------|
| $R_{\text{max}_r}$ | レーダ探知距離           | m    |
| $\Delta t_r$       | レーダ探知周期(0.1s)     | s    |
| $\boldsymbol{r}_{0_r}$| レーダ設置位置       | m    |
| $\psi_r, \theta_r$ | レーダ設置方位角, 仰角   | rad  |

### 2.4. 初期条件

- 例: 弾道ミサイル ($_t$)  
  - $\boldsymbol{r}_t(0)$, $\theta_t(0)$, $\psi_t(0)$, $\phi_t(0)=0$, $\boldsymbol{v}_t(0)$  
- 迎撃ミサイル ($_m$)  
  - $\boldsymbol{r}_m(0)$, $\theta_m(0)$, $\psi_m(0)$, $\phi_m(0)=0$, $\boldsymbol{v}_m(0)$  

---

## 3. シミュレーション中の変数

- $m_t(t)$, $m_m(t)$: 時刻 $t$ のミサイル質量  
- $\boldsymbol{r}_t(t)$, $\boldsymbol{r}_m(t)$: 位置ベクトル $(x,y,z)$  
- $\boldsymbol{v}_t(t)$, $\boldsymbol{v}_m(t)$: 速度ベクトル  
- $\theta_t(t), \psi_t(t)$, $\theta_m(t), \psi_m(t)$: ピッチ角・ヨー角  
- $\boldsymbol{F}_{T_t}(t)$, $\boldsymbol{F}_{T_m}(t)$: 推力ベクトル  
- $\boldsymbol{F}_{D_t}(t)$, $\boldsymbol{F}_{D_m}(t)$: 抗力ベクトル  
- $\boldsymbol{a}_t(t)$, $\boldsymbol{a}_m(t)$: 加速度ベクトル  

---

## 4. 質量変化の式

### 4.1. ミサイル質量更新
$
  m_t(t) = m_{0_t} \;-\; \dot{m}_{\text{fuel}_t}\;\min\Bigl(t,\;t_{\text{burn}_t}\Bigr),
$

$
  m_m(t) = m_{0_m} \;-\; \dot{m}_{\text{fuel}_m}\;\min\Bigl(t,\;t_{\text{burn}_m}\Bigr).
$

#### 四則演算での実装例
```plaintext
burnTime_t  = t_burn_t
fuelRate_t  = m_fuel_t
timeFactor  = min(currentTime, burnTime_t)
m_t(t)      = m_0_t - (fuelRate_t * timeFactor)
```
(迎撃ミサイル _m も同様)

---

## 5. 推力

### 5.1. 推力大きさ

$
  T_t(t) = \begin{cases} T_{\text{const}_t}, & (0 \le t < t_{\text{burn}_t}),\\ 0, & (t \ge t_{\text{burn}_t}), \end{cases}
$

$
  T_m(t) = \begin{cases} T_{\text{const}_m}, & (0 \le t < t_{\text{burn}_m}),\\ 0, & (t \ge t_{\text{burn}_m}). \end{cases}
$

#### 四則演算での実装例
```plaintext
if ( currentTime < t_burn_t ):
    Tt = T_const_t
else:
    Tt = 0
```
(迎撃ミサイル _m も同様)

### 5.2. 推力ベクトル（姿勢による向き）

機体座標系で $\begin{bmatrix}0\\0\\T\end{bmatrix}$ を、$\phi=0,\theta,\psi$ による回転行列 $R(0,\theta,\psi)$ でシミュレーション座標系へ変換:

$
  \boldsymbol{F}_{T}(t) = R(0,\theta,\psi) \begin{bmatrix}0\\0\\T(t)\end{bmatrix}.
$

**(ロール0の行列例)**  

$
  R(0,\theta,\psi) = \begin{bmatrix} \cos\psi\,\cos\theta & \sin\psi\,\cos\theta & -\sin\theta \\ -\sin\psi & \cos\psi & 0 \\ \cos\psi\,\sin\theta & \sin\psi\,\sin\theta & \cos\theta \end{bmatrix}.
$

#### 四則演算での実装例（ロール=0を想定）

- $F_{Tx} = T \cdot \bigl(\sin\psi \cdot 0 + \cdots\bigr)$ と書くよりも、要素展開:
  ```plaintext
  FT_x = (cos(psi)*cos(theta))*0 + (sin(psi)*cos(theta))*0 + (-sin(theta))*T
       = - T * sin(theta)

  FT_y = (-sin(psi))*0 + cos(psi)*0 + 0*T
       = 0   # 実際は要要素確認

  FT_z = (cos(psi)*sin(theta))*0 + (sin(psi)*sin(theta))*0 + cos(theta)*T
       = T * cos(theta)
  ```
  ただし、この行列要素は厳密に確認してください。行列の第1行第3列が $-\sin\theta$ であることなど、並びに注意が必要です。  

  実際には
  $
    F_{T_x} = T \bigl(\cos\psi\sin\theta\bigr),\quad
    F_{T_y} = T \bigl(\sin\psi\sin\theta\bigr),\quad
    F_{T_z} = T \bigl(\cos\theta\bigr),
  $
  となる形（行列の定義次第）。行列の要素は必ず式に基づきご確認ください。

---

## 6. 空力抵抗

$
  \boldsymbol{F}_D = -\,\tfrac12\,\rho(z)\,C_D\,A\,\|\boldsymbol{v}\|\;\boldsymbol{v}.
$
ここで $\|\boldsymbol{v}\|$ は速度ベクトルの大きさ。

#### 四則演算での実装例
```plaintext
vx = v_x(t)
vy = v_y(t)
vz = v_z(t)

speed = sqrt( vx^2 + vy^2 + vz^2 )

FD_x = -0.5 * rho(z) * CD * A * speed * vx
FD_y = -0.5 * rho(z) * CD * A * speed * vy
FD_z = -0.5 * rho(z) * CD * A * speed * vz
```

---

## 7. 加速度

ミサイル（弾道/迎撃いずれも）に作用する合力:

$
  \boldsymbol{F}_{\text{total}}(t)  =  \boldsymbol{F}_T(t)   + \boldsymbol{F}_D(t)  + m(t)\,\boldsymbol{g},
$

ただし $\boldsymbol{g}=(0,0,-g)$。  
よって

$
  \boldsymbol{a}(t) = \frac{\boldsymbol{F}_{\text{total}}(t)}{m(t)}.
$

#### 四則演算での実装例
```plaintext
Fx_total = FT_x + FD_x + m(t)*0
Fy_total = FT_y + FD_y + m(t)*0
Fz_total = FT_z + FD_z + m(t)*(-g)

a_x = Fx_total / m(t)
a_y = Fy_total / m(t)
a_z = Fz_total / m(t)
```

---

## 8. 姿勢角演算

### 8.1. 弾道ミサイル ($_t$)

- 簡易モデル: 発射時の $\theta_t(0), \psi_t(0)$ を固定し、誘導制御をしない場合は姿勢を一定とみなしてもよい。  
- あるいは、空力モーメント等を入れたい場合は別途式を設定する。

### 8.2. 迎撃ミサイル ($_m$)

- **比例航法**等で指令加速度 $\boldsymbol{a}_{\mathrm{cmd}_m}$ を求め、そこから必要な $\dot{\theta}_m, \dot{\psi}_m$ を導出。  
- ローパスフィルタで急激な変化を抑えたうえで、2段 Adams-Bashforth 法で $\theta_m, \psi_m$ を更新する:

$
  \theta_m(t+\Delta t) = \theta_m(t) +  \frac{\Delta t}{2}\bigl[3\,\dot{\theta}_m(t) - \dot{\theta}_m(t-\Delta t)\bigr],
$

$ 
\psi_m(t+\Delta t) = \psi_m(t) +  \frac{\Delta t}{2}\bigl[3\,\dot{\psi}_m(t) - \dot{\psi}_m(t-\Delta t)\bigr].
$

#### 四則演算での実装例
```plaintext
theta_m_new = theta_m[t] + 0.5*dt * ( 3*theta_dot_m[t] - theta_dot_m[t-1] )
psi_m_new   = psi_m[t]   + 0.5*dt * ( 3*psi_dot_m[t]   - psi_dot_m[t-1]   )
```

---

## 9. 速度・位置の更新 (Adams-Bashforth 法)

### 9.1. 速度更新

$
  \boldsymbol{v}(t+\Delta t) = \boldsymbol{v}(t) + \frac{\Delta t}{2} \Bigl[ 3\,\boldsymbol{a}(t) - \boldsymbol{a}(t-\Delta t) \Bigr].
$
成分ごとに:
$
  v_x(t+\Delta t) = v_x(t) + \tfrac{\Delta t}{2}\bigl[3\,a_x(t) - a_x(t-\Delta t)\bigr],
$

$
 v_y(t+\Delta t) = v_y(t) + \tfrac{\Delta t}{2}\bigl[3\,a_y(t) - a_y(t-\Delta t)\bigr],
$

$
  v_z(t+\Delta t) = v_z(t) + \tfrac{\Delta t}{2}\bigl[3\,a_z(t) - a_z(t-\Delta t)\bigr].
$

#### 四則演算での実装例
```plaintext
vx_new = vx[t] + 0.5*dt * ( 3*ax[t] - ax[t-1] )
vy_new = vy[t] + 0.5*dt * ( 3*ay[t] - ay[t-1] )
vz_new = vz[t] + 0.5*dt * ( 3*az[t] - az[t-1] )
```

### 9.2. 位置更新

$
  \boldsymbol{r}(t+\Delta t) = \boldsymbol{r}(t) + \frac{\Delta t}{2} \Bigl[ 3\,\boldsymbol{v}(t) - \boldsymbol{v}(t-\Delta t) \Bigr].
$

成分ごとに:
$
  x(t+\Delta t) = x(t) + \tfrac{\Delta t}{2}\bigl[3\,v_x(t) - v_x(t-\Delta t)\bigr],
$

$
  y(t+\Delta t) = y(t) + \tfrac{\Delta t}{2}\bigl[3\,v_y(t) - v_y(t-\Delta t)\bigr],
$

$
  z(t+\Delta t) = z(t) + \tfrac{\Delta t}{2}\bigl[3\,v_z(t) - v_z(t-\Delta t)\bigr].
$

#### 四則演算での実装例
```plaintext
x_new = x[t] + 0.5*dt * ( 3*vx[t] - vx[t-1] )
y_new = y[t] + 0.5*dt * ( 3*vy[t] - vy[t-1] )
z_new = z[t] + 0.5*dt * ( 3*vz[t] - vz[t-1] )
```

---

## 10. 迎撃判定

弾道ミサイルと迎撃ミサイルの相対距離が閾値 $\epsilon$ 以下であれば「迎撃成功」とする。

$
  \|\boldsymbol{r}_m(t) - \boldsymbol{r}_t(t)\| \;\le\; \epsilon.
$

#### 四則演算での実装例
```plaintext
dx = x_m - x_t
dy = y_m - y_t
dz = z_m - z_t
dist = sqrt( dx^2 + dy^2 + dz^2 )

if ( dist <= epsilon ) then
    # 迎撃成功処理
```

---

## 11. 座標変換（機体座標 → シミュレーション空間座標）

### 11.1. 回転行列（ロール=0の場合）

$
  R(0,\theta,\psi) = \begin{bmatrix}  \cos\psi \cos\theta & \sin\psi \cos\theta & -\sin\theta \\  -\sin\psi & \cos\psi & 0 \\  \cos\psi \sin\theta & \sin\psi \sin\theta & \cos\theta \end{bmatrix}.
$

### 11.2. ベクトル変換

$
  \begin{bmatrix}   X \\ Y \\ Z \end{bmatrix}_{(\text{sim})} = R(0,\theta,\psi)
  \begin{bmatrix}   X' \\ Y' \\ Z' \end{bmatrix}_{(\text{body})}.
$

- 推力ベクトル $\boldsymbol{F}_T$ は機体座標系で $\begin{bmatrix}0\\0\\T\end{bmatrix}$ → シミュレーション空間座標系へ。  

#### 四則演算での実装例
```plaintext
FT_x = + ( cos(psi)*cos(theta) ) * 0 
        + ( sin(psi)*cos(theta) ) * 0
        + ( -sin(theta) )        * T
      = - T * sin(theta)

FT_y = + ( -sin(psi) ) * 0
        + ( cos(psi) )  * 0
        + ( 0 )         * T
      = 0   # (行列を再確認して要調整)

FT_z = + ( cos(psi)*sin(theta) )*0
        + ( sin(psi)*sin(theta) )*0
        + ( cos(theta) )        * T
      = T * cos(theta)
```
（※ 行列要素は実際の定義と照らし合わせて整合を取ってください。上表は一例の並びです。）

---

## 12. レーダ演算

1. **探知判定**  
   - レーダ位置 $\boldsymbol{r}_{0_r}$ と弾道ミサイル位置 $\boldsymbol{r}_t(t)$ の距離が $R_{\text{max}_r}$ 以下 → 「探知」。  
     $
       \|\boldsymbol{r}_t(t) - \boldsymbol{r}_{0_r}\| \;\le\; R_{\text{max}_r}.
     $  
2. **探知周期**  
   - $\Delta t_r = 0.1$ [s] ごとに探知。  

#### 四則演算例
```plaintext
dx = x_t - x_r
dy = y_t - y_r
dz = z_t - z_r
dist = sqrt(dx^2 + dy^2 + dz^2)
if ( dist <= R_max_r ):
    # ミサイルを探知
```

---

## 13. シミュレーションの流れ

1. **初期化**  
   - 弾道ミサイル($_t$)・迎撃ミサイル($_m$)・レーダ($_r$) の性能値・初期値を設定  
   - $\boldsymbol{r}_t(0),\boldsymbol{v}_t(0),\theta_t(0),\dots$ など各変数を初期化。  

2. **メインループ**  
   - (a) レーダ探知判定  
   - (b) 迎撃ミサイル誘導演算（比例航法など）  
   - (c) 姿勢角計算（ローパスフィルタ適用、Adams-Bashforthで更新）  
   - (d) 推力算出 (5.1参照)  
   - (e) 空力抵抗算出 (6参照)  
   - (f) 合力→加速度計算 (7参照)  
   - (g) 速度・位置更新 (9参照)  
   - (h) 迎撃判定 (10参照) & 地表衝突などの終了判定  

3. **終了条件**  
   - 弾道ミサイルが迎撃成功 or 地表(z=0) 到達 → その弾道ミサイルは演算打切り。  
   - 迎撃ミサイルも目標喪失 or 地表衝突で演算終了。  

---

# 変数一覧

最後に、**演算モデルの性能値・初期条件・変数値**の3種類をまとめます。

## (A) 演算モデルの性能値

| 区分 | 変数名                          | 説明                                          | 単位 |
|:---:|----------------------------------|-----------------------------------------------|:----:|
| _t  | $T_{\text{const}_t}$           | 弾道ミサイル推力                              | N    |
| _t  | $t_{\text{burn}_t}$            | 弾道ミサイル燃焼時間                          | s    |
| _t  | $m_{0_t}$                      | 弾道ミサイル初期質量                          | kg   |
| _t  | $\dot{m}_{\text{fuel}_t}$      | 弾道ミサイル燃料消費率                        | kg/s |
| _t  | $C_{D_t}$                      | 弾道ミサイル抗力係数                          | –    |
| _t  | $A_t$                          | 弾道ミサイル断面積                            | m²   |
| _t  | $\tau_{\theta_t},\tau_{\psi_t}$| (必要なら) 姿勢フィルタ時間定数              | s    |

| 区分 | 変数名                          | 説明                                          | 単位 |
|:---:|----------------------------------|-----------------------------------------------|:----:|
| _m  | $T_{\text{const}_m}$           | 迎撃ミサイル推力                              | N    |
| _m  | $t_{\text{burn}_m}$            | 迎撃ミサイル燃焼時間                          | s    |
| _m  | $m_{0_m}$                      | 迎撃ミサイル初期質量                          | kg   |
| _m  | $\dot{m}_{\text{fuel}_m}$      | 迎撃ミサイル燃料消費率                        | kg/s |
| _m  | $C_{D_m}$                      | 迎撃ミサイル抗力係数                          | –    |
| _m  | $A_m$                          | 迎撃ミサイル断面積                            | m²   |
| _m  | $N_m$                          | 迎撃ミサイル比例航法定数                      | –    |
| _m  | $\tau_{\theta_m},\tau_{\psi_m}$| 迎撃ミサイル姿勢フィルタ時間定数              | s    |

| 区分 | 変数名                     | 説明                                  | 単位 |
|:---:|-----------------------------|---------------------------------------|:----:|
| _r  | $R_{\text{max}_r}$        | レーダ探知距離                        | m    |
| _r  | $\Delta t_r$             | レーダ探知周期 (0.1s)                 | s    |
| _r  | $\boldsymbol{r}_{0_r}$    | レーダ設置位置                        | m    |
| _r  | $\psi_r,\theta_r$         | レーダ設置方位角, 仰角                | rad  |

| 共通 | $g$                    | 重力加速度(9.80665)                    | m/s² |
| 共通 | $\rho_0,\beta,T_0,\alpha$| 標準大気モデル定数                    | –    |
| 共通 | $\Delta t$             | 演算サイクル(0.1s)                     | s    |

## (B) 演算モデルの初期条件

| 区分 | 変数名            | 説明                                          | 単位 |
|:---:|--------------------|-----------------------------------------------|:----:|
| _t  | $\boldsymbol{r}_t(0)$| 弾道ミサイル発射位置                     | m    |
| _t  | $\theta_t(0)$    | 弾道ミサイル発射ピッチ角                     | rad  |
| _t  | $\psi_t(0)$      | 弾道ミサイル発射ヨー角                       | rad  |
| _t  | $\phi_t(0)$      | 弾道ミサイルロール角(0固定)                  | rad  |
| _t  | $\boldsymbol{v}_t(0)$| 弾道ミサイル発射時速度ベクトル           | m/s  |

| 区分 | 変数名            | 説明                                          | 単位 |
|:---:|--------------------|-----------------------------------------------|:----:|
| _m  | $\boldsymbol{r}_m(0)$| 迎撃ミサイル発射位置                     | m    |
| _m  | $\theta_m(0)$    | 迎撃ミサイル発射ピッチ角                     | rad  |
| _m  | $\psi_m(0)$      | 迎撃ミサイル発射ヨー角                       | rad  |
| _m  | $\phi_m(0)$      | 迎撃ミサイルロール角(0固定)                  | rad  |
| _m  | $\boldsymbol{v}_m(0)$| 迎撃ミサイル発射時速度ベクトル           | m/s  |

| 区分 | 変数名     | 説明                 | 単位 |
|:---:|------------|----------------------|:----:|
| _r  | $\boldsymbol{r}_r$| レーダ設置位置 | m    |

## (C) シミュレーション中に変動する変数

| 区分 | 変数名                   | 説明                                                 | 単位   |
|:---:|---------------------------|------------------------------------------------------|:------:|
| _t/_m | $m_t(t)$, $m_m(t)$   | 質量                                                 | kg     |
| _t/_m | $\boldsymbol{r}_t(t)$, $\boldsymbol{r}_m(t)$| 位置ベクトル                      | m      |
| _t/_m | $\boldsymbol{v}_t(t)$, $\boldsymbol{v}_m(t)$| 速度ベクトル                      | m/s    |
| _t/_m | $\boldsymbol{a}_t(t)$, $\boldsymbol{a}_m(t)$| 加速度ベクトル                    | m/s²   |
| _t/_m | $\theta_t(t), \psi_t(t)$, $\theta_m(t), \psi_m(t)$| ピッチ角・ヨー角      | rad    |
| _t/_m | $\phi_t(t)$, $\phi_m(t)$ | ロール角 (常に 0 だが変数として定義)               | rad    |
| _t/_m | $\dot{\theta}_t(t)$, $\dot{\psi}_t(t)$, … | 姿勢角速度                                          | rad/s  |
| _t/_m | $\boldsymbol{F}_{T_t}(t)$, $\boldsymbol{F}_{T_m}(t)$| 推力ベクトル         | N      |
| _t/_m | $\boldsymbol{F}_{D_t}(t)$, $\boldsymbol{F}_{D_m}(t)$| 抗力ベクトル         | N      |
| _m    | $\boldsymbol{a}_{\mathrm{cmd}_m}(t)$        | 迎撃ミサイル指令加速度（比例航法など）               | m/s²   |
| _r    | –                           | レーダ内部の処理で使う位置・探知情報                  | –      |
