# dynamixel_controller
金澤がdynamixelを動かすときのテンプレート。使えるけど未完成。

## できること
  複数のDYNAMIXELを制御できます。
  コンフィグファイルで初期設定できます。
  
## 使い方
dynamixelworkbenchをクローンしてビルドしてください。
https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench_jp/

僕が使うときはコードを書き換えています

dynamixelworkbench getVelocityが使えないので、
catkin_ws/src/dynamixel_workbench/dynamixel_toolbox/dynamixel_workbench.cpp
1187?行
result=readRegister(id,"Present_Velocity",......
に変えると使えます。多分ソースコード間違ってる

このパッケージをクローンしてビルドしてください。

## ファイル構成

/configー設定を書く

/launchー

/srcーhedder

main　DYNAMIXELの一部がつながってないときに、プログラムを起動するか選択できる
     
class ここを書き換える。
            
            未完成なので、以下は自分で書き換えないと動きません
            
            ・サブスクライバのトピック名とコールバック関数
            
            ・dynamixelのPゲインなどをItemWriteする場合
            


## dynamixelを使うとき
/U2D2にDYNAMIXELとバッテリーをつないでください

rosrun dynamixel_workbench_controllers find_dynamixel /dev/ttyUSB0
で、得られたものをコンフィグに書く。model_numberはネットで検索する（確か間違ってても問題なかったような。。。）

このパッケージを起動する

## 注意
未完成です。コードを読んでから動かしてください

## 意味がわからなかったら
聞いてください
