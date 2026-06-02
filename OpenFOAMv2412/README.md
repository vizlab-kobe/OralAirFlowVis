# OralAirFlowVis可視化（参考情報）

OralAirFlowをOpenFOAM v2412で動作させるためのドキュメントです．[OpenFOAM v2412の用意とKVSの用意](https://github.com/vizlab-kobe/OpenFOAMVis)はリンクを参照してください．

まずはOralAirFlowVisを入手します．
``` 
$ cd ~/Work/Github
$ git clone https://github.com/vizlab-kobe/OralAirFlowVis.git
$ cd OralAirFlowVis
```
今回はOralAirFlowVisに含まれる解析realistic-s3について，圧力pの等値面を可視化します．

## 実行方法
```
$ cd ~/Work/GitHub
$ git clone https://github.com/vizlab-kobe/OralAirFlowVis.git
$ cd OralAirFlowVis/OpenFOAMv2412
```
まずソルバーをビルドします．KVSをEGL版でビルドした場合には
```
$ cd my_rhoPimpleFoam
$ wclean && wmake
```
この結果`rhoPimpleFoam`が生成されます．

KVSをOSMesa版でビルドした場合には
```
$ cd my_rhoPimpleFoam
$ cp Make/options_osmesa Make/options
$ wclean && wmake
```

次に解析ディレクトリに移動し実行します．
```
$ cd ../realistic3
$ chmod +x run_insitu.sh
$ ./run_insitu.sh
```
`Output`ディレクトリに画像が出力されます．

## 実行結果の例

<img width="512" height="512" alt="OralAirFlowVis" src="https://github.com/user-attachments/assets/b6fc3d9b-23ce-46cc-9ca9-892fb14936d5" />

