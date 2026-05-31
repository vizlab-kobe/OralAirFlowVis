# OralAirFlowVis可視化
OralAirFlowをOpenFOAM v2412で動作させるためのドキュメントです．[OpenFOAM v2412の用意とKVSの用意](https://github.com/vizlab-kobe/OpenFOAMVis)はリンクを参照してください．

まずはOralAirFlowVisを入手します
``` 
$ cd ~/Work/Github
$ git clone https://github.com/vizlab-kobe/OralAirFlowVis.git
$ cd OralAirFlowVis
```
今回はOralAirFlowVisに含まれる解析realistic-s3について，圧力pの等値面を可視化します．

## 実行方法
coming soon..

## ソルバーの改造の詳細
OralAirFlowVisに含まれているrhoPimpleFoamはOpenFOAM 2.3.1に対応したものであり，そのままOpenFOAM v2412で使用することはできません．
OpenFOAM v2412のソルバーを改造します．

``` 
$ cp –r $FOAM_SOLVERS/compressible/rhoPimpleFoam ./my_rhoPimpleFoam
$ cd my_rhoPimpleFoam
$ cp ../rhoPimpleFoam_InSituVis/InSituVis.h .
```

`rhoPimpleFoam.C`を`OralAirFlowVis/rhoPimpleFoam_InSituVis/rhoPimpleFoam.C`を参考に改造します．

追記は計5箇所です．

1つ目．ヘッダー部分は
```cpp
#include "fvCFD.H"
#include "dynamicFvMesh.H"
#include "fluidThermo.H"
#include "turbulentFluidThermoModel.H"
#include "bound.H"
#include "pimpleControl.H"
#include "pressureControl.H"
#include "CorrectPhi.H"
#include "fvOptions.H"
#include "localEulerDdtScheme.H"
#include "fvcSmooth.H"

// In-situ visualization
#define IN_SITU_VIS
#if defined( IN_SITU_VIS )
#include "InSituVis.h"
#include <InSituVis/Lib.foam/FoamToKVS.h>

// IN_SITU_VIS__P: Pressure
// IN_SITU_VIS__U: Velocity
// IN_SITU_VIS__T: Temperature
#define IN_SITU_VIS__P
#endif
```
とします．**`#define IN_SITU_VIS`から`#endif`までが追記内容**です．以下同じです．今回は圧力Pの可視化を試みます．

2つ目．main関数内部にて
```cpp
    turbulence->validate();

    if (!LTS)
    {
        #include "compressibleCourantNo.H"
        #include "setInitialDeltaT.H"
    }

#if defined( IN_SITU_VIS )
    // In-situ visualization setup
    Foam::messageStream::level = 0; // Disable Foam::Info
    const kvs::Indent indent(4); // indent for log stream
    local::InSituVis vis( MPI_COMM_WORLD );
    if ( !vis.initialize() )
      {
        vis.log() << "ERROR: " << "Cannot initialize visualization process." << std::endl;
        vis.world().abort();
      }

    // Time-loop information
    const auto start_time = runTime.startTime().value();
    const auto start_time_index = runTime.startTimeIndex();
    const auto end_time = runTime.endTime().value();
    const auto end_time_index = static_cast<int>( end_time / runTime.deltaT().value() );
    vis.log() << std::endl;
    vis.log() << "STARTING TIME LOOP" << std::endl;
    vis.log() << indent << "Start time and index: " << start_time << ", " << start_time_index << std::endl;
    vis.log() << indent << "End time and index: " << end_time << ", " << end_time_index << std::endl;
    vis.log() << std::endl;
#endif // IN_SITU_VIS
    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //

    Info<< "\nStarting time loop\n" << endl;
```
を追記します．

3つ目．時間発展のwhile文の手前です．
```cpp
        if (LTS)
        {
            #include "setRDeltaT.H"
        }
        else
        {
            #include "compressibleCourantNo.H"
            #include "setDeltaT.H"
        }

        ++runTime;

#if defined( IN_SITU_VIS )
        // Loop information
        const auto current_time_value = runTime.value();
        const auto current_time_index = runTime.timeIndex();
        vis.log() << "LOOP[" << current_time_index << "/" << end_time_index << "]: " << std::endl;
        vis.log() << indent << "T: " << current_time_value << std::endl;
        vis.log() << indent << "End T: " << end_time << std::endl;
        vis.log() << indent << "Delta T: " << runTime.deltaT().value() << std::endl;
        vis.simTimer().start();
#endif // IN_SITU_VIS

        Info<< "Time = " << runTime.timeName() << nl << endl;

        // --- Pressure-velocity PIMPLE corrector loop
        while (pimple.loop())
        {
```

4つ目．時間発展のwhile文の内部です．最も重要な部分です．各格子ごとにvis.putで可視化パイプラインに引き渡し，具体的な可視化を行います．
```cpp
        rho = thermo.rho();

        runTime.write();

#if defined( IN_SITU_VIS )
        vis.simTimer().stamp();
        const auto ts = vis.simTimer().last();
        const auto Ts = kvs::String::From( ts, 4 );
        vis.log() << indent << "Processing Times:" << std::endl;
        vis.log() << indent.nextIndent() << "Simulation: " << Ts << " s" << std::endl;

        // Execute in-situ visualization process
#if defined( IN_SITU_VIS__P ) // p: pressure
        auto& field = p;
        // A (whole min/max values)
        //const auto min_value = 97928.796875;
        //const auto max_value = 106227.53906;
        //const auto min_value = 0.99998 * 100000.0;
        //const auto max_value = 1.02000 * 100000.0;
        // B
        //const auto min_value = 9.94 * 10000.0;
        //const auto max_value = 1.02 * 100000.0;
        // C
        //const auto min_value = 0.999999 * 100000.0;
        //const auto max_value = 1.000020 * 100000.0;
        const auto min_value = 0.999990 * 100000.0;
        const auto max_value = 1.000200 * 100000.0;
#elif defined( IN_SITU_VIS__U ) // U: velocity
        auto& field = U;
        // A (whole min/max values)
        const auto min_value = 0.0;
        const auto max_value = 71.645393372;
        //const auto min_value = 0.0224;
        //const auto max_value = 70.9;
#elif defined( IN_SITU_VIS__T ) // T: temperature
        auto& field = thermo.T();
        // A (whole min/max values)
        const auto min_value = 289.91583252;
        const auto max_value = 296.15917969;
        //const auto min_value = 290.0;
        //const auto max_value = 296.16;
#endif

        // Convert OpenFOAM data to KVS data
        vis.cnvTimer().start();
        InSituVis::foam::FoamToKVS converter( field );
        using CellType = InSituVis::foam::FoamToKVS::CellType;
        auto vol_tet = converter.exec( vis.world(), field, CellType::Tetrahedra );
        auto vol_hex = converter.exec( vis.world(), field, CellType::Hexahedra );
        auto vol_pri = converter.exec( vis.world(), field, CellType::Prism );
        auto vol_pyr = converter.exec( vis.world(), field, CellType::Pyramid );
        vis.cnvTimer().stamp();

        vol_tet.setName("Tet");
        vol_hex.setName("Hex");
        vol_pri.setName("Pri");
        vol_pyr.setName("Pyr");

        vol_tet.setMinMaxValues( min_value, max_value );
        vol_hex.setMinMaxValues( min_value, max_value );
        vol_pri.setMinMaxValues( min_value, max_value );
        vol_pyr.setMinMaxValues( min_value, max_value );

        const auto tc = vis.cnvTimer().last();
        const auto Tc = kvs::String::From( tc, 4 );
        vis.log() << indent.nextIndent() << "Conversion: " << Tc << " s" << std::endl;

        // Execute visualization pipeline and rendering
        vis.visTimer().start();
        vis.put( vol_tet );
        vis.put( vol_hex );
        vis.put( vol_pri );
        vis.put( vol_pyr );
        vis.exec( { current_time_value, current_time_index } );
        vis.visTimer().stamp();

        const auto tv = vis.visTimer().last();
        const auto Tv = kvs::String::From( tv, 4 );
        vis.log() << indent.nextIndent() << "Visualization: " << Tv << " s" << std::endl;

        const auto elapsed_time = runTime.elapsedCpuTime();
        vis.log() << indent << "Elapsed Time: " << elapsed_time << " s" << std::endl;
        vis.log() << std::endl;
#endif // IN_SITU_VIS

        runTime.printExecutionTime(Info);
    }
```

5つ目．解析終了後に出力するログです．
```cpp
        runTime.printExecutionTime(Info);
    }

#if defined( IN_SITU_VIS )
    if ( !vis.finalize() )
    {
        vis.log() << "ERROR: " << "Cannot finalize visualization process." << std::endl;
        vis.world().abort();
    }
#endif // IN_SITU_VIS

    Info<< "End\n" << endl;

    return 0;
}
```
## ソルバーのコンパイルの設定
### `Make/files`の編集
`Make/files`を以下の通り編集します
```bash
rhoPimpleFoam.C

EXE = rhoPimpleFoam
```

なお，ここで改造したソルバーをOralAirFlow以外の別の解析で使用したい場合には
```bash
rhoPimpleFoam.C

EXE = $(FOAM_USER_APPBIN)/my_PimpleFoam
```
とします．上記の設定をすれば，`my_PimpleFoam`というコマンドがローカル環境に登録されます．

### `Make/options`の編集（EGL版）
EGL版のKVSを使用する場合には以下の通り編集して下さい．
```diff
EXE_INC = \
（数行省略）
    -I$(LIB_SRC)/TurbulenceModels/compressible/lnInclude \
    -I$(LIB_SRC)/regionFaModels/lnInclude \
+    -I$(HOME)/local/openmpi-4.1.2/include \ 

EXE_LIBS = \
（省略）


/* KVS settings (EGL / GPU Mode) */
+EXE_INC += \
+    -I${KVS_DIR}/include -DKVS_SUPPORT_MPI -DKVS_USE_MPI \
+    -DKVS_SUPPORT_EGL -DEGL_NO_X11 -DMESA_EGL_NO_X11_HEADERS

+EXE_LIBS += \
+    -L${KVS_DIR}/lib -lkvsSupportMPI \
+    -lkvsSupportEGL -lkvsCore \
+    -lEGL -lGL

/* InSitu settings */
+EXE_INC += -I$(HOME)/Work/Github
+EXE_LIBS += -L$(HOME)/Work/Github/InSituVis/Lib -lInSituVis

/* OpenMP settings */
+EXE_INC += -fopenmp
+EXE_LIBS += -fopenmp
```
### `Make/options`の編集（OSMesa版）
OSMesa版のkVSを使用する場合には以下のとおり編集して下さい

```diff
EXE_INC = \
（数行省略）
    -I$(LIB_SRC)/regionFaModels/lnInclude \
+    -I$(HOME)/local/openmpi-4.1.2/include \

EXE_LIBS = \
（省略）

+LLVM_LIB = $(HOME)/local/llvm_15.0.7/lib

+/* KVS settings */
+EXE_INC += \
+        -I${KVS_DIR}/include -DKVS_SUPPORT_MPI -DKVS_USE_MPI\
+        -I${KVS_OSMESA_DIR}/include -DKVS_SUPPORT_OSMESA
+EXE_LIBS += \
+    -L$(LLVM_LIB) \
+        -L${KVS_DIR}/lib -lkvsSupportMPI -lkvsSupportOSMesa -lkvsCore \
+        -L${KVS_OSMESA_DIR}/lib/x86_64-linux-gnu ${KVS_OSMESA_LINK_LIBRARY} \
+        -L$(KVS_LIB_DIR) -lkvs

+/* InSitu settings */
+EXE_INC += -I$(HOME)/Work/Github
+EXE_LIBS += -L$(HOME)/Work/Github/InSituVis/Lib -lInSituVis

+/* OpenMP settings */
+EXE_INC += -fopenmp
+EXE_LIBS += -fopenmp
```

### ビルドの実行
terminalで
``` 
$ wclean && wmake
```
でビルドします．エラーが出なければ成功です．

## OralAirFlowVisの可視化
可視化をします．ここでは例としてOralAirFlowVisに含まれるrealistic-s3を使用します．
``` 
$ cd ~/Work/GitHub/OralAirFlowVis/realistic-s3
```

### 解析ファイルの編集
OralAirFlowVisはOpenFOAM 2.3.1向けの記述が残っているため，そのままでは動きません．OpenFOAM v2412で動作するように解析ファイルにも手を入れる必要があります．
constant/turbulencePropertiesを以下の通り改造します．乱流モデルの設定です．

まずconstant/turbulencePropertiesを以下のとおり改造します．
```diff
- simulationType LESModel;
+ simulationType LES;
+ {
+   LESModel WALE;
+   turbulence on;
+   printCoeffs on;
+   delta cubeRootVol;
+}
```
次に初期条件の設定ファイル名を変更します．
``` 
$ rm -r processor*
$ mv 0/alphaSgs 0/alphat
$ mv 0/muSgs 0/nut
```
`./0/nut`を編集します．

```diff
FoamFile
{
    version     2.0;
    format      ascii;
    class       volScalarField;
-    object      muSgs;
+    location    "0";
+    object      nut;
}
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //
-dimensions      [1 -1 -1 0 0 0 0];
+dimensions      [0 2 -1 0 0 0 0];

internalField   uniform 0;

```

`./system/fvSchemes`を以下のとおり編集します．
```diff
divSchemes
{
    default         none;
    div(phi,U)      Gauss LUST grad(U);
    div(phi,e)      Gauss LUST grad(e);
    div(phi,K)      Gauss linear;
    div(phiv,p)     Gauss linear;
    div(phi,k)      Gauss limitedLinear 1;
    div(phi,B)      Gauss limitedLinear 1;
    div(phi,muTilda) Gauss limitedLinear 1;
    div(B)          Gauss linear;
    div((muEff*dev2(T(grad(U))))) Gauss linear;
    div(div(((rho*U)*U)))  Gauss linear;
    div(((rho*U)*U))   Gauss linear;
+    div(((rho*nuEff)*dev2(T(grad(U))))) Gauss linear;
}
```
### 実行
並列計算向けにメッシュ分割を行います
``` 
$ decomposePar
```

`run_insitu.sh`を以下の内容に編集します．
```bash
#!/bin/sh

#unset FOAM_SIGFPE
export FOAM_SIGFPE=false
mpirun -n 8 ../my_rhoPimpleFoam/rhoPimpleFoam -parallel
```

実行します．
``` 
$ ./run_insitu.sh
```
`Output`に画像が出力されます．

### 実行結果の例

<img width="512" height="512" alt="OralAirFlowVis" src="https://github.com/user-attachments/assets/b6fc3d9b-23ce-46cc-9ca9-892fb14936d5" />

