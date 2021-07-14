# In-situ Visualization for Oral Airflow Simulation based on OpenFOAM

## Instaltion

### KVS
KVS supports OSMesa and MPI needs to be installed.

1. Install OSMesa and MPI

    OSMesa and MPI need to be install before compile the KVS. Please refer to the followin URLs to install them.<br>
    - [OSMesa](https://github.com/naohisas/KVS/blob/develop/Source/SupportOSMesa/README.md)
    - [MPI](https://github.com/naohisas/KVS/blob/develop/Source/SupportMPI/README.md)

2. Get KVS source codes from the GitHub repository as follows:
    ```
    $ git clone https://github.com/naohisas/KVS.git
    ```

3. Modify kvs.conf as follows:
    ```
    $ cd KVS
    $ <modify kvs.conf>
    ```

    - Change the following items from 0(disable) to 1(enable).<br>
    ```
    KVS_ENABLE_OPENGL     = 1
    KVS_SUPPORT_MPI       = 1
    KVS_SUPPORT_OSMESA    = 1
    ```
    - Change the following items from 1(enable) to 0(disable).<br>
    ```
    KVS_SUPPORT_GLU       = 0
    KVS_SUPPORT_GLUT      = 0
    ```
    - Change the following items to 1 if needed. <br>
    ```
    KVS_ENABLE_OPENMP     = 1
    KVS_SUPPORT_PYTHON    = 1
    ```

4. Compile and install the KVS
    ```
    $ make
    $ make install
    ```

### InSituVis
InSituVis is required.

1. Get the InSituVis source codes from the GitHub repository as follows:
    ```
    $ git clone https://github.com/vizlab-kobe/InSituVis.git
    ```

2. Move to Lib directory.
    ```
    $ cd InSituVis/Lib
    ```

3. Compile the InSituVis library.
    ```
    $ ./kvsmake.py
    ```

### OpenFOAM
OpenFOAM 2.3.1 is required.

1. Get OpenFOAM 2.3.1 source codes from the following URL.

    - [openfoam321.tar.gz](https://www.dropbox.com/s/aa8azaz2jt0inta/openfoam231.tar.gz?dl=0)

2. Compile the OpenFOAM
    ```
    $ cd <download directory>
    $ tar -zxvf openfoam231.tar.gz 
    * Change 'g++' in wmake/rules/linux64Gcc/c++ to 'mpicxx'
    $ cd OpenFOAM/OpenFOAM-2.3.1
    $ source etc/bashrc
    $ cd src/Pstream/mpi 
    $ wclean &wmake 
    $ cd ../../../
    $ ./Allwmake
    ```

## Execution

### OralAirFlowVis

1. Get the OralAirFlowVis source codes from the GitHub repository as follows:
    ```
    $ git clone https://github.com/vizlab-kobe/OralAirFlowVis.git
    ```

2. Move to the OralAirFlowVis directory.
    ```
    $ cd OralAirflowVis
    ```

### rhoPimpleFoam_InSituVis
The compilation and execuation of the application are done in a separate terminal.

- Compilation<br>
    ```
    $ cd rhoPimpleFoam_InSituVis
    $ ./make.sh
    * 'clear.sh' is a shell script for removing the compiled files.
    ```

- Execution<br>
    ```
    $ cd realistic-s3
    $ ./run_insitu.sh
    * The rendering results will be output in the 'Output' directory.
    * 'realistic-s3' is a 8-parallel version of the application program.
    * 'realistic-s1' and 'realistic-s2' are 4- and 48-parallel versions.
    * 'clear.sh' is a shell script for removing the output directory.
    ```

### rhoPimpleFoam
The rhoPimpleFoam is an original program based OpenFOAM for the oral airflow simulation. This program can be compiled and executed same as the rhoPimpleFoam_InSituVis program.

- Compilation<br>
    ```
    $ cd rhoPimpleFoam
    $ ./make.sh
    ```

- Execution<br>
    ```
    $ cd realistic-s3
    $ ./run.sh
    ```
