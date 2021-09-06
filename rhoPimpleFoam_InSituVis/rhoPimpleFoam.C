/*---------------------------------------------------------------------------* \
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     |
    \\  /    A nd           | Copyright (C) 2011-2013 OpenFOAM Foundation
     \\/     M anipulation  |
-------------------------------------------------------------------------------
License
    This file is part of OpenFOAM.

    OpenFOAM is free software: you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    OpenFOAM is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
    for more details.

    You should have received a copy of the GNU General Public License
    along with OpenFOAM.  If not, see <http://www.gnu.org/licenses/>.

Application
    rhoPimpleFoam

Description
    Transient solver for laminar or turbulent flow of compressible fluids
    for HVAC and similar applications.

    Uses the flexible PIMPLE (PISO-SIMPLE) solution for time-resolved and
    pseudo-transient simulations.

\*---------------------------------------------------------------------------*/

#include "fvCFD.H"
#include "psiThermo.H"
#include "turbulenceModel.H"
#include "bound.H"
#include "pimpleControl.H"
#include "fvIOoptionList.H"

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

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //



int main(int argc, char *argv[])
{
    #include "setRootCase.H"
    #include "createTime.H"
    #include "createMesh.H"

    pimpleControl pimple(mesh);

    #include "createFields.H"
    #include "createFvOptions.H"
    #include "initContinuityErrs.H"

#if defined( IN_SITU_VIS )
    // In-situ visualization setup
    Foam::messageStream::level = 0; // Disable Foam::Info
    const kvs::Indent indent(4); // indent for log stream
    local::InSituVis vis( MPI_COMM_WORLD );
    vis.importBoundaryMesh( "./constant/triSurface/realistic-cfd3.stl" );
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

    while (runTime.run())
    {
        #include "readTimeControls.H"
        #include "compressibleCourantNo.H"
        #include "setDeltaT.H"

        runTime++;

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

        if (pimple.nCorrPIMPLE() <= 1)
        {
            #include "rhoEqn.H"
        }

        // --- Pressure-velocity PIMPLE corrector loop
        while (pimple.loop())
        {
            #include "UEqn.H"
            #include "EEqn.H"

            // --- Pressure corrector loop
            while (pimple.correct())
            {
                #include "pEqn.H"
            }

            if (pimple.turbCorr())
            {
                turbulence->correct();
            }
        }

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
        // A
//        const auto min_value = 0.99998 * 100000.0;
//        const auto max_value = 1.02000 * 100000.0;
        // B
//        const auto min_value = 9.94 * 10000.0;
//        const auto max_value = 1.02 * 100000.0;
        // C
        const auto min_value = 0.999999 * 100000.0;
        const auto max_value = 1.000020 * 100000.0;
#elif defined( IN_SITU_VIS__U ) // U: velocity
        auto& field = U;
        // A (whole min/max values)
        const auto min_value = 0.0;
        const auto max_value = 71.645393372;
//        const auto min_value = 0.0224;
//        const auto max_value = 70.9;
#elif defined( IN_SITU_VIS__T ) // T: temperature
        auto& field = thermo.T();
        // A (whole min/max values)
//        const auto min_value = 0.0;
//        const auto max_value = 296.15917969;
        const auto min_value = 290.0;
        const auto max_value = 296.16;
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

        Info<< "ExecutionTime = " << runTime.elapsedCpuTime() << " s"
            << "  ClockTime = " << runTime.elapsedClockTime() << " s"
            << nl << endl;
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


// ************************************************************************* //
