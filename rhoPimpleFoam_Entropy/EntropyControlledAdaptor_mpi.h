/*****************************************************************************/
/**
 *  @file   EntropyControlledAdaptor_mpi.h
 *  @author Ken Iwata, Naohisa Sakamoto
 */
/*****************************************************************************/
#pragma once
#if defined( KVS_SUPPORT_MPI )
#include <InSituVis/Lib/Adaptor_mpi.h>
#include "EntropyTimestepController.h"
#include <list>
#include <queue>


namespace local
{

namespace mpi
{

class EntropyControlledAdaptor : public InSituVis::mpi::Adaptor, public local::EntropyTimestepController
{
public:
    using BaseClass = InSituVis::mpi::Adaptor;
    using Controller = local::EntropyTimestepController;

public:
    EntropyControlledAdaptor( const MPI_Comm world = MPI_COMM_WORLD, const int root = 0 ): BaseClass( world, root ) {}
    virtual ~EntropyControlledAdaptor() = default;

protected:
    bool isEntropyStep();
    virtual void exec( const BaseClass::SimTime sim_time = {} );
    virtual void execRendering();

private:
    kvs::Vec3 process( const Data& data );
    void process( const Data& data , const InSituVis::Viewpoint& path, const size_t i );
};

} // end of namespace mpi

} // end of namespace local

#include "EntropyControlledAdaptor_mpi.hpp"

#endif // KVS_SUPPORT_MPI
