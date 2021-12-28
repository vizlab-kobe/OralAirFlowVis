/*****************************************************************************/
/**
 *  @file   TimestepControlledAdaptor_mpi.h
 *  @author Naohisa Sakamoto
 */
/*****************************************************************************/
#pragma once
#if defined( KVS_SUPPORT_MPI )
#include "Adaptor_mpi.h"
#include "AdaptiveTimestepController.h"
#include <list>
#include <queue>


namespace local
{

namespace mpi
{

class TimestepControlledAdaptor : public local::mpi::Adaptor, public local::AdaptiveTimestepController
{
public:
    using BaseClass = local::mpi::Adaptor;
    using Controller = local::AdaptiveTimestepController;

public:
    TimestepControlledAdaptor( const MPI_Comm world = MPI_COMM_WORLD, const int root = 0 ): BaseClass( world, root ) {}
    virtual ~TimestepControlledAdaptor() = default;

    virtual void exec( const BaseClass::SimTime sim_time = {} );

private:
    kvs::Vec3 calcProcess( const Data& data );
    void process( const Data& data , const InSituVis::Viewpoint& path, const size_t i );
};

} // end of namespace mpi

} // end of namespace InSituVis

#include "TimestepControlledAdaptor_mpi.hpp"

#endif // KVS_SUPPORT_MPI
